/* ============================================================
   RADAR MANAGER — IMPLEMENTAÇÃO v3.1
   ------------------------------------------------------------
   @file      radar_manager.c
   @brief     Driver com descodificação de sinal HLK-LD2450.

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Correcções v2.9 → v3.0:
   -------------------------
   CORRECÇÃO 6 — BUG CRÍTICO: sinal de velocidade perdido
     O HLK-LD2450 reporta velocidade em cm/s com sinal:
       spd_cms < 0 → objecto a APROXIMAR-SE do sensor
       spd_cms > 0 → objecto a AFASTAR-SE do sensor
     A versão anterior guardava apenas o módulo em tgt->speed,
     perdendo completamente a informação de direcção.
     Consequência: radar_vehicle_in_range() devolvia true para
     qualquer objecto no campo, incluindo carros que já tinham
     passado pelo sensor e ainda estavam no alcance de 8m a
     afastar-se. O debounce s_veiculo_presente ficava true mais
     tempo do que devia, e ao baixar podia disparar detecção falsa.
     Solução: guardar também tgt->speed_signed (km/h com sinal)
     em radar_vehicle_t. O campo speed (módulo) é preservado para
     o display. radar_vehicle_in_range() e radar_get_closest_speed()
     filtram agora por speed_signed <= AFASTAR_THRESHOLD_KMH.

   CORRECÇÃO 7 — BUG: rasto fantasma em radar_manager_get_objects()
     Quando s_last_data.count == 0 (zona vazia), internal_objs[]
     preservava as posições das detecções anteriores indefinidamente.
     Na próxima detecção, _update_trail() iniciava o rasto com
     posições antigas, causando um "salto" visual no canvas.
     Solução: quando count == 0, limpa trail_len de todos os slots.
     A posição x_mm/y_mm é preservada para associação futura mas
     o rasto visual recomeça do zero na próxima detecção.

   CORRECÇÃO 8 — Diagnóstico mostra direcção do alvo
     radar_diagnostic() passa a imprimir o sinal de velocidade:
     "APROX" (a aproximar-se) ou "AFAS" (a afastar-se).
     Facilita verificação da orientação do sensor em campo.

   Correcções v2.8 → v2.9 (mantidas):
   ------------------------------------
   CORRECÇÃO 1 — filtro "y == 0" removido
   CORRECÇÃO 2 — log de diagnóstico corrigido
   CORRECÇÃO 3 — NO_FRAME_LIMIT aumentado de 20 para 100
   CORRECÇÃO 4 — AUTODETECT_ITER aumentado de 20 para 100
   CORRECÇÃO 5 — candidatos[] usa RADAR_BAUD_RATE de hw_config.h

   Dependências:
   -------------
   - radar_manager.h : API pública, tipos, AFASTAR_THRESHOLD_KMH
   - hw_config.h     : RADAR_UART_PORT, RADAR_PIN_TX/RX, RADAR_BAUD_RATE
   - driver/uart.h   : ESP-IDF UART driver
   - freertos/task.h : vTaskDelay, pdMS_TO_TICKS
   - esp_log.h       : ESP_LOGI, ESP_LOGW, ESP_LOGE
============================================================ */

#include "radar_manager.h"
#include "hw_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

/* Etiqueta de log deste módulo */
static const char *TAG = "RADAR_MGR";

/* ============================================================
   CONSTANTES INTERNAS
============================================================ */

/* Comprimento fixo de um frame HLK-LD2450 em bytes */
#define FRAME_LEN       30

/* Tamanho do ring buffer de software para acumulação UART */
#define UART_BUF_SIZE   256

/* Distância máxima válida em milímetros (8 metros).
 * Alvos além deste valor são descartados como ruído. */
#define MAX_DIST_MM     8000

/*
 * CORRECÇÃO 3: NO_FRAME_LIMIT aumentado de 20 para 100.
 * 100 ciclos × 100ms (período da FSM) = 10 segundos sem frame
 * antes de declarar s_last_read_ok = false.
 * O HLK-LD2450 emite frames a 10 Hz mesmo sem alvos —
 * frames com count=0 são válidos e repõem este contador a zero.
 */
#define NO_FRAME_LIMIT  100

/*
 * CORRECÇÃO 4: AUTODETECT_ITER aumentado de 20 para 100.
 * 100 × 50ms = 5000ms por candidato.
 * Garante ~50 frames a 10 Hz por tentativa, cobrindo o período
 * de arranque do oscilador interno do HLK-LD2450 (~500ms).
 */
#define AUTODETECT_ITER      100
#define AUTODETECT_STEP_MS    50
#define AUTODETECT_ACUM_SIZE 512

/* ============================================================
   ESTADO INTERNO DO MÓDULO
============================================================ */

/* Modo de operação actual (UART ou simulado) */
static radar_mode_t s_mode         = RADAR_MODE_DEFAULT;

/* true enquanto o sensor está a responder com frames válidos */
static bool         s_last_read_ok = false;

/* Contador de ciclos consecutivos sem frame válido */
static int          s_no_frame_cnt = 0;

/* Ring buffer de software para acumulação de bytes UART */
static uint8_t      s_ring_buf[UART_BUF_SIZE];
static int          s_ring_len     = 0;

/*
 * Cache do último frame processado com sucesso.
 * Partilhado com radar_manager_get_objects() para que o canvas
 * do display use os mesmos dados que a FSM leu — sem consumir
 * o ring buffer uma segunda vez (o que resultaria em count=0).
 * Limpo no início de cada chamada a radar_read_data().
 */
static radar_data_t s_last_data = {0};

/*
 * Estado persistente para detector de obstáculo estático (v3.1).
 * Um array paralelo a s_last_data.targets[], indexado pelo slot.
 * frames_est[i] : frames consecutivos com alvo i parado.
 * dist_ant[i]   : distância do slot i no frame anterior (mm).
 * Limpo quando count==0 (zona vazia) ou slot muda de alvo.
 */
static uint16_t s_frames_est[MAX_RADAR_TARGETS] = {0};
static int      s_dist_ant[MAX_RADAR_TARGETS]   = {0};

/* ============================================================
   _decode_radar_value
   ------------------------------------------------------------
   Descodifica um valor de 16 bits do protocolo HLK-LD2450.

   Protocolo (Manual pág. 13):
     Bit 15 (MSB do byte alto) = 1 → valor positivo
     Bit 15 (MSB do byte alto) = 0 → valor negativo
     Bits 14..0 = magnitude do valor

   Parâmetros:
     lsb — byte menos significativo (byte de índice par no frame)
     msb — byte mais significativo  (byte de índice ímpar no frame)

   Retorna:
     int16_t — valor com sinal em mm (X, Y) ou cm/s (velocidade)
============================================================ */
static int16_t _decode_radar_value(uint8_t lsb, uint8_t msb)
{
    /* Extrai magnitude dos 15 bits inferiores (ignora bit de sinal) */
    uint16_t magnitude = (uint16_t)lsb | ((uint16_t)(msb & 0x7F) << 8);

    /* Bit 7 do MSB é o bit de sinal: 1 = positivo, 0 = negativo */
    return (msb & 0x80) ? (int16_t)magnitude : -(int16_t)magnitude;
}

/* ============================================================
   _update_trail
   ------------------------------------------------------------
   Actualiza o rasto de um objecto do radar.
   Desloca as posições anteriores uma casa para a direita
   e insere a nova posição no início, mantendo os últimos
   RADAR_TRAIL_MAX pontos para o display desenhar o rastro.
============================================================ */
static void _update_trail(radar_obj_t *obj, int new_x, int new_y)
{
    /* Desloca rasto existente uma posição para a direita */
    for (int i = RADAR_TRAIL_MAX - 1; i > 0; i--) {
        obj->trail_x[i] = obj->trail_x[i - 1];
        obj->trail_y[i] = obj->trail_y[i - 1];
    }

    /* Posição actual passa a ser a primeira entrada do rasto */
    obj->trail_x[0] = obj->x_mm;
    obj->trail_y[0] = obj->y_mm;

    /* Incrementa comprimento do rasto até ao máximo */
    if (obj->trail_len < RADAR_TRAIL_MAX) obj->trail_len++;

    /* Actualiza posição actual com os novos valores */
    obj->x_mm = new_x;
    obj->y_mm = new_y;
}

/* ============================================================
   radar_init
   ------------------------------------------------------------
   Inicializa o driver UART2 para comunicação com o HLK-LD2450,
   ou prepara o modo simulado (sem hardware).

   Modo UART: instala driver com buffer HW de UART_BUF_SIZE×2,
   configura 8N1 sem controlo de fluxo, e mapeia os pinos GPIO
   definidos em hw_config.h.
============================================================ */
void radar_init(radar_mode_t mode)
{
    s_mode = mode;

    if (mode == RADAR_MODE_UART)
    {
        uart_config_t uart_config = {
            .baud_rate  = RADAR_BAUD_RATE,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_driver_install(
            RADAR_UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(
            RADAR_UART_PORT,
            RADAR_PIN_TX, RADAR_PIN_RX,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        ESP_LOGI(TAG,
                 "Radar iniciado: UART%d | TX=GPIO%d RX=GPIO%d | %d baud",
                 RADAR_UART_PORT, RADAR_PIN_TX, RADAR_PIN_RX,
                 RADAR_BAUD_RATE);
    }
    else
    {
        ESP_LOGI(TAG, "Radar: modo simulado activo");
    }
}

/* ============================================================
   radar_flush_rx
   ------------------------------------------------------------
   Descarta todos os bytes acumulados no buffer HW da UART
   e limpa o ring buffer de software.
   Chamar após o delay de arranque da fsm_task para descartar
   dados acumulados durante o auto-detect e o boot.
   Ignorado em modo simulado.
============================================================ */
void radar_flush_rx(void)
{
    if (s_mode == RADAR_MODE_UART)
    {
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;
        memset(&s_last_data, 0, sizeof(s_last_data));
        ESP_LOGI(TAG, "RX UART limpo (backlog descartado)");
    }
}

/* ============================================================
   radar_read_data
   ------------------------------------------------------------
   Lê bytes da UART, acumula no ring buffer de software e
   procura frames válidos HLK-LD2450.

   Estrutura do frame HLK-LD2450 (30 bytes):
     Bytes  [0..3]  : AA FF 03 00        (cabeçalho fixo)
     Bytes  [4..11] : alvo 1 — X(2) Y(2) Vel(2) Dist(2)
     Bytes [12..19] : alvo 2
     Bytes [20..27] : alvo 3
     Bytes [28..29] : 55 CC              (rodapé fixo)

   CORRECÇÃO 1 (v2.9) — Filtro de slot vazio:
     Antes: if (dist_mm == 0 || dist_mm >= MAX_DIST_MM || y == 0)
     O "y == 0" descartava o alvo quando passava exactamente em
     frente ao sensor (Y=0mm) — o momento mais crítico.
     Agora:  if (dist_mm == 0 || dist_mm >= MAX_DIST_MM)
     dist_mm == 0 identifica slots verdadeiramente vazios.
     Y pode ser zero, negativo ou positivo — é posição válida.

   Retorna:
     true  — encontrou e processou pelo menos 1 frame válido
     false — sem frame neste ciclo
============================================================ */
bool radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;

    /* Limpa cache desta leitura */
    memset(&s_last_data, 0, sizeof(s_last_data));
    memset(out_data,     0, sizeof(radar_data_t));

    /* ── Modo simulado ── */
    if (s_mode == RADAR_MODE_SIMULATED)
    {
        if (sim_input && sim_input->active)
        {
            out_data->count               = 1;
            out_data->targets[0].y_mm     = sim_input->distance;
            out_data->targets[0].distance = (float)sim_input->distance / 1000.0f;
            out_data->targets[0].detected = true;
            out_data->targets[0].speed    = 0.0f;
            s_last_data = *out_data;
            return true;
        }
        return false;
    }

    /* ── Lê bytes novos da UART (timeout 10ms) ── */
    uint8_t temp_rx[64];
    int len = uart_read_bytes(
        RADAR_UART_PORT, temp_rx, sizeof(temp_rx), pdMS_TO_TICKS(10));

    if (len > 0)
    {
        /*
         * Gestão de overflow do ring buffer:
         * Preserva os últimos (FRAME_LEN-1) = 29 bytes quando cheio —
         * podem ser o início de um frame ainda incompleto.
         */
        if (s_ring_len + len >= UART_BUF_SIZE)
        {
            int keep = FRAME_LEN - 1;
            if (s_ring_len > keep)
            {
                memmove(s_ring_buf,
                        &s_ring_buf[s_ring_len - keep],
                        (size_t)keep);
                s_ring_len = keep;
            }
        }
        memcpy(&s_ring_buf[s_ring_len], temp_rx, (size_t)len);
        s_ring_len += len;
    }

    /* ── Procura frame válido no ring buffer ── */
    int found_idx = -1;
    for (int i = 0; i <= (s_ring_len - FRAME_LEN); i++)
    {
        if (s_ring_buf[i]    == 0xAA && s_ring_buf[i+1]  == 0xFF &&
            s_ring_buf[i+2]  == 0x03 && s_ring_buf[i+3]  == 0x00 &&
            s_ring_buf[i+28] == 0x55 && s_ring_buf[i+29] == 0xCC)
        {
            found_idx = i;
            break;
        }
    }

    if (found_idx >= 0)
    {
        uint8_t *f = &s_ring_buf[found_idx];
        out_data->count = 0;

        /* Processa os 3 slots de alvo do frame */
        for (int t = 0; t < 3; t++)
        {
            int base = 4 + (t * 8);

            int16_t  x       = _decode_radar_value(f[base],   f[base+1]);
            int16_t  y       = _decode_radar_value(f[base+2], f[base+3]);
            int16_t  spd_cms = _decode_radar_value(f[base+4], f[base+5]);
            uint16_t dist_mm = (uint16_t)f[base+6]
                             | ((uint16_t)f[base+7] << 8);

            /*
             * CORRECÇÃO 1 (v2.9) — filtro de slot vazio corrigido.
             * dist_mm == 0 → slot vazio (sem alvo neste slot).
             * dist_mm >= MAX_DIST_MM → além do alcance útil (ruído).
             * Y pode ser qualquer valor incluindo 0 ou negativo —
             * não é critério de filtragem válido.
             */
            if (dist_mm == 0 || dist_mm >= MAX_DIST_MM) continue;

            radar_vehicle_t *tgt = &out_data->targets[out_data->count];
            tgt->x_mm     = (int)x;
            tgt->y_mm     = (int)y;
            tgt->distance = (float)dist_mm / 1000.0f;

            /*
             * CORRECÇÃO 6 (v3.0) — preservar sinal de velocidade.
             *
             * O HLK-LD2450 reporta velocidade em cm/s com sinal:
             *   spd_cms < 0 → objecto a APROXIMAR-SE do sensor
             *   spd_cms > 0 → objecto a AFASTAR-SE do sensor
             *   spd_cms = 0 → objecto parado ou velocidade nula
             *
             * Conversão: 1 cm/s = 0.036 km/h
             *
             * speed        : módulo em km/h (sempre >= 0)
             *                Usado pelo display e para cálculo de ETA.
             * speed_signed : km/h com sinal (negativo = a aproximar)
             *                Usado por radar_vehicle_in_range() para
             *                filtrar objectos a afastar-se.
             */
            tgt->speed_signed = (float)spd_cms * 0.036f;
            tgt->speed        = (tgt->speed_signed < 0.0f)
                                ? -tgt->speed_signed
                                :  tgt->speed_signed;

            /*
             * Detector de obstáculo estático (v3.1).
             * Condição de "parado":
             *   speed < OBSTACULO_SPEED_MAX_KMH  — velocidade baixa
             *   |dist_mm - dist_anterior| < OBSTACULO_DIST_TOL_MM — posição estável
             * Se ambas verdadeiras: incrementa contador do slot.
             * Se qualquer uma falhar: reset do contador.
             * O slot é identificado pelo índice t (0,1,2) — mesma
             * correspondência frame a frame usada pelo HLK-LD2450.
             */
            int dist_i = (int)dist_mm;
            if (tgt->speed < OBSTACULO_SPEED_MAX_KMH &&
                abs(dist_i - s_dist_ant[t]) < OBSTACULO_DIST_TOL_MM)
            {
                if (s_frames_est[t] < 0xFFFF) s_frames_est[t]++;
            }
            else
            {
                s_frames_est[t] = 0;
            }
            s_dist_ant[t]         = dist_i;
            tgt->frames_estaticos = s_frames_est[t];
            tgt->dist_mm_anterior = s_dist_ant[t];

            tgt->detected = true;
            out_data->count++;
        }

        /* Consome o frame processado do ring buffer */
        int consumed = found_idx + FRAME_LEN;
        memmove(s_ring_buf,
                &s_ring_buf[consumed],
                (size_t)(s_ring_len - consumed));
        s_ring_len -= consumed;

        /* Actualiza cache partilhada e estado de saúde */
        s_last_data    = *out_data;
        s_last_read_ok = true;
        s_no_frame_cnt = 0;
        return true;
    }

    /* Sem frame válido neste ciclo — incrementa contador de falha */
    s_no_frame_cnt++;
    if (s_no_frame_cnt > NO_FRAME_LIMIT)
    {
        s_last_read_ok = false;
    }
    return false;
}

/* ============================================================
   radar_manager_get_objects
   ------------------------------------------------------------
   Fornece ao display_manager os objectos do radar com rasto.
   Lê da cache s_last_data em vez de chamar radar_read_data()
   novamente — evita consumir o ring buffer duas vezes por ciclo
   (o que resultaria sempre em count=0 no canvas).

   CORRECÇÃO 7 (v3.0) — rasto fantasma eliminado:
   Quando s_last_data.count == 0 (zona vazia, sem alvos),
   os internal_objs[] eram preservados com posições antigas.
   Na próxima detecção, _update_trail() iniciava com esses
   pontos, causando um "salto" visual no canvas entre a última
   posição antiga e a nova posição real.
   Solução: quando count == 0, reset de trail_len em todos os
   slots. A posição x_mm/y_mm é preservada para que a
   associação por proximidade do display ainda funcione se o
   alvo reaparecer rapidamente, mas o rasto visual recomeça.
============================================================ */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max)
{
    /* Estado de rasto persistente entre chamadas */
    static radar_obj_t internal_objs[MAX_RADAR_TARGETS] = {0};

    if (s_last_data.count == 0)
    {
        /* Zona vazia — limpa rasto e contadores de obstáculo */
        for (int i = 0; i < MAX_RADAR_TARGETS; i++)
        {
            internal_objs[i].trail_len = 0;
            s_frames_est[i] = 0;
            s_dist_ant[i]   = 0;
        }
        return 0;
    }

    uint8_t count = 0;
    for (int i = 0;
         i < s_last_data.count && i < (int)max && i < MAX_RADAR_TARGETS;
         i++)
    {
        _update_trail(&internal_objs[i],
                      s_last_data.targets[i].x_mm,
                      s_last_data.targets[i].y_mm);
        internal_objs[i].speed_kmh = s_last_data.targets[i].speed;
        objs[i] = internal_objs[i];
        count++;
    }
    return count;
}

/* ============================================================
   radar_auto_detect_baud
   ------------------------------------------------------------
   Detecta automaticamente o baud rate do HLK-LD2450.

   CORRECÇÃO 5 (v2.9): candidatos[] usa RADAR_BAUD_RATE de
   hw_config.h — fonte única de verdade para o baud rate.
   Elimina redundância e garante consistência automática.

   CORRECÇÃO 4 (v2.9): AUTODETECT_ITER = 100 (5000ms).
   Garante ~50 frames a 10 Hz por tentativa.

   CORRECÇÃO 2 (v2.9): log de diagnóstico de falha corrigido.
   bytes_recebidos[] tem dimensão 1 (n_cand=1). O log anterior
   acedia a [1] e [2] — undefined behaviour em C.

   Retorna:
     Baud rate detectado, ou 0 se não funcionar.
============================================================ */
int radar_auto_detect_baud(void)
{
    if (s_mode != RADAR_MODE_UART) return 0;

    /*
     * CORRECÇÃO 5: candidatos[] usa RADAR_BAUD_RATE de hw_config.h.
     * n_cand = 1 — só um candidato, bytes_recebidos[1].
     */
    static const int candidatos[] = { RADAR_BAUD_RATE };
    static const int n_cand = 1;
    int bytes_recebidos[1] = {0};   /* dimensão exacta = n_cand */

    for (int c = 0; c < n_cand; c++)
    {
        int baud = candidatos[c];
        ESP_LOGI(TAG, "Auto-detect: a tentar %d baud (%dms)...",
                 baud, AUTODETECT_ITER * AUTODETECT_STEP_MS);

        uart_set_baudrate(RADAR_UART_PORT, baud);
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;

        /* Buffer de acumulação para esta tentativa */
        static uint8_t acum[AUTODETECT_ACUM_SIZE];
        uint8_t        tmp[128];
        int            acum_len   = 0;
        bool           encontrado = false;

        /*
         * CORRECÇÃO 4: 100 iterações × 50ms = 5000ms por baud.
         * ~50 frames a 10 Hz — cobre arranque do oscilador interno.
         */
        for (int iter = 0; iter < AUTODETECT_ITER && !encontrado; iter++)
        {
            vTaskDelay(pdMS_TO_TICKS(AUTODETECT_STEP_MS));

            int n = uart_read_bytes(RADAR_UART_PORT, tmp, sizeof(tmp),
                                    pdMS_TO_TICKS(10));
            if (n > 0 && acum_len + n < AUTODETECT_ACUM_SIZE)
            {
                memcpy(&acum[acum_len], tmp, (size_t)n);
                acum_len += n;
                bytes_recebidos[c] += n;
            }

            /* Procura frame válido no buffer acumulado */
            for (int i = 0; i <= acum_len - FRAME_LEN; i++)
            {
                if (acum[i]    == 0xAA && acum[i+1]  == 0xFF &&
                    acum[i+2]  == 0x03 && acum[i+3]  == 0x00 &&
                    acum[i+28] == 0x55 && acum[i+29] == 0xCC)
                {
                    encontrado = true;
                    break;
                }
            }
        }

        if (encontrado)
        {
            ESP_LOGI(TAG, "Baud rate detectado: %d (%d bytes lidos)",
                     baud, bytes_recebidos[c]);
            uart_flush_input(RADAR_UART_PORT);
            s_ring_len = 0;
            return baud;
        }

        ESP_LOGW(TAG, "  %d baud: sem frame (%d bytes recebidos)",
                 baud, bytes_recebidos[c]);
    }

    /*
     * CORRECÇÃO 2 (v2.9) — log de diagnóstico corrigido.
     * bytes_recebidos[] tem dimensão 1 — acesso apenas a [0].
     * Versão anterior acedia a [1] e [2] — undefined behaviour.
     */
    int total_bytes = bytes_recebidos[0];

    ESP_LOGE(TAG, "=== AUTO-DETECT FALHOU ===");
    ESP_LOGE(TAG, "Bytes recebidos: %d | baud tentado: %d",
             bytes_recebidos[0], RADAR_BAUD_RATE);

    if (total_bytes == 0)
    {
        /* Nenhum byte → problema físico */
        ESP_LOGE(TAG, "DIAGNOSTICO: 0 bytes recebidos.");
        ESP_LOGE(TAG, "  Causa provavel: RX/TX invertidos"
                      " (GPIO%d <-> GPIO%d)",
                 RADAR_PIN_TX, RADAR_PIN_RX);
        ESP_LOGE(TAG, "  Ou: sensor sem alimentacao (verificar 5V).");
        ESP_LOGE(TAG, "  Accao: trocar TX e RX fisicamente e reiniciar.");
    }
    else
    {
        /* Chegaram bytes mas sem frame válido */
        ESP_LOGW(TAG, "DIAGNOSTICO: %d bytes recebidos mas sem frame.",
                 total_bytes);
        ESP_LOGW(TAG, "  Causa: baud rate incorrecto ou sensor em"
                      " modo de configuracao.");
        ESP_LOGW(TAG, "  Verificar RADAR_BAUD_RATE em hw_config.h");
        ESP_LOGW(TAG, "  O baud de fabrica do HLK-LD2450 e 256000.");
    }

    /* Repõe fallback seguro definido em hw_config.h */
    ESP_LOGW(TAG, "Fallback: %d baud — radar em FAIL ate frame valido",
             RADAR_BAUD_RATE);
    uart_set_baudrate(RADAR_UART_PORT, RADAR_BAUD_RATE);
    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;
    return 0;
}

/* ============================================================
   radar_get_status_str
   "REAL" → modo UART, a receber frames válidos
   "SIM"  → modo simulado activo
   "FAIL" → modo UART, sem frames válidos (sensor offline)
============================================================ */
const char *radar_get_status_str(void)
{
    if (s_mode == RADAR_MODE_SIMULATED) return "SIM";
    return s_last_read_ok ? "REAL" : "FAIL";
}

/* ============================================================
   Getters utilitários
============================================================ */

bool radar_is_connected(void)
{
    return s_last_read_ok;
}

/* ============================================================
   radar_vehicle_in_range
   ------------------------------------------------------------
   CORRECÇÃO 6 (v3.0) — filtra por direcção de movimento.

   Anteriormente: devolvia true para qualquer objecto no campo,
   incluindo carros que já tinham passado e ainda estavam nos
   8m de alcance a afastar-se. O debounce s_veiculo_presente
   ficava true mais tempo, e ao baixar podia causar detecção falsa.

   Agora: devolve true apenas se pelo menos um alvo tem
   speed_signed <= AFASTAR_THRESHOLD_KMH (a aproximar-se ou parado).
   Objectos com speed_signed > threshold são ignorados — "já passou".

   Nota sobre velocidade zero:
   Se speed_signed == 0 (sensor não reportou movimento), o alvo
   é considerado válido — pode ser um carro lento ou parado.
   Só é descartado quando claramente a afastar-se (> threshold).
============================================================ */
bool radar_vehicle_in_range(const radar_data_t *data)
{
    if (!data || data->count == 0) return false;

    for (int i = 0; i < data->count; i++)
    {
        /*
         * Inclui o alvo se:
         *   - speed_signed <= 0          : a aproximar-se ou parado
         *   - speed_signed <= THRESHOLD  : velocidade positiva mas
         *     dentro da margem de tolerância (micro-oscilações)
         */
        if (data->targets[i].speed_signed <= AFASTAR_THRESHOLD_KMH)
            return true;
    }
    return false;  /* todos os alvos claramente a afastar-se */
}

/* ============================================================
   radar_get_closest_speed
   ------------------------------------------------------------
   Devolve o módulo de velocidade do alvo mais próximo que se
   está a aproximar (speed_signed <= AFASTAR_THRESHOLD_KMH).

   CORRECÇÃO 6 (v3.0): ignora alvos a afastar-se — apenas o
   alvo mais próximo E a aproximar-se conta para a FSM.
   Retorna 0.0 se todos os alvos se estão a afastar.
============================================================ */
float radar_get_closest_speed(const radar_data_t *data)
{
    if (!data || data->count == 0) return 0.0f;

    float min_dist = 999.0f;
    float speed    = 0.0f;

    for (int i = 0; i < data->count; i++)
    {
        /* Ignora alvos claramente a afastar-se */
        if (data->targets[i].speed_signed > AFASTAR_THRESHOLD_KMH)
            continue;

        if (data->targets[i].distance < min_dist)
        {
            min_dist = data->targets[i].distance;
            speed    = data->targets[i].speed;  /* módulo para ETA */
        }
    }
    return speed;
}

radar_mode_t radar_get_mode(void)
{
    return s_mode;
}

/* ============================================================
   radar_static_object_present  (v3.1)
   ------------------------------------------------------------
   Verifica se há obstáculo estático persistente no campo.

   Algoritmo:
     Para cada alvo no frame actual, verifica se frames_estaticos
     atingiu OBSTACULO_MIN_FRAMES (calculado em radar_read_data).
     Se sim, é um alvo que esteve parado na mesma posição durante
     pelo menos OBSTACULO_MIN_FRAMES × 100ms = 8 segundos.

   Distinção obstáculo real vs falso positivo:
     Pássaro/folha: desaparece em 1-3 segundos → frames_est < 80
     Carro avariado: permanece minutos → frames_est >> 80
     Pedaço de pneu: pode não ser detectado (baixa reflectividade)
       → count=0, não chega a esta função.

   Limitação documentada:
     Objectos de baixa reflectividade radar (detritos pequenos,
     borracha, plástico fino) podem não ser detectados pelo
     HLK-LD2450 — limitação intrínseca do radar Doppler/FMCW.
     O sistema garante detecção de veículos e objectos metálicos.
============================================================ */
bool radar_static_object_present(const radar_data_t *data)
{
    if (!data || data->count == 0) return false;
    if (s_mode != RADAR_MODE_UART) return false;

    for (int i = 0; i < data->count; i++)
    {
        if (data->targets[i].frames_estaticos >= OBSTACULO_MIN_FRAMES)
            return true;
    }
    return false;
}

/* ============================================================
   radar_diagnostic
   ------------------------------------------------------------
   Janela de diagnóstico interactiva de 8 segundos.
   Imprime coordenadas, distância e velocidade de cada alvo.
   Não é chamada no arranque normal — disponível para uso
   manual se necessário para confirmar o sensor.
   Chamar APÓS radar_auto_detect_baud() se necessário.
============================================================ */
void radar_diagnostic(void)
{
    if (s_mode != RADAR_MODE_UART) return;

#define DIAG_DURATION_S  8
#define DIAG_CICLO_MS    100

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  DIAGNOSTICO RADAR — %d segundos", DIAG_DURATION_S);
    ESP_LOGI(TAG, "  Passe a mao (ou braco) a frente do sensor...");
    ESP_LOGI(TAG, "================================================");

    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;

    int frames_ok  = 0;
    int ciclos_sem = 0;
    int total      = (DIAG_DURATION_S * 1000) / DIAG_CICLO_MS;

    for (int i = 0; i < total; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(DIAG_CICLO_MS));

        radar_data_t d = {0};
        bool ok = radar_read_data(&d, NULL);

        if (ok)
        {
            frames_ok++;
            ciclos_sem = 0;

            if (d.count == 0)
            {
                ESP_LOGI(TAG, "[DIAG] Frame OK — sem alvos (zona livre)");
            }
            else
            {
                for (int t = 0; t < d.count; t++)
                {
                    /* CORRECÇÃO 8 (v3.0): mostra direcção do alvo.
                     * Facilita verificação da orientação do sensor. */
                    const char *dir = (d.targets[t].speed_signed <= AFASTAR_THRESHOLD_KMH)
                                      ? "APROX" : "AFAS";
                    ESP_LOGI(TAG,
                        "[DIAG] Alvo %d | X=%5dmm  Y=%5dmm"
                        "  Dist=%.2fm  Vel=%.1fkm/h [%s]",
                        t + 1,
                        d.targets[t].x_mm,
                        d.targets[t].y_mm,
                        d.targets[t].distance,
                        d.targets[t].speed,
                        dir);
                }
            }
        }
        else
        {
            ciclos_sem++;
            if (ciclos_sem % 10 == 0)
            {
                ESP_LOGW(TAG, "[DIAG] %d ciclos sem frame...",
                         ciclos_sem);
            }
        }
    }

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  RESULTADO: %d frames validos em %ds",
             frames_ok, DIAG_DURATION_S);

    if (frames_ok > 0)
    {
        s_last_read_ok = true;
        s_no_frame_cnt = 0;
        ESP_LOGI(TAG, "  RADAR OK — sensor a funcionar correctamente");
    }
    else
    {
        ESP_LOGE(TAG, "  RADAR FAIL — nenhum frame recebido");
        ESP_LOGE(TAG,
                 "  Verificar: divisor tensao TX | GND comum | 5V");
    }
    ESP_LOGI(TAG, "================================================");

    /* Limpa buffer antes da FSM começar a ler */
    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;

#undef DIAG_DURATION_S
#undef DIAG_CICLO_MS
}