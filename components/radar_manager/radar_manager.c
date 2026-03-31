/* ============================================================
   RADAR MANAGER — IMPLEMENTAÇÃO v2.7
   ------------------------------------------------------------
   @file      radar_manager.c
   @brief     Driver com descodificação de sinal HLK-LD2450.

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno

   Correcções v2.7:
   - Ring buffer overflow: guarda tail do frame anterior em vez
     de descartar TUDO (incluindo os bytes novos) — causa raiz
     do "sem frame válido" mesmo com radar ligado.
   - Velocidade: HLK-LD2450 reporta em cm/s → convertido para
     km/h (× 0.036) antes de ser armazenado na struct.
   - Cache s_last_data: radar_manager_get_objects() usa o último
     frame lido por radar_read_data() em vez de chamar
     radar_read_data() de novo (evitava dupla leitura do mesmo
     ring buffer, resultando em 0 objectos no canvas).
   - radar_is_connected() reseta para false após NO_FRAME_LIMIT
     ciclos consecutivos sem frame válido.
   - radar_flush_rx(): limpa backlog UART acumulado durante o
     delay de arranque da fsm_task.
   ============================================================ */

#include "radar_manager.h"
#include "hw_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RADAR_MGR";

#define FRAME_LEN       30      /* bytes por frame HLK-LD2450          */
#define UART_BUF_SIZE   256     /* ring buffer SW                       */
#define MAX_DIST_MM     8000    /* filtra alvos > 8 m                   */
#define NO_FRAME_LIMIT  20      /* ciclos sem frame → connected = false */

static radar_mode_t s_mode         = RADAR_MODE_DEFAULT;
static bool         s_last_read_ok = false;
static int          s_no_frame_cnt = 0;
static uint8_t      s_ring_buf[UART_BUF_SIZE];
static int          s_ring_len     = 0;

/* Cache partilhado com radar_manager_get_objects().
 * Actualizado a cada frame bem sucedido; limpo no início de
 * cada chamada a radar_read_data() para que o display mostre
 * "sem veículo" quando o frame não tiver alvos. */
static radar_data_t s_last_data = {0};

/* ----------------------------------------------------------------
   Auxiliares de descodificação (Manual HLK-LD2450 pág. 13)
   Bit 15 (MSB) = 1 → Positivo | = 0 → Negativo
---------------------------------------------------------------- */
static int16_t _decode_radar_value(uint8_t lsb, uint8_t msb)
{
    uint16_t magnitude = (uint16_t)lsb | ((uint16_t)(msb & 0x7F) << 8);
    return (msb & 0x80) ? (int16_t)magnitude : -(int16_t)magnitude;
}

static void _update_trail(radar_obj_t *obj, int new_x, int new_y)
{
    for (int i = RADAR_TRAIL_MAX - 1; i > 0; i--) {
        obj->trail_x[i] = obj->trail_x[i - 1];
        obj->trail_y[i] = obj->trail_y[i - 1];
    }
    obj->trail_x[0] = obj->x_mm;
    obj->trail_y[0] = obj->y_mm;
    if (obj->trail_len < RADAR_TRAIL_MAX) obj->trail_len++;
    obj->x_mm = new_x;
    obj->y_mm = new_y;
}

/* ================================================================
   radar_init
================================================================ */
void radar_init(radar_mode_t mode)
{
    s_mode = mode;

    if (mode == RADAR_MODE_UART) {
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
            RADAR_UART_PORT, RADAR_PIN_TX, RADAR_PIN_RX, -1, -1));

        ESP_LOGI(TAG, "Radar Iniciado: UART%d (TX:%d, RX:%d)",
                 RADAR_UART_PORT, RADAR_PIN_TX, RADAR_PIN_RX);
    }
}

/* ================================================================
   radar_flush_rx
   Limpa o buffer HW da UART e o ring buffer SW.
   Chamar após o delay de arranque da fsm_task para descartar
   o backlog acumulado e começar com dados frescos.
================================================================ */
void radar_flush_rx(void)
{
    if (s_mode == RADAR_MODE_UART) {
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;
        memset(&s_last_data, 0, sizeof(s_last_data));
        ESP_LOGI(TAG, "RX UART limpo (backlog descartado)");
    }
}

/* ================================================================
   radar_read_data
================================================================ */
bool radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;

    /* Limpa cache desta leitura — display fica sem veículos se
     * não chegar frame válido neste ciclo.                      */
    memset(&s_last_data, 0, sizeof(s_last_data));
    memset(out_data,     0, sizeof(radar_data_t));

    /* --- Modo simulado --- */
    if (s_mode == RADAR_MODE_SIMULATED) {
        if (sim_input && sim_input->active) {
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

    /* --- Lê bytes da UART e acumula no ring buffer --- */
    uint8_t temp_rx[64];
    int len = uart_read_bytes(
        RADAR_UART_PORT, temp_rx, sizeof(temp_rx), pdMS_TO_TICKS(10));

    if (len > 0) {
        /*
         * FIX — ring buffer overflow:
         * Versão anterior fazia apenas s_ring_len = 0 quando
         * s_ring_len + len >= UART_BUF_SIZE, descartando também
         * os bytes novos de temp_rx sem os copiar. Resultado:
         * nunca havia dados suficientes para montar um frame.
         *
         * Solução: quando o buffer estaria a transbordar,
         * guardamos os últimos (FRAME_LEN-1) bytes existentes
         * (podem ser o início de um frame a chegar) e depois
         * acrescentamos os bytes novos normalmente.
         */
        if (s_ring_len + len >= UART_BUF_SIZE) {
            int keep = FRAME_LEN - 1;   /* 29 bytes */
            if (s_ring_len > keep) {
                memmove(s_ring_buf,
                        &s_ring_buf[s_ring_len - keep],
                        keep);
                s_ring_len = keep;
            }
        }
        memcpy(&s_ring_buf[s_ring_len], temp_rx, len);
        s_ring_len += len;
    }

    /* --- Procura frame válido no ring buffer ---
     * Cabeçalho: AA FF 03 00
     * Rodapé   : 55 CC  (nas posições +28 e +29)            */
    int found_idx = -1;
    for (int i = 0; i <= (s_ring_len - FRAME_LEN); i++) {
        if (s_ring_buf[i]    == 0xAA && s_ring_buf[i+1]  == 0xFF &&
            s_ring_buf[i+2]  == 0x03 && s_ring_buf[i+3]  == 0x00 &&
            s_ring_buf[i+28] == 0x55 && s_ring_buf[i+29] == 0xCC) {
            found_idx = i;
            break;
        }
    }

    if (found_idx >= 0) {
        uint8_t *f = &s_ring_buf[found_idx];
        out_data->count = 0;

        for (int t = 0; t < 3; t++) {
            int base = 4 + (t * 8);

            int16_t  x       = _decode_radar_value(f[base],   f[base+1]);
            int16_t  y       = _decode_radar_value(f[base+2], f[base+3]);
            int16_t  spd_cms = _decode_radar_value(f[base+4], f[base+5]);
            uint16_t dist_mm = (uint16_t)f[base+6] | ((uint16_t)f[base+7] << 8);

            if (dist_mm > 0 && dist_mm < MAX_DIST_MM && y != 0) {
                radar_vehicle_t *tgt = &out_data->targets[out_data->count];
                tgt->x_mm     = x;
                tgt->y_mm     = y;
                tgt->distance = (float)dist_mm / 1000.0f;
                /*
                 * FIX — unidade de velocidade:
                 * HLK-LD2450 reporta em cm/s (com sinal).
                 * Convertemos para km/h e guardamos como positivo.
                 * 1 cm/s = 0.036 km/h
                 */
                tgt->speed    = (float)(spd_cms < 0 ? -spd_cms : spd_cms)
                                * 0.036f;
                tgt->detected = true;
                out_data->count++;
            }
        }

        /* Consome o frame do ring buffer */
        int consumed = found_idx + FRAME_LEN;
        memmove(s_ring_buf, &s_ring_buf[consumed], s_ring_len - consumed);
        s_ring_len -= consumed;

        /* Actualiza cache e estado de saúde */
        s_last_data    = *out_data;
        s_last_read_ok = true;
        s_no_frame_cnt = 0;
        return true;
    }

    /* Sem frame válido neste ciclo */
    s_no_frame_cnt++;
    if (s_no_frame_cnt > NO_FRAME_LIMIT) {
        s_last_read_ok = false;
    }
    return false;
}

/* ================================================================
   radar_manager_get_objects
   FIX: usa s_last_data (cache do último frame lido por
   radar_read_data) em vez de chamar radar_read_data() de novo.
   Chamar radar_read_data() aqui consumia o mesmo ring buffer
   que state_machine_update() já lera, resultando sempre em
   count=0 no canvas do display.
================================================================ */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max)
{
    static radar_obj_t internal_objs[MAX_RADAR_TARGETS] = {0};
    uint8_t count = 0;

    for (int i = 0;
         i < s_last_data.count && i < max && i < MAX_RADAR_TARGETS;
         i++)
    {
        _update_trail(&internal_objs[i],
                      s_last_data.targets[i].x_mm,
                      s_last_data.targets[i].y_mm);
        objs[i] = internal_objs[i];
        count++;
    }
    return count;
}

/* ================================================================
   radar_auto_detect_baud
   Tenta cada baud rate candidato durante 300ms e verifica se
   chegam frames com header AA FF 03 00 + footer 55 CC.
   Se encontrar, reconfigura a UART para esse baud rate.
   Retorna o baud rate detectado, ou 0 se nenhum funcionar.
================================================================ */
int radar_auto_detect_baud(void)
{
    if (s_mode != RADAR_MODE_UART) return 0;

    static const int candidatos[] = { 256000, 115200, 57600 };
    static const int n_cand = sizeof(candidatos) / sizeof(candidatos[0]);

    for (int c = 0; c < n_cand; c++) {
        int baud = candidatos[c];
        ESP_LOGI(TAG, "Auto-detect baud: a tentar %d...", baud);

        uart_set_baudrate(RADAR_UART_PORT, baud);
        uart_flush_input(RADAR_UART_PORT);
        s_ring_len = 0;

        /* Aguarda ~300ms = ~3 frames a 10 Hz */
        uint8_t tmp[128];
        uint8_t acum[256];
        int acum_len = 0;
        bool encontrado = false;

        for (int t = 0; t < 6 && !encontrado; t++) {
            vTaskDelay(pdMS_TO_TICKS(50));
            int n = uart_read_bytes(RADAR_UART_PORT, tmp, sizeof(tmp),
                                    pdMS_TO_TICKS(10));
            if (n > 0 && acum_len + n < (int)sizeof(acum)) {
                memcpy(&acum[acum_len], tmp, n);
                acum_len += n;
            }
            /* Procura frame válido no acumulado */
            for (int i = 0; i <= acum_len - FRAME_LEN; i++) {
                if (acum[i]    == 0xAA && acum[i+1]  == 0xFF &&
                    acum[i+2]  == 0x03 && acum[i+3]  == 0x00 &&
                    acum[i+28] == 0x55 && acum[i+29] == 0xCC) {
                    encontrado = true;
                    break;
                }
            }
        }

        if (encontrado) {
            ESP_LOGI(TAG, "Baud rate detectado: %d", baud);
            /* UART já está configurada para este baud — limpa e usa */
            uart_flush_input(RADAR_UART_PORT);
            s_ring_len = 0;
            return baud;
        }
    }

    /* Nenhum baud funcionou — volta a 115200 como fallback seguro */
    ESP_LOGW(TAG, "Auto-detect falhou — a usar %d (fallback)", RADAR_BAUD_RATE);
    uart_set_baudrate(RADAR_UART_PORT, RADAR_BAUD_RATE);
    uart_flush_input(RADAR_UART_PORT);
    s_ring_len = 0;
    return 0;
}

/* ================================================================
   radar_get_status_str
   "REAL" → UART a receber frames
   "SIM"  → modo simulado
   "FAIL" → UART sem frames válidos
================================================================ */
const char *radar_get_status_str(void)
{
    if (s_mode == RADAR_MODE_SIMULATED) return "SIM";
    return s_last_read_ok ? "REAL" : "FAIL";
}

/* ================================================================
   Getters
================================================================ */
bool radar_is_connected(void)
{
    return s_last_read_ok;
}

bool radar_vehicle_in_range(const radar_data_t *data)
{
    return (data && data->count > 0);
}

float radar_get_closest_speed(const radar_data_t *data)
{
    if (!data || data->count == 0) return 0.0f;
    float min_dist = 999.0f;
    float speed    = 0.0f;
    for (int i = 0; i < data->count; i++) {
        if (data->targets[i].distance < min_dist) {
            min_dist = data->targets[i].distance;
            speed    = data->targets[i].speed; /* já em km/h e positivo */
        }
    }
    return speed;
}

radar_mode_t radar_get_mode(void)
{
    return s_mode;
}
