/* ============================================================
   RADAR MANAGER — IMPLEMENTAÇÃO v2.0
   ------------------------------------------------------------
   @file      radar_manager.c
   @brief     Driver HLK-LD2450 com parse completo do frame 30 bytes
   @version   2.0
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Protocolo HLK-LD2450 — Frame de 30 bytes:
   -------------------------------------------
   Header : AA FF 03 00                          (4 bytes)
   Alvo 1 : X[2] Y[2] Speed[2] Resolution[2]    (8 bytes)
   Alvo 2 : X[2] Y[2] Speed[2] Resolution[2]    (8 bytes)
   Alvo 3 : X[2] Y[2] Speed[2] Resolution[2]    (8 bytes)
   Tail   : 55 CC                                (2 bytes)
   Total  : 30 bytes

   Codificação de coordenadas:
   ----------------------------
   X (mm, int16 little-endian):
     bit15=0 → X positivo (direita do sensor)
     bit15=1 → X negativo (esquerda): valor = -(raw & 0x7FFF)
   Y (mm, int16 little-endian):
     bit15=0 → Y positivo (frente do sensor, distância)
     bit15=1 → Y negativo (atrás): ignorado (fora do alcance)
   Speed (cm/s, int16 little-endian):
     bit15=0 → aproxima-se (positivo)
     bit15=1 → afasta-se  (negativo): -(raw & 0x7FFF)
   Resolution: sempre 0 (reservado)

   Alvo inactivo: X=0, Y=0, Speed=0, Resolution=0

   Ref: https://github.com/hermannsblum/ld2450
        HLK-LD2450 User Manual v1.02

============================================================ */

#include "radar_manager.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

static const char *TAG = "RADAR_MGR";

static radar_mode_t s_mode = RADAR_MODE_DEFAULT;

/* Tamanho do frame completo HLK-LD2450 */
#define FRAME_LEN       30
#define FRAME_HDR0      0xAA
#define FRAME_HDR1      0xFF
#define FRAME_HDR2      0x03
#define FRAME_HDR3      0x00
#define FRAME_TAIL0     0x55
#define FRAME_TAIL1     0xCC

/* Buffer UART — espaço para 3 frames completos */
#define RADAR_BUF_SIZE        (FRAME_LEN * 3)
#define RADAR_UART_TIMEOUT_MS  20

/* Buffer interno para acumulação de bytes UART */
static uint8_t  s_uart_buf[RADAR_BUF_SIZE * 2];
static int      s_uart_buf_len = 0;

/* ============================================================
   _parse_int16
   ------------------------------------------------------------
   Interpreta 2 bytes little-endian com codificação de sinal
   do HLK-LD2450:
     bit15=0 → positivo: valor = raw
     bit15=1 → negativo: valor = -(raw & 0x7FFF)
============================================================ */
static int16_t _parse_int16(const uint8_t *p)
{
    uint16_t raw = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    if (raw & 0x8000)
        return -(int16_t)(raw & 0x7FFF);
    return (int16_t)raw;
}

/* ============================================================
   _parse_frame
   ------------------------------------------------------------
   Extrai até 3 alvos de um frame de 30 bytes validado.
   Alvos inactivos (X=Y=Speed=0) são ignorados.
   Retorna número de alvos activos encontrados.
============================================================ */
static int _parse_frame(const uint8_t *frame, radar_data_t *out)
{
    int count = 0;

    for (int i = 0; i < MAX_RADAR_TARGETS; i++)
    {
        /* Offset de cada alvo: header=4 bytes, alvo=8 bytes cada */
        const uint8_t *p = frame + 4 + (i * 8);

        int16_t x_mm   = _parse_int16(p + 0);
        int16_t y_mm   = _parse_int16(p + 2);
        int16_t spd_cs = _parse_int16(p + 4); /* cm/s */

        /* Alvo inactivo: coordenadas todas zero */
        if (x_mm == 0 && y_mm == 0 && spd_cs == 0)
            continue;

        /* Y negativo = alvo atrás do sensor — ignorar */
        if (y_mm <= 0)
            continue;

        radar_vehicle_t *t = &out->targets[count];
        t->detected  = true;
        t->distance  = (float)y_mm / 1000.0f;   /* mm → metros  */
        t->speed     = fabsf((float)spd_cs) * 0.036f; /* cm/s → km/h */
        t->x_mm      = (int)x_mm;
        t->y_mm      = (int)y_mm;

        ESP_LOGD(TAG, "Alvo %d: X=%dmm Y=%dmm Spd=%.1fkm/h",
                 count, x_mm, y_mm, t->speed);
        count++;
    }

    out->count = count;
    return count;
}

/* ============================================================
   _procura_frame
   ------------------------------------------------------------
   Procura um frame válido no buffer acumulado.
   Valida header (AA FF 03 00) e tail (55 CC).
   Retorna o índice de início ou -1 se não encontrado.
============================================================ */
static int _procura_frame(const uint8_t *buf, int len)
{
    for (int i = 0; i <= len - FRAME_LEN; i++)
    {
        /* Verifica header */
        if (buf[i]   != FRAME_HDR0 || buf[i+1] != FRAME_HDR1 ||
            buf[i+2] != FRAME_HDR2 || buf[i+3] != FRAME_HDR3)
            continue;

        /* Verifica tail */
        if (buf[i + FRAME_LEN - 2] != FRAME_TAIL0 ||
            buf[i + FRAME_LEN - 1] != FRAME_TAIL1)
            continue;

        return i;
    }
    return -1;
}

/* ============================================================
   INICIALIZAÇÃO
============================================================ */
void radar_init(radar_mode_t mode)
{
    s_mode = mode;
    s_uart_buf_len = 0;

#if USE_RADAR
    if (mode == RADAR_MODE_UART || mode == RADAR_MODE_DEFAULT)
    {
        uart_config_t cfg = {
            .baud_rate  = RADAR_BAUD_RATE,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
        };
        ESP_ERROR_CHECK(uart_driver_install(
            RADAR_UART_PORT, RADAR_BUF_SIZE * 2, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(
            RADAR_UART_PORT,
            RADAR_PIN_TX, RADAR_PIN_RX,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        ESP_LOGI(TAG, "UART v2.0 | RX=%d TX=%d %dbaud | frame=%d bytes",
                 RADAR_PIN_RX, RADAR_PIN_TX, RADAR_BAUD_RATE, FRAME_LEN);
    }
#else
    if (mode != RADAR_MODE_SIMULATED)
    {
        ESP_LOGW(TAG, "USE_RADAR=0 — modo SIMULADO activado");
        s_mode = RADAR_MODE_SIMULATED;
    }
    ESP_LOGI(TAG, "Radar SIMULADO (physics em state_machine.c)");
#endif
}

/* ============================================================
   radar_read_data
   ------------------------------------------------------------
   Lê bytes do UART, acumula no buffer interno e extrai frames.
   Processa APENAS o frame mais recente (descarta os anteriores)
   para ter sempre a posição actual dos alvos.
============================================================ */
bool radar_read_data(radar_data_t *out_data,
                     radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;
    memset(out_data, 0, sizeof(radar_data_t));

#if USE_RADAR
    /* Lê novos bytes do UART */
    int cap = (int)sizeof(s_uart_buf) - s_uart_buf_len;
    if (cap > 0)
    {
        int n = uart_read_bytes(RADAR_UART_PORT,
                                s_uart_buf + s_uart_buf_len,
                                cap,
                                pdMS_TO_TICKS(RADAR_UART_TIMEOUT_MS));
        if (n > 0) s_uart_buf_len += n;
    }

    /* Procura e processa o frame mais recente no buffer */
    int last_frame_idx = -1;
    int search_from    = 0;

    while (1)
    {
        int idx = _procura_frame(s_uart_buf + search_from,
                                 s_uart_buf_len - search_from);
        if (idx < 0) break;
        last_frame_idx = search_from + idx;
        search_from    = last_frame_idx + FRAME_LEN;
    }

    if (last_frame_idx >= 0)
    {
        /* Processa o frame mais recente */
        _parse_frame(s_uart_buf + last_frame_idx, out_data);

        /* Descarta bytes processados — mantém apenas o que sobra */
        int consumed = last_frame_idx + FRAME_LEN;
        s_uart_buf_len -= consumed;
        if (s_uart_buf_len > 0)
            memmove(s_uart_buf, s_uart_buf + consumed, s_uart_buf_len);
        else
            s_uart_buf_len = 0;
    }
    else if (s_uart_buf_len > (int)sizeof(s_uart_buf) - FRAME_LEN)
    {
        /* Buffer cheio sem frame válido — limpa (dados corrompidos) */
        ESP_LOGW(TAG, "Buffer cheio sem frame valido — a limpar");
        s_uart_buf_len = 0;
    }

#else
    /* Modo simulado — dados injectados externamente se necessário */
    if (s_mode == RADAR_MODE_SIMULATED && sim_input != NULL)
        memcpy(out_data, sim_input, sizeof(radar_data_t));
#endif

    return (out_data->count > 0);
}

/* ============================================================
   radar_manager_get_objects
   ------------------------------------------------------------
   API nova para o main.c (USE_RADAR=1).
   Lê frame UART, extrai posições X/Y em mm e preenche
   array de radar_obj_t para o display_manager_set_radar().

   Retorna número de objectos activos (0..MAX_RADAR_TARGETS).
============================================================ */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max)
{
    if (!objs || max == 0) return 0;

    radar_data_t data;
    memset(&data, 0, sizeof(data));

    if (!radar_read_data(&data, NULL) || data.count == 0)
        return 0;

    uint8_t count = 0;
    for (int i = 0; i < data.count && count < max; i++)
    {
        if (!data.targets[i].detected) continue;

        objs[count].x_mm      = data.targets[i].x_mm;
        objs[count].y_mm      = data.targets[i].y_mm;
        objs[count].trail_len = 0; /* rasto gerido pelo display_manager */
        count++;
    }

    return count;
}

/* ============================================================
   UTILITÁRIOS
============================================================ */

radar_mode_t radar_get_mode(void)
{
    return s_mode;
}

bool radar_vehicle_in_range(const radar_data_t *data)
{
    if (!data) return false;
    for (int i = 0; i < data->count; i++)
        if (data->targets[i].detected &&
            data->targets[i].distance <= (float)RADAR_DETECT_M)
            return true;
    return false;
}

float radar_get_closest_speed(const radar_data_t *data)
{
    if (!data) return 0.0f;
    float best_dist  = 1000.0f;
    float best_speed = 0.0f;

    for (int i = 0; i < data->count; i++)
    {
        if (data->targets[i].detected &&
            data->targets[i].distance < best_dist)
        {
            best_dist  = data->targets[i].distance;
            best_speed = data->targets[i].speed;
        }
    }
    return best_speed;
}