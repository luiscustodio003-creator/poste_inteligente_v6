/* ============================================================
   RADAR MANAGER — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Driver para o sensor HLK-LD2450 via UART2.
   O protocolo usa frame com byte de inicio 0xAA:
     byte[0] = 0xAA (frame start)
     byte[2] = distancia / 10.0 (metros)
     byte[3] = velocidade (km/h, conversao explicita uint8_t->float)

   Ref: https://github.com/hermannsblum/ld2450
============================================================ */

#include "radar_manager.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "RADAR_MGR";

static radar_mode_t s_mode = RADAR_MODE_DEFAULT;

#define RADAR_BUF_SIZE        256
#define RADAR_UART_TIMEOUT_MS 50

/* ===========================================================
   INICIALIZACAO
   =========================================================== */

void radar_init(radar_mode_t mode)
{
    s_mode = mode;

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

        ESP_LOGI(TAG, "UART inicializado RX=%d TX=%d %dbaud",
                 RADAR_PIN_RX, RADAR_PIN_TX, RADAR_BAUD_RATE);
    }
#else
    /* Modo simulado -- nenhum hardware necessario */
    if (mode != RADAR_MODE_SIMULATED)
    {
        ESP_LOGW(TAG, "USE_RADAR=0 -- modo SIMULADO activado");
        s_mode = RADAR_MODE_SIMULATED;
    }
    ESP_LOGI(TAG, "Radar em modo SIMULADO");
#endif
}

/* ===========================================================
   LEITURA DE DADOS
   =========================================================== */

bool radar_read_data(radar_data_t *out_data,
                     radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;
    memset(out_data, 0, sizeof(radar_data_t));

#if USE_RADAR
    if (s_mode == RADAR_MODE_UART || s_mode == RADAR_MODE_DEFAULT)
    {
        uint8_t buf[RADAR_BUF_SIZE];
        int len = uart_read_bytes(RADAR_UART_PORT, buf, RADAR_BUF_SIZE,
                                   pdMS_TO_TICKS(RADAR_UART_TIMEOUT_MS));
        if (len > 0)
        {
            int idx = 0;
            while ((idx + 5) < len &&
                   out_data->count < MAX_RADAR_TARGETS)
            {
                if (buf[idx] == 0xAA)
                {
                    radar_vehicle_t *t = &out_data->targets[out_data->count];
                    t->detected  = true;
                    t->distance  = buf[idx + 2] / 10.0f;
                    /* Conversao explicita uint8_t -> float para velocidade */
                    t->speed     = (float)buf[idx + 3];
                    out_data->count++;
                    idx += 6;
                }
                else idx++;
            }
        }
    }
#endif /* USE_RADAR */

    /* Modo simulado: copia dados injectados pelo chamador */
    if (s_mode == RADAR_MODE_SIMULATED && sim_input != NULL)
        memcpy(out_data, sim_input, sizeof(radar_data_t));

    return (out_data->count > 0);
}

/* ===========================================================
   UTILITARIOS
   =========================================================== */

radar_mode_t radar_get_mode(void)
{
    return s_mode;
}

/* Retorna true se algum alvo esta dentro de RADAR_DETECT_M metros */
bool radar_vehicle_in_range(const radar_data_t *data)
{
    if (!data) return false;
    for (int i = 0; i < data->count; i++)
        if (data->targets[i].detected &&
            data->targets[i].distance <= (float)RADAR_DETECT_M)
            return true;
    return false;
}

/* Retorna velocidade do alvo mais proximo em range */
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