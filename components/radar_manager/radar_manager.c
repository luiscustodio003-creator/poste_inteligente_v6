/* ============================================================
   RADAR MANAGER IMPLEMENTATION - HLK-LD2450
   ------------------------------------------------------------
   @file radar_manager.c
   @brief Leitura do radar HLK-LD2450 via UART

   Projeto: Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Correções aplicadas:
   - USE_RADAR movido para system_config.h (era hardcoded aqui)
   - Conversão explícita de uint8_t para float em tgt->speed
   - Remoção do bloco de comentário duplicado no topo
   ============================================================ */

#include "radar_manager.h"
#include "hw_config.h"
#include "system_config.h"   /* USE_RADAR definido aqui */
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RADAR_MGR";

static radar_mode_t g_radar_mode = RADAR_MODE_DEFAULT;

#define RADAR_BUF_SIZE        256
#define RADAR_UART_TIMEOUT_MS 50

/* ============================================================
   Inicialização
   ============================================================ */
void radar_init(radar_mode_t mode)
{
    g_radar_mode = mode;

#if USE_RADAR
    if (mode == RADAR_MODE_UART || mode == RADAR_MODE_DEFAULT) {
        uart_config_t uart_config = {
            .baud_rate  = RADAR_BAUD_RATE,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
        };

        ESP_ERROR_CHECK(uart_driver_install(
            RADAR_UART_PORT, RADAR_BUF_SIZE * 2, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(
            RADAR_UART_PORT,
            RADAR_PIN_TX, RADAR_PIN_RX,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        ESP_LOGI(TAG, "Radar UART inicializado (RX=%d, TX=%d, %d baud)",
                 RADAR_PIN_RX, RADAR_PIN_TX, RADAR_BAUD_RATE);
    }
#else
    if (mode == RADAR_MODE_SIMULATED) {
        ESP_LOGI(TAG, "Radar em modo SIMULADO (USE_RADAR=0)");
    } else {
        ESP_LOGW(TAG, "USE_RADAR=0: modo UART ignorado, usando SIMULADO");
        g_radar_mode = RADAR_MODE_SIMULATED;
    }
#endif
}

/* ============================================================
   Leitura de dados
   ============================================================ */
bool radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input)
{
    if (!out_data) return false;
    memset(out_data, 0, sizeof(radar_data_t));

#if USE_RADAR
    if (g_radar_mode == RADAR_MODE_UART ||
        g_radar_mode == RADAR_MODE_DEFAULT)
    {
        uint8_t buf[RADAR_BUF_SIZE];
        int len = uart_read_bytes(
            RADAR_UART_PORT, buf, RADAR_BUF_SIZE,
            pdMS_TO_TICKS(RADAR_UART_TIMEOUT_MS));

        if (len > 0) {
            int index = 0;
            while ((index + 5) < len &&
                   out_data->count < MAX_RADAR_TARGETS)
            {
                if (buf[index] == 0xAA) {
                    radar_vehicle_t *tgt = &out_data->targets[out_data->count];
                    tgt->detected = true;
                    tgt->distance = buf[index + 2] / 10.0f;
                    /* FIX: conversão explícita uint8_t -> float para speed */
                    tgt->speed    = (float)buf[index + 3];
                    out_data->count++;
                    index += 6;
                } else {
                    index++;
                }
            }
        }
    }
#endif /* USE_RADAR */

    /* Modo simulado: copia dados injetados pelo chamador */
    if (g_radar_mode == RADAR_MODE_SIMULATED && sim_input != NULL) {
        memcpy(out_data, sim_input, sizeof(radar_data_t));
    }

    return (out_data->count > 0);
}

/* ============================================================
   Modo atual
   ============================================================ */
radar_mode_t radar_get_mode(void)
{
    return g_radar_mode;
}
