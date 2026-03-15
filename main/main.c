/* ============================================================
   MAIN APPLICATION
   ------------------------------------------------------------
   @file      main.c
   @brief     Ponto de entrada do sistema Poste Inteligente
   @version   4.1
   @date      2026-03-15

   Alteracoes (v4.0 -> v4.1):
   --------------------------
   1. update_task actualiza brilho e estado FSM no display via
      display_manager_update_brightness(). Chama
      dali_get_brightness() para o valor real do PWM e
      state_machine_get_state_name() para o nome do estado.
      O display mostra agora: "LUZ:  50%  [SAFE MODE]" quando
      o radar e o wifi falham, ou "LUZ: 100%  [LIGHT ON]"
      quando um veiculo esta a passar.

   Fluxo de arranque:
   ------------------
   1. nvs_flash_init()       -- NVS unica para todo o sistema
   2. display_manager_start()-- display visivel imediatamente
   3. post_config_init()     -- carrega nome/ID da NVS
   4. dali_init()            -- PWM LEDC
   5. radar_init()           -- UART2 ou simulado
   6. wifi_start_auto()      -- STA ou AP
   7. comm_init()            -- UDP apos Wi-Fi activo
   8. state_machine_init()   -- FSM em IDLE
   9. xQueueCreate(spd_queue)
   10. xTaskCreate x5

   Dependencias:
   -------------
   - display_manager, wifi_manager, comm_manager
   - post_config, dali_manager, radar_manager
   - state_machine, system_config
   - nvs_flash, freertos, esp_log
============================================================ */

#include "display_manager.h"
#include "wifi_manager.h"
#include "comm_manager.h"
#include "post_config.h"
#include "dali_manager.h"
#include "radar_manager.h"
#include "state_machine.h"
#include "system_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

/* -----------------------------------------------------------
   g_radar_event -- partilhado entre radar_task e fsm_task
   Protegido por spinlock para seguranca em dual-core ESP32
   ----------------------------------------------------------- */
static radar_vehicle_t g_radar_event = {0};
static portMUX_TYPE    s_radar_mux   = portMUX_INITIALIZER_UNLOCKED;

/* -----------------------------------------------------------
   spd_queue -- fila de velocidades SPD recebidas via UDP
   udp_rx_task escreve, fsm_task le
   ----------------------------------------------------------- */
static QueueHandle_t spd_queue = NULL;

typedef struct {
    float speed;
    int   post_id;
    int   post_pos;
} spd_msg_t;

/* -----------------------------------------------------------
   fsm_task -- Maquina de estados (100 ms)
   Le g_radar_event com spinlock. Consome spd_queue.
   ----------------------------------------------------------- */
static void fsm_task(void *pv)
{
    spd_msg_t msg;

    while (1)
    {
        /* Le radar com proteccao de concorrencia */
        radar_vehicle_t event;
        portENTER_CRITICAL(&s_radar_mux);
        event = g_radar_event;
        portEXIT_CRITICAL(&s_radar_mux);

        /* Consome mensagens SPD da fila */
        while (xQueueReceive(spd_queue, &msg, 0) == pdTRUE)
        {
            ESP_LOGI(TAG, "SPD vizinho: %.1f km/h | ID=%d | pos=%d",
                     msg.speed, msg.post_id, msg.post_pos);
        }

        bool comm_ok = comm_status_ok();
        state_machine_update(event, comm_ok);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* -----------------------------------------------------------
   radar_task -- Leitura do radar (100 ms)
   Escreve em g_radar_event com spinlock.
   ----------------------------------------------------------- */
static void radar_task(void *pv)
{
    radar_data_t data = {0};

#if !USE_RADAR
    static int sim_cycle = 0;
#endif

    while (1)
    {
        radar_vehicle_t new_event = {0};

#if USE_RADAR
        if (radar_read_data(&data, NULL) && data.count > 0)
            new_event = data.targets[0];
#else
        /* Simula: deteccao durante 3s, pausa 7s */
        sim_cycle++;
        if (sim_cycle < 30)
        {
            new_event.detected = true;
            new_event.speed    = 45.0f + (sim_cycle % 10);
            new_event.distance = 10.0f;
        }
        if (sim_cycle >= 100) sim_cycle = 0;
#endif

        portENTER_CRITICAL(&s_radar_mux);
        g_radar_event = new_event;
        portEXIT_CRITICAL(&s_radar_mux);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* -----------------------------------------------------------
   udp_rx_task -- UNICA task a ler o socket UDP (20 ms)
   Drena o socket e envia SPD para fsm_task via spd_queue.
   ----------------------------------------------------------- */
static void udp_rx_task(void *pv)
{
    while (1)
    {
        spd_msg_t msg = {0};

        while (comm_receive_vehicle(&msg.speed,
                                    &msg.post_id,
                                    &msg.post_pos))
        {
            xQueueSend(spd_queue, &msg, 0);
        }

        udp_manager_check_timeouts();

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* -----------------------------------------------------------
   discover_task -- DISCOVER periodico (DISCOVER_INTERVAL_MS)
   ----------------------------------------------------------- */
static void discover_task(void *pv)
{
    while (1)
    {
        if (comm_status_ok())
            comm_discover();

        vTaskDelay(pdMS_TO_TICKS(DISCOVER_INTERVAL_MS));
    }
}

/* -----------------------------------------------------------
   update_task -- Actualizacao do display (DISPLAY_UPDATE_MS)

   Actualiza todos os elementos do display incluindo:
   - Icone Wi-Fi e modo AP/STA
   - Tabela de vizinhos
   - Labels de poste, vizinhos, velocidade
   - NOVO: brilho actual da luminaria e estado FSM

   O brilho e lido directamente do dali_manager (valor real
   do PWM LEDC) e o estado FSM e lido da state_machine.
   Exemplos de saida no display:
     "LUZ: 100%  [LIGHT ON]"  -- veiculo presente
     "LUZ:  50%  [SAFE MODE]" -- radar e wifi falharam
     "LUZ:  55%  [DETECTION]" -- a confirmar veiculo
     "LUZ:  10%  [IDLE]"      -- em repouso
     "LUZ:  10%  [TIMEOUT]"   -- a aguardar ciclo seguinte
   ----------------------------------------------------------- */
static void update_task(void *pv)
{
    while (1)
    {
        bool        connected = wifi_is_connected();
        bool        ap_active = wifi_is_ap_active();
        const char *ip        = wifi_get_ip();

        /* Icone Wi-Fi */
        display_manager_set_wifi(connected);

        /* Label IP com prefixo correcto (AP: ou IP:) */
        display_manager_set_ap_mode(ap_active, ip);

        /* Tabela de vizinhos */
        neighbor_t neighbors[MAX_NEIGHBORS];
        size_t     n = comm_get_neighbors(neighbors, MAX_NEIGHBORS);
        display_manager_update_neighbors(neighbors, n);

        /* Le radar com spinlock para velocidade no display */
        radar_vehicle_t event;
        portENTER_CRITICAL(&s_radar_mux);
        event = g_radar_event;
        portEXIT_CRITICAL(&s_radar_mux);

        /* Labels principais -- ip vazio: gerido por set_ap_mode */
        display_manager_update_info(
            post_get_name(),
            "",
            (int)n,
            (int)event.speed
        );

        /* Brilho actual + estado FSM no display
           dali_get_brightness() retorna o valor real do PWM.
           state_machine_get_state_name() retorna o estado textual.
           Juntos formam: "LUZ:  50%  [SAFE MODE]"              */
        uint8_t     brightness = dali_get_brightness();
        const char *fsm_state  = state_machine_get_state_name();
        display_manager_update_brightness(brightness, fsm_state);

        ESP_LOGI(TAG,
                 "Display | STA=%d AP=%d IP=%s VIZ=%d VEL=%.1f LUZ=%d%% [%s]",
                 connected, ap_active, ip,
                 (int)n, event.speed,
                 brightness, fsm_state);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

/* -----------------------------------------------------------
   app_main -- Ponto de entrada do ESP-IDF
   ----------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Poste Inteligente a arrancar ===");

    /* 1. NVS -- unica inicializacao */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS corrompida -- a apagar e reinicializar");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Display */
    display_manager_start();

    /* 3. Configuracao do poste */
    post_config_init();
    display_manager_update_info(post_get_name(), "", 0, 0);

    /* 4. Luminaria PWM */
    dali_init();

    /* 5. Radar */
    radar_init(RADAR_MODE_DEFAULT);

    /* 6. Wi-Fi -- unica chamada a wifi_start_auto() */
    wifi_start_auto();

    /* 7. UDP -- so apos Wi-Fi activo */
    if (!comm_init())
        ESP_LOGE(TAG, "Falha iniciar comunicacao -- continuando sem UDP");

    /* 8. FSM */
    state_machine_init();

    /* 9. Fila SPD */
    spd_queue = xQueueCreate(4, sizeof(spd_msg_t));
    if (!spd_queue)
        ESP_LOGE(TAG, "Falha criar spd_queue");

    /* 10. Tasks FreeRTOS
       Prioridades: lvgl(5) > fsm/udp_rx(4) > radar/discover(3) > update(2) */
    xTaskCreate(fsm_task,      "fsm_task",      4096, NULL, 4, NULL);
    xTaskCreate(radar_task,    "radar_task",    3072, NULL, 3, NULL);
    xTaskCreate(udp_rx_task,   "udp_rx_task",   4096, NULL, 4, NULL);
    xTaskCreate(discover_task, "discover_task", 3072, NULL, 3, NULL);
    xTaskCreate(update_task,   "update_task",   4096, NULL, 2, NULL);

    ESP_LOGI(TAG,
             "Sistema iniciado | POSTE=%s | ID=%d | USE_RADAR=%d",
             post_get_name(), post_get_id(), USE_RADAR);
}