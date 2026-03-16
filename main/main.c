/* ============================================================
   MAIN — PONTO DE ENTRADA
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Ponto de entrada do sistema. Inicializa todos os modulos
   pela ordem correcta e cria as tasks FreeRTOS.

   Fluxo de arranque:
   ------------------
   1. nvs_flash_init()         NVS unica
   2. display_manager_start()  Display imediato (pri=5)
   3. post_config_init()       Nome/ID da NVS
   4. dali_init()              PWM luminaria
   5. radar_init()             UART ou simulado
   6. wifi_start_auto()        STA com fallback AP
   7. comm_init()              UDP socket
   8. state_machine_init()     FSM em IDLE 10%
   9. Filas + Tasks

   Tasks e prioridades:
   --------------------
   lvgl_task       pri=5  10ms   Rendering display
   fsm_task        pri=4 100ms   FSM + logica T/Tc
   udp_rx_task     pri=4  20ms   Recepcao UDP (unica)
   radar_task      pri=3 100ms   Leitura HLK-LD2450
   discover_task   pri=3   5s    DISCOVER broadcast
   wifi_recon      pri=2  30s    Reconexao background
   update_task     pri=2 500ms   Display completo

   Sincronizacao:
   --------------
   g_radar_event : portMUX spinlock (radar->fsm, radar->update)
   q_spd         : Queue SPD (udp_rx->fsm)
   q_tc          : Queue TC_INC (udp_rx->fsm)
   q_master      : Queue MASTER_CLAIM (udp_rx->fsm)
============================================================ */

#include "display_manager.h"
#include "wifi_manager.h"
#include "comm_manager.h"
#include "post_config.h"
#include "dali_manager.h"
#include "radar_manager.h"
#include "state_machine.h"
#include "udp_manager.h"
#include "system_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MAIN";

/* ===========================================================
   VARIAVEIS PARTILHADAS ENTRE TASKS
   =========================================================== */

/* Leitura do radar -- spinlock para dual-core */
static radar_vehicle_t g_radar_event = {0};
static portMUX_TYPE    s_radar_mux   = portMUX_INITIALIZER_UNLOCKED;

/* Filas entre udp_rx_task e fsm_task */
static QueueHandle_t q_spd    = NULL;
static QueueHandle_t q_tc     = NULL;
static QueueHandle_t q_master = NULL;

typedef struct { float speed; uint32_t eta_ms; int src_id; } msg_spd_t;
typedef struct { float speed; int src_id; bool is_prev_passed; } msg_tc_t;
typedef struct { int src_id; int src_pos; } msg_master_t;

/* ===========================================================
   FSM TASK (100ms)
   =========================================================== */

static void fsm_task(void *pv)
{
    msg_spd_t    ms;
    msg_tc_t     mt;
    msg_master_t mm;

    while (1)
    {
        /* Le evento do radar com proteccao de concorrencia */
        radar_vehicle_t ev;
        portENTER_CRITICAL(&s_radar_mux);
        ev = g_radar_event;
        portEXIT_CRITICAL(&s_radar_mux);

        /* Radar local deteta carro dentro do alcance */
        if (ev.detected && ev.distance <= (float)RADAR_DETECT_M)
            sm_on_radar_detect(ev.speed);

        /* TC_INC: carro a caminho ou notificacao de passagem */
        while (xQueueReceive(q_tc, &mt, 0) == pdTRUE)
        {
            if (mt.is_prev_passed)
                sm_on_prev_passed();
            else
                sm_on_tc_inc(mt.speed);
        }

        /* SPD: velocidade recebida de vizinho */
        while (xQueueReceive(q_spd, &ms, 0) == pdTRUE)
            sm_on_spd_received(ms.speed, ms.eta_ms);

        /* MASTER_CLAIM: vizinho reclama lideranca */
        while (xQueueReceive(q_master, &mm, 0) == pdTRUE)
        {
            if (mm.src_pos < POST_POSITION)
            {
                ESP_LOGI(TAG, "MASTER_CLAIM pos=%d -- a ceder", mm.src_pos);
                comm_send_master_claim(); /* Propaga para o proximo */
            }
        }

        /* Ciclo principal FSM */
        state_machine_update(comm_status_ok(), comm_is_master());

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ===========================================================
   RADAR TASK (100ms)
   =========================================================== */

static void radar_task(void *pv)
{
#if !USE_RADAR
    static int sim_cycle = 0;
#endif

    while (1)
    {
        radar_vehicle_t new_ev = {0};

#if USE_RADAR
        radar_data_t data = {0};
        if (radar_read_data(&data, NULL) && radar_vehicle_in_range(&data))
        {
            new_ev.detected  = true;
            new_ev.distance  = data.targets[0].distance;
            new_ev.speed     = radar_get_closest_speed(&data);
        }
#else
        /* Simulacao: 3s com veiculo, 7s sem */
        sim_cycle++;
        if (sim_cycle < 30)
        {
            new_ev.detected = true;
            new_ev.distance = (float)RADAR_DETECT_M - 1.0f;
            new_ev.speed    = 45.0f + (sim_cycle % 20);
        }
        if (sim_cycle >= 100) sim_cycle = 0;
#endif

        /* Escreve com spinlock */
        portENTER_CRITICAL(&s_radar_mux);
        g_radar_event = new_ev;
        portEXIT_CRITICAL(&s_radar_mux);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ===========================================================
   UDP RX TASK (20ms) -- UNICA task a ler o socket
   =========================================================== */

static void udp_rx_task(void *pv)
{
    while (1)
    {
        spd_msg_t         spd_out    = {0};
        neighbor_status_t status_out = NEIGHBOR_OK;
        int               src_id    = 0;
        int               src_pos   = 0;

        udp_msg_type_t tipo;
        while ((tipo = udp_manager_process(&spd_out, &status_out,
                                            &src_id, &src_pos)) != UDP_MSG_NONE)
        {
            switch (tipo)
            {
                case UDP_MSG_SPD:
                {
                    msg_spd_t m = { spd_out.speed, spd_out.eta_ms, src_id };
                    xQueueSend(q_spd, &m, 0);
                    break;
                }
                case UDP_MSG_TC_INC:
                {
                    /* Velocidade negativa = notificacao que carro passou */
                    msg_tc_t m = {
                        .speed          = fabsf(spd_out.speed),
                        .src_id         = src_id,
                        .is_prev_passed = (spd_out.speed < 0.0f)
                    };
                    xQueueSend(q_tc, &m, 0);
                    break;
                }
                case UDP_MSG_MASTER_CLAIM:
                {
                    msg_master_t m = { src_id, src_pos };
                    xQueueSend(q_master, &m, 0);
                    break;
                }
                default:
                    break;
            }
        }

        udp_manager_check_timeouts();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ===========================================================
   DISCOVER TASK (DISCOVER_INTERVAL_MS)
   =========================================================== */

static void discover_task(void *pv)
{
    while (1)
    {
        if (comm_status_ok()) comm_discover();
        vTaskDelay(pdMS_TO_TICKS(DISCOVER_INTERVAL_MS));
    }
}

/* ===========================================================
   WIFI RECONNECT TASK (WIFI_RECONNECT_MS)
   =========================================================== */

static void wifi_reconnect_task(void *pv)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_MS));
        if (!wifi_is_connected())
        {
            ESP_LOGI(TAG, "Reconexao Wi-Fi background");
            wifi_try_reconnect();
        }
    }
}

/* ===========================================================
   UPDATE TASK (DISPLAY_UPDATE_MS)
   =========================================================== */

static void update_task(void *pv)
{
    while (1)
    {
        /* Wi-Fi */
        display_manager_set_ap_mode(wifi_is_ap_active(), wifi_get_ip());

        /* Nome + estado linha */
        display_manager_update_info(post_get_name(),
                                     state_machine_get_state_name());

        /* Vizinhos ESQ e DIR */
        neighbor_t *left  = comm_get_neighbor_left();
        neighbor_t *right = comm_get_neighbor_right();
        char esq[16]="---", dir[16]="---";

        if (left)
            snprintf(esq, sizeof(esq), "P%02d %s", left->id,
                     left->status==NEIGHBOR_OK?"OK":
                     left->status==NEIGHBOR_MASTER?"MST":
                     left->status==NEIGHBOR_SAFE_MODE?"SF":"OFF");
        if (right)
            snprintf(dir, sizeof(dir), "P%02d %s", right->id,
                     right->status==NEIGHBOR_OK?"OK":
                     right->status==NEIGHBOR_MASTER?"MST":
                     right->status==NEIGHBOR_SAFE_MODE?"SF":"OFF");

        display_manager_update_neighbors_lr(esq, dir);

        /* Contadores T, Tc e velocidade */
        radar_vehicle_t ev;
        portENTER_CRITICAL(&s_radar_mux);
        ev = g_radar_event;
        portEXIT_CRITICAL(&s_radar_mux);

        display_manager_update_traffic(
            state_machine_get_T(),
            state_machine_get_Tc(),
            ev.detected ? (int)ev.speed : 0);

        /* Tabela vizinhos */
        neighbor_t list[MAX_NEIGHBORS];
        size_t n = comm_get_neighbors(list, MAX_NEIGHBORS);
        display_manager_update_neighbors(list, n);

        /* Brilho + estado FSM */
        display_manager_update_brightness(dali_get_brightness(),
                                           state_machine_get_state_name());

        ESP_LOGI(TAG, "IP=%s T=%d Tc=%d VEL=%.1f LUZ=%d%% [%s]",
                 wifi_get_ip(),
                 state_machine_get_T(), state_machine_get_Tc(),
                 ev.speed, dali_get_brightness(),
                 state_machine_get_state_name());

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

/* ===========================================================
   APP_MAIN
   =========================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Poste Inteligente a arrancar ===");

    /* 1. NVS -- antes de qualquer modulo */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS corrompida -- a reinicializar");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Display */
    display_manager_start();

    /* 3. Configuracao do poste */
    post_config_init();
    display_manager_update_info(post_get_name(), "INIT");

    /* 4. Luminaria */
    dali_init();

    /* 5. Radar */
    radar_init(RADAR_MODE_DEFAULT);

    /* 6. Wi-Fi -- unica chamada */
    wifi_start_auto();

    /* 7. UDP -- apos Wi-Fi */
    if (!comm_init())
        ESP_LOGE(TAG, "Falha UDP -- sem comunicacao");

    /* 8. FSM */
    state_machine_init();

    /* 9. Filas */
    q_spd    = xQueueCreate(8, sizeof(msg_spd_t));
    q_tc     = xQueueCreate(8, sizeof(msg_tc_t));
    q_master = xQueueCreate(4, sizeof(msg_master_t));
    if (!q_spd || !q_tc || !q_master)
        ESP_LOGE(TAG, "Falha criar filas");

    /* 10. Tasks */
    xTaskCreate(fsm_task,            "fsm_task",      4096, NULL, 4, NULL);
    xTaskCreate(radar_task,          "radar_task",    3072, NULL, 3, NULL);
    xTaskCreate(udp_rx_task,         "udp_rx_task",   4096, NULL, 4, NULL);
    xTaskCreate(discover_task,       "discover_task", 3072, NULL, 3, NULL);
    xTaskCreate(wifi_reconnect_task, "wifi_recon",    2048, NULL, 2, NULL);
    xTaskCreate(update_task,         "update_task",   4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "Pronto | %s | ID=%d | pos=%d | USE_RADAR=%d",
             post_get_name(), post_get_id(), POST_POSITION, USE_RADAR);
}