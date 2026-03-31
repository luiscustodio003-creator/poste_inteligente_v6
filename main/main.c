/* ============================================================
   MAIN — PONTO DE ENTRADA v3.6
   ------------------------------------------------------------
   @file      main.c
   @brief     Inicialização e ciclo principal do Poste Inteligente
   @version   3.6
   @date      2026-03-30

   NOTAS DE VERSÃO:
   - Independência da FSM: Radar e Lógica arrancam antes do Wi-Fi.
   - Prevenção de Buffer Overflow na UART2.
   - Compatibilidade com display_manager v5.1 e radar_manager v2.6.
============================================================ */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_event.h"
#include <string.h>
#include <stdbool.h>

/* Inclusões do Projecto */
#include "system_config.h"
#include "hw_config.h"
#include "post_config.h"
#include "wifi_manager.h"
#include "display_manager.h" 
#include "radar_manager.h"   
#include "comm_manager.h"
#include "udp_manager.h"
#include "dali_manager.h"
#include "state_machine.h"

static const char *TAG = "MAIN";

/* Flags de teste manual via consola */
volatile bool   g_test_car       = false;
volatile float g_test_car_speed = 50.0f;

/* ============================================================
   AUXILIAR: Inicialização NVS
============================================================ */
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/* ============================================================
   AUXILIAR: Atualização do Canvas Radar
============================================================ */
static void _atualiza_radar_display(void)
{
#if USE_RADAR == 0
    float x_mm = 0.0f, y_mm = 0.0f;
    if (sim_get_objeto(&x_mm, &y_mm)) {
        radar_obj_t obj = {0};
        obj.x_mm = (int)x_mm;
        obj.y_mm = (int)y_mm;
        display_manager_set_radar(&obj, 1);
    } else {
        display_manager_set_radar(NULL, 0);
    }
#else
    radar_obj_t objs[RADAR_MAX_OBJ];
    uint8_t count = radar_manager_get_objects(objs, RADAR_MAX_OBJ);
    // Envia os objectos detectados (com rasto) para o display
    display_manager_set_radar(count > 0 ? objs : NULL, count);
#endif
}

/* ============================================================
   fsm_task — Ciclo de Controlo (Período 100ms)
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "FSM task iniciada | período=100ms");
    
    /* Aguarda estabilização do sensor radar (o HLK-LD2450
     * precisa de ~1s após alimentação para começar a enviar).
     * Depois limpa o backlog UART acumulado durante o delay
     * para a primeira leitura começar com dados frescos.    */
    vTaskDelay(pdMS_TO_TICKS(1500));
    radar_flush_rx();

    while (1)
    {
        // 1. Injeção de teste manual (via Debugger/Consola)
        if (g_test_car) {
            g_test_car = false;
            sm_inject_test_car(g_test_car_speed);
        }

        // 2. Atualiza Lógica da Máquina de Estados
        // Passa o estado da comunicação UDP para decidir se opera em MASTER ou SLAVE
        state_machine_update(comm_status_ok(), comm_is_master());

        // 3. Sincroniza Interface (Display Manager)
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(state_machine_get_T(), state_machine_get_Tc());
        display_manager_set_speed((int)state_machine_get_last_speed());
        
        // Informa o display: modo do radar (REAL/SIM/FAIL) e brilho DALI actual
        display_manager_set_hardware(radar_get_status_str(),
                                     radar_is_connected(),
                                     (uint8_t)dali_get_brightness());

        // 4. Atualiza pontos no Canvas do Radar
        _atualiza_radar_display();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   main_task — Supervisão, LVGL e Rede
============================================================ */
static void main_task(void *arg) 
{
    ESP_LOGI(TAG, "[1/6] Inicializando NVS...");
    init_nvs();

    ESP_LOGI(TAG, "[2/6] Configurando Rede...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "[3/6] Lendo Configurações...");
    post_config_init();
    
    ESP_LOGI(TAG, "[4/6] Inicializando Hardware (DALI e RADAR)...");
    dali_init();
    radar_init(USE_RADAR ? RADAR_MODE_UART : RADAR_MODE_SIMULATED);
#if USE_RADAR
    int baud_det = radar_auto_detect_baud(); /* detecta 256000 / 115200 / 57600 */
    if (baud_det > 0)
        ESP_LOGI(TAG, "Radar baud detectado: %d", baud_det);
    else
        ESP_LOGW(TAG, "Radar baud auto-detect falhou — a usar %d", RADAR_BAUD_RATE);
#endif
    
    ESP_LOGI(TAG, "[5/6] Inicializando Display e FSM...");
    display_manager_init();
    
    /* CRÍTICO: Arrancamos a FSM antes do WiFi para o radar começar a ser lido logo */
    state_machine_init();
    xTaskCreate(fsm_task, "fsm_task", 6144, NULL, 6, NULL);

    ESP_LOGI(TAG, "[6/6] Inicializando WIFI...");
    wifi_manager_init();

    ESP_LOGI(TAG, ">>> SISTEMA OPERACIONAL <<<");

    bool comm_iniciado = false;
    uint32_t last_tick = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        // --- GESTÃO DO DISPLAY (Thread Principal) ---
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        display_manager_tick(now - last_tick);
        display_manager_task(); // Processa eventos LVGL
        last_tick = now;

        // --- GESTÃO DE COMUNICAÇÃO ENTRE POSTES ---
        bool wifi_curr = wifi_manager_is_connected();
        
        // Se ligou ao WiFi mas o serviço UDP ainda não arrancou
        if (!comm_iniciado && wifi_curr) {
            if (comm_init()) {
                comm_iniciado = true;
                ESP_LOGI(TAG, "Protocolo de rede iniciado.");
            }
        }

        // Se a rede estiver ativa, atualiza dados dos vizinhos no display
        if (comm_iniciado) {
            char nL[MAX_IP_LEN], nR[MAX_IP_LEN];
            udp_manager_get_neighbors(nL, nR);
            display_manager_set_neighbors(nL, nR, comm_left_online(), comm_right_online());
            display_manager_set_wifi(true, wifi_manager_get_ip());
        } else {
            display_manager_set_wifi(false, "Desligado");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Frequência para LVGL fluido
    }
}

void app_main(void)
{
    // Criamos a main_task com stack suficiente para o WiFi e Display
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}