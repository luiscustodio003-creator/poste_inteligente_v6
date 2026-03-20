/* ============================================================
   MAIN — PONTO DE ENTRADA DO SISTEMA
   ------------------------------------------------------------
   @file      main.c
   @brief     Inicialização e ciclo principal do Poste Inteligente
   @version   3.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Ordem de inicialização:
   -----------------------
   1. NVS          — obrigatória antes de Wi-Fi e post_config
   2. post_config  — carrega ID e nome do poste da NVS
   3. dali_manager — PWM da luminária (antes de qualquer brilho)
   4. radar_manager— sensor HLK-LD2450 ou modo simulado
   5. display      — ST7789 + LVGL + UI
   6. Wi-Fi        — inicia ligação STA
   7. comm_manager — UDP (após Wi-Fi com IP)
   8. state_machine— FSM (após comm_manager)

   Tasks:
   ------
   main_task  — display + Wi-Fi + comm (500ms)
   fsm_task   — radar + state_machine (100ms)

   Alterações v2.5 → v3.0:
   ------------------------
   1. dali_init() e radar_init() adicionados à sequência
   2. fsm_task criada: lê radar, corre state_machine_update()
   3. Display actualizado com estado FSM, T, Tc, velocidade,
      estado do radar e brilho DALI em tempo real
   4. sm_inject_test_car() disponível via flag g_test_car
      (setar a true no monitor para simular carro)

============================================================ */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdbool.h>

#include "system_config.h"
#include "post_config.h"
#include "wifi_manager.h"
#include "display_manager.h"
#include "comm_manager.h"
#include "udp_manager.h"
#include "dali_manager.h"
#include "radar_manager.h"
#include "state_machine.h"
#include "esp_timer.h"   /* esp_timer_get_time para teste simulado */

static const char *TAG = "MAIN";

/* ============================================================
   FLAG DE TESTE — setar a true para simular carro
   Útil com USE_RADAR=0 sem precisar de hardware.
   Exemplo de uso no monitor série com OpenOCD/GDB:
     set g_test_car = 1
   Ou chamar sm_inject_test_car(50.0) directamente.
============================================================ */
volatile bool  g_test_car       = false;
volatile float g_test_car_speed = 50.0f;  /* km/h por omissão */

/* ============================================================
   init_nvs
============================================================ */
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS corrompida — a limpar...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializada");
}

/* ============================================================
   fsm_task
   ------------------------------------------------------------
   Task dedicada à máquina de estados.
   Corre a 100ms — independente do ciclo de display (500ms).
   Responsabilidades:
     - Verificar flag de teste (USE_RADAR=0)
     - Chamar state_machine_update()
     - Actualizar display com estado FSM actual
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "FSM task iniciada");

    /* Aguarda comm_manager estar pronto */
    vTaskDelay(pdMS_TO_TICKS(2000));

#if USE_RADAR == 0
    /* Modo simulado: injeta carro automaticamente a cada
       TEST_CAR_INTERVAL_MS para validar o ciclo completo.
       Alterna velocidades para simular trafego variado.    */
    #define TEST_CAR_INTERVAL_MS   12000
    static const float test_speeds[] = { 30.0f, 50.0f, 80.0f, 50.0f };
    static int   test_speed_idx = 0;
    uint32_t     last_test_ms   = 0;
    ESP_LOGI(TAG, "MODO SIMULADO: carro a cada %ds",
             TEST_CAR_INTERVAL_MS / 1000);
#endif

    while (1)
    {
        /* --- Teste manual via flag (qualquer modo) --- */
        if (g_test_car)
        {
            g_test_car = false;
            sm_inject_test_car(g_test_car_speed);
        }

#if USE_RADAR == 0
        /* --- Teste automatico periodico (so USE_RADAR=0) --- */
        uint32_t agora = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (last_test_ms == 0 ||
            (agora - last_test_ms) >= TEST_CAR_INTERVAL_MS)
        {
            float vel = test_speeds[test_speed_idx];
            test_speed_idx = (test_speed_idx + 1) %
                (sizeof(test_speeds) / sizeof(test_speeds[0]));
            last_test_ms = agora;
            ESP_LOGI(TAG, "AUTO-TESTE: carro %.0fkm/h", vel);
            sm_inject_test_car(vel);
        }
#endif

        /* --- Ciclo principal da FSM --- */
        bool comm_ok   = comm_status_ok();
        bool is_master = comm_is_master();

        state_machine_update(comm_ok, is_master);

        /* --- Actualiza display com estado da FSM --- */
        display_manager_set_status(
            state_machine_get_state_name());

        display_manager_set_traffic(
            state_machine_get_T(),
            state_machine_get_Tc());

        display_manager_set_speed(
            (int)state_machine_get_last_speed());

        display_manager_set_hardware(
            state_machine_radar_ok(),
            dali_get_brightness());

        display_manager_set_leader(is_master);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   main_task
   ------------------------------------------------------------
   Task principal: display + Wi-Fi + comm.
   Período: DISPLAY_UPDATE_MS (500ms).
============================================================ */
static void main_task(void *arg)
{
    /* ----------------------------------------------------------
       FASE 1 — Inicialização (ordem obrigatória)
    ---------------------------------------------------------- */

    ESP_LOGI(TAG, "[1/6] NVS...");
    init_nvs();

    ESP_LOGI(TAG, "[2/6] Configuração do poste...");
    post_config_init();
    ESP_LOGI(TAG, "ID=%d | POS=%d | NOME=%s",
             post_get_id(), POST_POSITION, post_get_name());

    ESP_LOGI(TAG, "[3/6] DALI (PWM luminária)...");
    dali_init();

    ESP_LOGI(TAG, "[4/6] Radar (%s)...",
             USE_RADAR ? "HLK-LD2450" : "SIMULADO");
    radar_init(USE_RADAR ? RADAR_MODE_UART : RADAR_MODE_SIMULATED);

    ESP_LOGI(TAG, "[5/6] Display ST7789 + LVGL...");
    display_manager_init();
    display_manager_set_wifi(false, NULL);
    display_manager_set_status("IDLE");
    display_manager_set_hardware(true, LIGHT_MIN);
    display_manager_set_traffic(0, 0);
    display_manager_set_speed(0);
    display_manager_set_neighbors(NULL, NULL, false, false);

    ESP_LOGI(TAG, "[6/6] Wi-Fi...");
    wifi_manager_init();

    /* ----------------------------------------------------------
       FASE 2 — Variáveis de controlo
    ---------------------------------------------------------- */
    bool wifi_prev           = false;
    char ip_prev[MAX_IP_LEN] = "---";
    char nebL[MAX_IP_LEN];
    char nebR[MAX_IP_LEN];
    bool comm_iniciado = false;
    bool fsm_iniciada  = false;

    ESP_LOGI(TAG, "Sistema pronto");

    /* ----------------------------------------------------------
       CICLO PRINCIPAL — 500ms
    ---------------------------------------------------------- */
    while (1)
    {
        /* Ciclo LVGL */
        display_manager_task();
        display_manager_tick(DISPLAY_UPDATE_MS);

        /* Estado Wi-Fi */
        bool        wifi_curr = wifi_manager_is_connected();
        const char *ip_curr   = wifi_manager_get_ip();

        if (wifi_curr != wifi_prev ||
            strncmp(ip_curr, ip_prev, MAX_IP_LEN) != 0)
        {
            display_manager_set_wifi(wifi_curr,
                                     wifi_curr ? ip_curr : NULL);
            wifi_prev = wifi_curr;
            strncpy(ip_prev, ip_curr, MAX_IP_LEN - 1);
            ip_prev[MAX_IP_LEN - 1] = '\0';
            ESP_LOGI(TAG, "Wi-Fi %s | IP: %s",
                     wifi_curr ? "ON" : "OFF", ip_curr);
        }

        /* Inicializa comm após Wi-Fi */
        if (!comm_iniciado && wifi_curr)
        {
            if (comm_init())
            {
                comm_iniciado = true;
                ESP_LOGI(TAG, "Comm pronto | MASTER=%s",
                         comm_is_master() ? "SIM" : "NAO");
            }
            else
            {
                ESP_LOGW(TAG, "comm_init() falhou — a tentar...");
            }
        }

        /* Inicializa FSM após comm */
        if (!fsm_iniciada && comm_iniciado)
        {
            state_machine_init();
            xTaskCreate(fsm_task, "fsm_task",
                        4096, NULL, 6, NULL);
            fsm_iniciada = true;
            ESP_LOGI(TAG, "FSM e fsm_task iniciadas");
        }

        /* Actualiza vizinhos no display */
        if (comm_iniciado)
        {
            udp_manager_get_neighbors(nebL, nebR);
            display_manager_set_neighbors(nebL, nebR,
                                          comm_left_online(),
                                          comm_right_online());
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

/* ============================================================
   app_main
============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, " Poste Inteligente v3.0");
    ESP_LOGI(TAG, " ID=%d | POS=%d | NOME=%s",
             POSTE_ID, POST_POSITION, POSTE_NAME);
    ESP_LOGI(TAG, " Radar=%s",
             USE_RADAR ? "HW" : "SIMULADO");
    ESP_LOGI(TAG, "============================================");

    xTaskCreate(main_task, "main_task",
                8192, NULL, 5, NULL);
}