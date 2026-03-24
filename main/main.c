/* ============================================================
   MAIN — PONTO DE ENTRADA DO SISTEMA
   ------------------------------------------------------------
   @file      main.c
   @brief     Inicialização e ciclo principal do Poste Inteligente
   @version   3.1
   @date      2026-03-23

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

   Alterações v3.0 → v3.1:
   ------------------------
   1. _atualiza_radar_display() adicionada — passa objectos ao
      display_manager_set_radar() em modo simulado e real
   2. display_manager_tick() corrigido: usa delta real via
      esp_timer_get_time() em vez de DISPLAY_UPDATE_MS fixo
   3. T++ com limite MAX_RADAR_TARGETS na state_machine
   4. fsm_task stack aumentada de 4096 para 6144
   5. display_manager_set_radar(NULL,0) no arranque
   6. Simulação de objectos X/Y em mm nas faixas de rodagem

============================================================ */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "system_config.h"
#include "post_config.h"
#include "wifi_manager.h"
#include "display_manager.h"
#include "comm_manager.h"
#include "udp_manager.h"
#include "dali_manager.h"
#include "radar_manager.h"
#include "state_machine.h"
#include "esp_timer.h"
#include "esp_netif.h"    /* esp_netif_init() — chamado antes do wifi */
#include "esp_event.h"    /* esp_event_loop_create_default()           */

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
   ESTADO SIMULADO DOS OBJECTOS DO RADAR
   Persiste entre chamadas de _atualiza_radar_display().
============================================================ */
#if USE_RADAR == 0

typedef struct {
    float   x_mm, y_mm, vy;
    int     trail_x[RADAR_TRAIL_MAX];
    int     trail_y[RADAR_TRAIL_MAX];
    uint8_t trail_len;
    bool    alive;
} sim_obj_t;

static sim_obj_t s_sim[RADAR_MAX_OBJ] = {0};
static bool      s_sim_init           = false;

static const float FAIXAS_MM[RADAR_MAX_OBJ] = {-1400.0f, 0.0f, 1400.0f};

static void _sim_reset(int i, float vel_kmh)
{
    s_sim[i].x_mm     = FAIXAS_MM[i] + (float)((esp_timer_get_time() & 0xFF) - 128) * 1.2f;
    s_sim[i].y_mm     = (float)(RADAR_MAX_MM) * (0.65f + (float)(i * 17 % 35) / 100.0f);
    s_sim[i].vy       = vel_kmh * 27.78f;  /* mm por ciclo de 100ms */
    s_sim[i].trail_len = 0;
    s_sim[i].alive    = true;
}

#endif /* USE_RADAR == 0 */

/* ============================================================
   _atualiza_radar_display
   ------------------------------------------------------------
   Recolhe posições X/Y dos objectos detectados e passa ao
   display_manager_set_radar() para redesenhar o canvas.

   USE_RADAR=1: chama radar_manager_get_objects() (a implementar
                no radar_manager para o protocolo completo 30 bytes).
   USE_RADAR=0: gera posições sintéticas nas faixas de rodagem
                com movimento contínuo em direcção ao sensor.

   T=0 → passa count=0 → canvas vazio (sem pontos vermelhos).
============================================================ */
static void _atualiza_radar_display(int T, float vel_kmh)
{
#if USE_RADAR == 1
    /* --- Modo real: lê do radar_manager --- */
    /* radar_manager_get_objects() deve ser implementado para
       extrair X/Y em mm do frame completo de 30 bytes          */
    radar_obj_t objs[RADAR_MAX_OBJ];
    uint8_t count = 0;
    /* count = radar_manager_get_objects(objs, RADAR_MAX_OBJ); */
    display_manager_set_radar(count > 0 ? objs : NULL, count);

#else
    /* --- Modo simulado: gera posições sintéticas --- */
    if (!s_sim_init) {
        for (int i = 0; i < RADAR_MAX_OBJ; i++) {
            _sim_reset(i, vel_kmh > 0 ? vel_kmh : 50.0f);
            s_sim[i].y_mm -= (float)(i * 1800);
            if (s_sim[i].y_mm < 500.0f) s_sim[i].y_mm = 500.0f;
        }
        s_sim_init = true;
    }

    radar_obj_t objs[RADAR_MAX_OBJ];
    uint8_t     count = 0;
    int         T_sim = (T > RADAR_MAX_OBJ) ? RADAR_MAX_OBJ : T;

    for (int i = 0; i < T_sim; i++) {
        if (!s_sim[i].alive) _sim_reset(i, vel_kmh > 0 ? vel_kmh : 50.0f);
        if (vel_kmh > 0)     s_sim[i].vy = vel_kmh * 27.78f;

        /* Guarda posição no rasto */
        if (s_sim[i].trail_len < RADAR_TRAIL_MAX) {
            s_sim[i].trail_x[s_sim[i].trail_len] = (int)s_sim[i].x_mm;
            s_sim[i].trail_y[s_sim[i].trail_len] = (int)s_sim[i].y_mm;
            s_sim[i].trail_len++;
        } else {
            memmove(&s_sim[i].trail_x[0], &s_sim[i].trail_x[1],
                    (RADAR_TRAIL_MAX - 1) * sizeof(int));
            memmove(&s_sim[i].trail_y[0], &s_sim[i].trail_y[1],
                    (RADAR_TRAIL_MAX - 1) * sizeof(int));
            s_sim[i].trail_x[RADAR_TRAIL_MAX - 1] = (int)s_sim[i].x_mm;
            s_sim[i].trail_y[RADAR_TRAIL_MAX - 1] = (int)s_sim[i].y_mm;
        }

        /* Move em direcção ao sensor */
        s_sim[i].y_mm -= s_sim[i].vy / 10.0f;

        if (s_sim[i].y_mm <= 0.0f) {
            /* Chegou ao sensor — reinicia no exterior */
            _sim_reset(i, vel_kmh > 0 ? vel_kmh : 50.0f);
            s_sim[i].y_mm = (float)RADAR_MAX_MM * (0.7f + 0.3f *
                            ((float)(esp_timer_get_time() & 0x3FF) / 1024.0f));
            continue;
        }

        objs[count].x_mm      = (int)s_sim[i].x_mm;
        objs[count].y_mm      = (int)s_sim[i].y_mm;
        objs[count].trail_len = s_sim[i].trail_len;
        memcpy(objs[count].trail_x, s_sim[i].trail_x,
               s_sim[i].trail_len * sizeof(int));
        memcpy(objs[count].trail_y, s_sim[i].trail_y,
               s_sim[i].trail_len * sizeof(int));
        count++;
    }

    /* Desactiva objectos excedentes se T diminuiu */
    for (int i = T_sim; i < RADAR_MAX_OBJ; i++) {
        s_sim[i].alive     = false;
        s_sim[i].trail_len = 0;
    }

    display_manager_set_radar(count > 0 ? objs : NULL, count);
#endif
}


/* ============================================================
   fsm_task
   ------------------------------------------------------------
   Task dedicada à máquina de estados.
   Corre a 100ms — independente do ciclo de display (500ms).
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
        bool  comm_ok   = comm_status_ok();
        bool  is_master = comm_is_master();
        /* Lê T e velocidade ANTES do update para o radar
           reflectir o estado do ciclo actual (não do próximo) */
        int   T         = state_machine_get_T();
        int   Tc        = state_machine_get_Tc();
        float vel       = state_machine_get_last_speed();

        state_machine_update(comm_ok, is_master);

        /* --- Actualiza display com estado da FSM --- */
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(T, Tc);
        display_manager_set_speed((int)vel);
        display_manager_set_hardware(state_machine_radar_ok(),
                                     dali_get_brightness());
        display_manager_set_leader(is_master);

        /* --- Actualiza canvas do radar (v5.1) ---
           T=0 → canvas vazio; T>0 → pontos a mover-se para o sensor */
        _atualiza_radar_display(T, vel);

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

    ESP_LOGI(TAG, "[1/7] NVS...");
    init_nvs();

    /* esp_netif_init() e esp_event_loop_create_default() devem ser
       chamados UMA ÚNICA VEZ em todo o sistema, antes de qualquer
       módulo que use eventos ou rede (Wi-Fi, LWIP, etc.).
       Colocados aqui em vez de dentro do wifi_manager_init()
       para evitar ESP_ERR_INVALID_STATE se chamados mais que uma vez. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "[2/7] Configuração do poste...");
    post_config_init();
    ESP_LOGI(TAG, "ID=%d | POS=%d | NOME=%s",
             post_get_id(), POST_POSITION, post_get_name());

    ESP_LOGI(TAG, "[3/7] DALI (PWM luminária)...");
    dali_init();

    ESP_LOGI(TAG, "[4/7] Radar (%s)...",
             USE_RADAR ? "HLK-LD2450" : "SIMULADO");
    radar_init(USE_RADAR ? RADAR_MODE_UART : RADAR_MODE_SIMULATED);

    ESP_LOGI(TAG, "[5/7] Display ST7789 + LVGL v5.1...");
    display_manager_init();
    display_manager_set_wifi(false, NULL);
    display_manager_set_status("IDLE");
    display_manager_set_hardware(true, LIGHT_MIN);
    display_manager_set_traffic(0, 0);
    display_manager_set_speed(0);
    display_manager_set_neighbors(NULL, NULL, false, false);
    display_manager_set_radar(NULL, 0);

    ESP_LOGI(TAG, "[6/7] Wi-Fi...");
    wifi_manager_init();

    ESP_LOGI(TAG, "[7/7] Sistema pronto");

    /* ----------------------------------------------------------
       FASE 2 — Variáveis de controlo
    ---------------------------------------------------------- */
    bool wifi_prev           = false;
    char ip_prev[MAX_IP_LEN] = "---";
    char nebL[MAX_IP_LEN];
    char nebR[MAX_IP_LEN];
    bool comm_iniciado = false;
    bool fsm_iniciada  = false;
    /* Timestamp para cálculo do delta real do LVGL tick */
    uint32_t tick_ultimo = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGI(TAG, "Sistema pronto");

    /* ----------------------------------------------------------
       CICLO PRINCIPAL — 500ms
    ---------------------------------------------------------- */
    while (1)
    {
        /* Ciclo LVGL — tick com delta real para temporização correcta */
        uint32_t agora_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t delta_ms = agora_ms - tick_ultimo;
        tick_ultimo = agora_ms;
        display_manager_tick(delta_ms > 0 ? delta_ms : 1);
        display_manager_task();

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
                        6144, NULL, 6, NULL);
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
    ESP_LOGI(TAG, " Poste Inteligente v3.1");
    ESP_LOGI(TAG, " ID=%d | POS=%d | NOME=%s",
             POSTE_ID, POST_POSITION, POSTE_NAME);
    ESP_LOGI(TAG, " Radar=%s | Display=v5.1 radar activo",
             USE_RADAR ? "HW" : "SIMULADO");
    ESP_LOGI(TAG, "============================================");

    xTaskCreate(main_task, "main_task",
                8192, NULL, 5, NULL);
}