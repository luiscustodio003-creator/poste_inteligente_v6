/* ============================================================
   MAIN — PONTO DE ENTRADA DO SISTEMA
   ------------------------------------------------------------
   @file      main.c
   @brief     Inicialização e ciclo principal do Poste Inteligente
   @version   3.4
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Alterações v3.3 → v3.4:
   ------------------------
   CORRECÇÃO 1: _atualiza_radar_display() — variável `obj`
     declarada como `radar_obj_t` (correcto). O parâmetro de
     display_manager_set_radar() é `const radar_obj_t *`, que
     aceita um ponteiro para variável não-const sem cast explícito
     (conversão implícita C89/C99). O aviso anterior era porque
     o compilador via duas definições incompatíveis de radar_obj_t
     — resolvido ao centralizar o tipo em radar_manager.h (v2.1).

   CORRECÇÃO 2: Activada a chamada real a
     radar_manager_get_objects() no modo USE_RADAR=1.
     O TODO anterior foi substituído pela chamada correcta.

   CORRECÇÃO 3: Inclusão directa de "radar_manager.h" removida
     do bloco de includes — vem transitivamente via
     display_manager.h (que inclui radar_manager.h).
     Mantida explicitamente para clareza de dependências.

   Ordem de inicialização (inalterada):
   -------------------------------------
   1. NVS             — obrigatória antes de Wi-Fi e post_config
   2. esp_netif_init  — singleton de sistema (antes do Wi-Fi)
   3. post_config     — carrega ID e nome do poste da NVS
   4. dali_manager    — PWM da luminária
   5. radar_manager   — sensor HLK-LD2450 ou modo simulado
   6. display         — ST7789 + LVGL + UI
   7. Wi-Fi           — inicia ligação STA
   8. comm_manager    — UDP (após Wi-Fi com IP)
   9. state_machine   — FSM (após comm_manager)

   Tasks:
   ------
   main_task — display + Wi-Fi + comm (DISPLAY_UPDATE_MS = 500ms)
   fsm_task  — radar + state_machine (100ms)

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

#include "system_config.h"
#include "post_config.h"
#include "wifi_manager.h"
#include "display_manager.h"   /* inclui transitivamente radar_manager.h */
#include "radar_manager.h"     /* explícito para clareza de dependências  */
#include "comm_manager.h"
#include "udp_manager.h"
#include "dali_manager.h"
#include "state_machine.h"

static const char *TAG = "MAIN";

/* Flags de teste manual — activadas via monitor série */
volatile bool  g_test_car       = false;
volatile float g_test_car_speed = 50.0f;

/* ============================================================
   init_nvs
   ------------------------------------------------------------
   Inicializa a NVS. Apaga partição se corrompida ou com
   versão incompatível (garante arranque limpo).
============================================================ */
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS corrompida — a limpar partição...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializada");
}

/* ============================================================
   _atualiza_radar_display
   ------------------------------------------------------------
   Actualiza o canvas do radar com a posição actual do carro.

   USE_RADAR=0 (simulado):
     Lê posição do simulador via sim_get_objeto().
     A lógica de detecção e injecção de eventos está em
     state_machine.c — o main.c apenas faz visualização.

   USE_RADAR=1 (hardware HLK-LD2450):
     Chama radar_manager_get_objects() que lê o frame UART
     de 30 bytes e extrai coordenadas X/Y em mm.
     Todos os objectos activos são enviados ao canvas.

   NOTA v3.4: radar_obj_t vem de radar_manager.h (v2.1).
   display_manager_set_radar() aceita `const radar_obj_t *`,
   compatível com ponteiro para radar_obj_t local (C99 §6.3.2.3).
============================================================ */
static void _atualiza_radar_display(void)
{
#if USE_RADAR == 0
    /* ---- Modo simulado ---- */
    float x_mm = 0.0f, y_mm = 0.0f;

    if (sim_get_objeto(&x_mm, &y_mm))
    {
        /* Carro visível — envia posição ao canvas */
        radar_obj_t obj = {0};
        obj.x_mm      = (int)x_mm;
        obj.y_mm      = (int)y_mm;
        obj.trail_len = 0; /* rasto gerido pelo simulador */

        /* CORRECÇÃO v3.4: &obj é radar_obj_t*, compatível com
           const radar_obj_t* — conversão implícita válida em C */
        display_manager_set_radar(&obj, 1);
    }
    else
    {
        /* Sem carro visível — canvas vazio */
        display_manager_set_radar(NULL, 0);
    }

#else
    /* ---- Modo hardware HLK-LD2450 ---- */

    /*
     * CORRECÇÃO v3.4: radar_manager_get_objects() activada.
     * Declarada em radar_manager.h (única fonte) — sem conflito.
     * Retorna número de alvos activos (0..RADAR_MAX_OBJ).
     */
    radar_obj_t objs[RADAR_MAX_OBJ];
    uint8_t count = radar_manager_get_objects(objs, RADAR_MAX_OBJ);

    display_manager_set_radar(count > 0 ? objs : NULL, count);
#endif
}

/* ============================================================
   fsm_task — período 100ms
   ------------------------------------------------------------
   Gere o ciclo da máquina de estados:
     1. Verifica flag de teste manual
     2. Atualiza FSM com estado da comunicação
     3. Actualiza display com métricas da FSM
     4. Actualiza canvas do radar
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG, "FSM task iniciada | período=100ms");

    /* Aguarda estabilização do sistema no arranque */
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        /* Injecção de carro de teste via flag global */
        if (g_test_car)
        {
            g_test_car = false;
            sm_inject_test_car(g_test_car_speed);
        }

        /* Avalia estado da comunicação e liderança */
        bool comm_ok   = comm_status_ok();
        bool is_master = comm_is_master();

        /* Ciclo principal da FSM */
        state_machine_update(comm_ok, is_master);

        /* Leitura dos contadores e métricas */
        int   T   = state_machine_get_T();
        int   Tc  = state_machine_get_Tc();
        float vel = state_machine_get_last_speed();

        /* Actualização do display com todos os parâmetros */
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(T, Tc);
        display_manager_set_speed((int)vel);
        display_manager_set_hardware(state_machine_radar_ok(),
                                     dali_get_brightness());
        display_manager_set_leader(is_master);

        /* Actualização do canvas radar */
        _atualiza_radar_display();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   main_task — período DISPLAY_UPDATE_MS (500ms)
   ------------------------------------------------------------
   Sequência de inicialização e ciclo de supervisão:
     - Inicializa todos os subsistemas por ordem
     - Monitoriza Wi-Fi e activa comm_manager ao ligar
     - Lança fsm_task após comm_manager inicializado
     - Actualiza vizinhos e display a cada 500ms
============================================================ */
static void main_task(void *arg)
{
    /* ---- Inicialização sequencial ---- */

    ESP_LOGI(TAG, "[1/7] NVS...");
    init_nvs();

    /* Singleton de rede — deve ser chamado antes do Wi-Fi */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "[2/7] Configuração do poste (NVS)...");
    post_config_init();
    ESP_LOGI(TAG, "      ID=%d | POS=%d | NOME=%s",
             post_get_id(), POST_POSITION, post_get_name());

    ESP_LOGI(TAG, "[3/7] DALI (PWM luminária)...");
    dali_init();

    ESP_LOGI(TAG, "[4/7] Radar (%s)...",
             USE_RADAR ? "HLK-LD2450 UART" : "SIMULADO");
    radar_init(USE_RADAR ? RADAR_MODE_UART : RADAR_MODE_SIMULATED);

    ESP_LOGI(TAG, "[5/7] Display ST7789 + LVGL v5.1...");
    display_manager_init();

    /* Estado inicial do display — todos os campos em repouso */
    display_manager_set_wifi(false, NULL);
    display_manager_set_status("IDLE");
    display_manager_set_hardware(true, LIGHT_MIN);
    display_manager_set_traffic(0, 0);
    display_manager_set_speed(0);
    display_manager_set_neighbors(NULL, NULL, false, false);
    display_manager_set_radar(NULL, 0);

    ESP_LOGI(TAG, "[6/7] Wi-Fi STA...");
    wifi_manager_init();

    ESP_LOGI(TAG, "[7/7] Sistema pronto | aguardar Wi-Fi...");

    /* ---- Ciclo de supervisão ---- */

    bool wifi_prev           = false;
    char ip_prev[MAX_IP_LEN] = "---";
    char nebL[MAX_IP_LEN];
    char nebR[MAX_IP_LEN];
    bool comm_iniciado = false;
    bool fsm_iniciada  = false;

    uint32_t tick_ultimo =
        (uint32_t)(esp_timer_get_time() / 1000ULL);

    while (1)
    {
        /* Ticker LVGL — necessário a cada iteração */
        uint32_t agora_ms =
            (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t delta_ms = agora_ms - tick_ultimo;
        tick_ultimo = agora_ms;
        display_manager_tick(delta_ms > 0 ? delta_ms : 1);
        display_manager_task();

        /* Detecta mudança de estado Wi-Fi */
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
                     wifi_curr ? "LIGADO" : "DESLIGADO",
                     ip_curr);
        }

        /* Inicializa comm_manager na primeira ligação Wi-Fi */
        if (!comm_iniciado && wifi_curr)
        {
            if (comm_init())
            {
                comm_iniciado = true;
                ESP_LOGI(TAG, "Comm inicializado | MASTER=%s",
                         comm_is_master() ? "SIM" : "NAO");
            }
        }

        /* Lança FSM após comm_manager pronto */
        if (!fsm_iniciada && comm_iniciado)
        {
            state_machine_init();
            xTaskCreate(fsm_task, "fsm_task",
                        6144, NULL, 6, NULL);
            fsm_iniciada = true;
            ESP_LOGI(TAG, "FSM task lançada");
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
   ------------------------------------------------------------
   Ponto de entrada do ESP-IDF. Cria a main_task e termina.
   Toda a lógica de sistema corre nas tasks FreeRTOS.
============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, " Poste Inteligente v3.4");
    ESP_LOGI(TAG, " ID=%d | POS=%d | NOME=%s",
             POSTE_ID, POST_POSITION, POSTE_NAME);
    ESP_LOGI(TAG, " Radar=%s",
             USE_RADAR ? "HW HLK-LD2450" : "SIMULADO");
    ESP_LOGI(TAG, "============================================");

    /* Stack de 8KB é suficiente para inicialização + ciclo */
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}