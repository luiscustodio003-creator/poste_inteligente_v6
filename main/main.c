/* ============================================================
   MAIN — PONTO DE ENTRADA DO SISTEMA
   ------------------------------------------------------------
   @file      main.c
   @brief     Inicialização e ciclo principal do Poste Inteligente
   @version   2.4
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Ponto de entrada do sistema. Inicializa todos os módulos
   pela ordem correcta e lança a task principal que gere o
   ciclo de vida do display e da comunicação UDP.

   Ordem de inicialização:
   -----------------------
   1. NVS          — obrigatória antes de Wi-Fi e post_config
   2. post_config  — carrega ID e nome do poste da NVS
   3. display      — ST7789 + LVGL + UI (mostra estado inicial)
   4. Wi-Fi        — inicia ligação STA; notifica display via evento
   5. UDP          — iniciado apenas após Wi-Fi com IP obtido

   Lógica de estado dos vizinhos:
   --------------------------------
   O udp_manager_get_neighbors() preenche os buffers nebL/nebR
   com o IP do vizinho ou "---" se não houver vizinho conhecido.
   O estado OK/OFF é derivado directamente dessa string:
     leftOk  = (nebL[0] != '-')   → true se tiver IP real
     rightOk = (nebR[0] != '-')   → true se tiver IP real
   Quando o udp_manager for expandido (Fase 1), este critério
   será substituído pelo campo neighbor_t.status.

   Alterações v2.3 → v2.4:
   ------------------------
   1. Declaradas variáveis leftOk e rightOk derivadas do IP
   2. display_manager_set_neighbors() actualizado para v3.0
      (recebe leftOk e rightOk como parâmetros adicionais)
   3. Adicionadas chamadas de teste ao display (comentadas)
      para validar o layout sem necessidade de radar ou FSM

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
#include "udp_manager.h"

/* Etiqueta de log deste módulo */
static const char *TAG = "MAIN";

/* ============================================================
   init_nvs
   ------------------------------------------------------------
   Inicializa a flash NVS. Se a partição estiver corrompida
   ou com versão diferente, apaga e reinicializa.
   OBRIGATÓRIO antes de Wi-Fi e post_config_init().
============================================================ */
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS corrompida ou versão diferente — a limpar...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializada");
}

/* ============================================================
   main_task
   ------------------------------------------------------------
   Task principal do sistema. Inicializa todos os módulos
   pela ordem correcta e entra no ciclo de actualização.

   Stack: 8192 bytes (LVGL requer mínimo ~6 KB)
   Prioridade: 5
============================================================ */
static void main_task(void *arg)
{
    /* ----------------------------------------------------------
       FASE 1 — Inicialização dos módulos (ordem obrigatória)
    ---------------------------------------------------------- */

    /* 1. NVS — base para Wi-Fi e post_config */
    ESP_LOGI(TAG, "[1/4] A inicializar NVS...");
    init_nvs();

    /* 2. Configuração persistente do poste (ID e nome da NVS) */
    ESP_LOGI(TAG, "[2/4] A carregar configuração do poste...");
    post_config_init();
    ESP_LOGI(TAG, "Poste: ID=%d | NOME=%s",
             post_get_id(), post_get_name());

    /* 3. Display — ST7789 + LVGL + interface gráfica */
    ESP_LOGI(TAG, "[3/4] A inicializar display...");
    display_manager_init();

    /* Estado inicial do display logo no arranque — tudo offline */
    display_manager_set_wifi(false, NULL);
    display_manager_set_status("IDLE");
    display_manager_set_hardware(false, LIGHT_MIN);
    display_manager_set_traffic(0, 0);
    display_manager_set_speed(0);
    display_manager_set_neighbors(NULL, NULL, false, false);

    /* ----------------------------------------------------------
       BLOCO DE TESTE DO DISPLAY — remover após validação visual
       Descomenta para ver o layout completo sem hardware:

    display_manager_set_wifi(true, "192.168.1.42");
    display_manager_set_status("LIGHT ON");
    display_manager_set_hardware(true, 100);
    display_manager_set_traffic(2, 1);
    display_manager_set_speed(47);
    display_manager_set_neighbors("192.168.1.41", "192.168.1.43",
                                  true, false);
    ---------------------------------------------------------- */

    /* 4. Wi-Fi — inicia ligação STA */
    ESP_LOGI(TAG, "[4/4] A inicializar Wi-Fi...");
    wifi_manager_init();

    /* ----------------------------------------------------------
       FASE 2 — Variáveis de controlo do ciclo principal
    ---------------------------------------------------------- */

    /* Controlo de mudança de estado Wi-Fi */
    bool wifi_prev          = false;
    char ip_prev[MAX_IP_LEN] = "---";

    /* Buffers para IPs dos vizinhos UDP */
    char nebL[MAX_IP_LEN];
    char nebR[MAX_IP_LEN];

    /* Flag: UDP iniciado (só após Wi-Fi com IP) */
    bool udp_iniciado = false;

    ESP_LOGI(TAG, "Sistema pronto — a entrar no ciclo principal");

    /* ----------------------------------------------------------
       CICLO PRINCIPAL
       Período: DISPLAY_UPDATE_MS (definido em system_config.h)
    ---------------------------------------------------------- */
    while (1)
    {
        /* --- Ciclo LVGL: processar eventos e timers --- */
        display_manager_task();
        display_manager_tick(DISPLAY_UPDATE_MS);

        /* --------------------------------------------------
           LÓGICA DE COR DO WI-FI
           Polling do estado actual. Actualiza o display apenas
           quando o estado ou o IP mudam (evita redraws).
        -------------------------------------------------- */
        bool        wifi_curr = wifi_manager_is_connected();
        const char *ip_curr   = wifi_manager_get_ip();

        if (wifi_curr != wifi_prev ||
            strncmp(ip_curr, ip_prev, MAX_IP_LEN) != 0)
        {
            if (wifi_curr)
            {
                ESP_LOGI(TAG, "Wi-Fi ON | IP: %s", ip_curr);
                display_manager_set_wifi(true, ip_curr);
            }
            else
            {
                ESP_LOGI(TAG, "Wi-Fi OFF");
                display_manager_set_wifi(false, NULL);
            }

            wifi_prev = wifi_curr;
            strncpy(ip_prev, ip_curr, MAX_IP_LEN - 1);
            ip_prev[MAX_IP_LEN - 1] = '\0';
        }

        /* --------------------------------------------------
           INICIALIZAÇÃO UDP (apenas uma vez, após ter IP)
        -------------------------------------------------- */
        if (!udp_iniciado && wifi_curr)
        {
            ESP_LOGI(TAG, "Wi-Fi com IP — a inicializar UDP...");
            udp_manager_init();
            udp_iniciado = true;
        }

        /* --------------------------------------------------
           ACTUALIZAÇÃO DE VIZINHOS UDP
           --------------------------------------------------
           udp_manager_get_neighbors() preenche nebL/nebR com
           o IP do vizinho ou "---" se não conhecido.

           leftOk  → true se nebL tem IP real (não começa com '-')
           rightOk → true se nebR tem IP real (não começa com '-')

           Quando o udp_manager for expandido (Fase 1 do plano),
           este critério será substituído por neighbor_t.status.
        -------------------------------------------------- */
        if (udp_iniciado)
        {
            udp_manager_get_neighbors(nebL, nebR);

            /* Deriva estado OK/OFF a partir do conteúdo do buffer */
            bool leftOk  = (nebL[0] != '\0' && nebL[0] != '-');
            bool rightOk = (nebR[0] != '\0' && nebR[0] != '-');

            display_manager_set_neighbors(nebL, nebR, leftOk, rightOk);
        }

        /* Aguarda próximo ciclo */
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

/* ============================================================
   app_main
   ------------------------------------------------------------
   Ponto de entrada do ESP-IDF. Lança a task principal.
============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, " Poste Inteligente — a arrancar...");
    ESP_LOGI(TAG, " ID: %d | NOME: %s", POSTE_ID, POSTE_NAME);
    ESP_LOGI(TAG, "============================================");

    xTaskCreate(main_task,
                "main_task",
                8192,
                NULL,
                5,
                NULL);
}