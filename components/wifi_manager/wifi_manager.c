/* ============================================================
   WIFI MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      wifi_manager.c
   @brief     Gestão automática de rede Wi-Fi para postes
   @version   3.1
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alterações (v3.0 → v3.1):
   --------------------------
   1. CORRECÇÃO: removido nvs_flash_init() de wifi_init().
      A NVS era inicializada aqui E em post_config_init(),
      causando inicialização dupla. A NVS é agora inicializada
      EXCLUSIVAMENTE em app_main() antes de qualquer módulo.
      wifi_init() assume que nvs_flash_init() já foi chamado.

   Descrição:
   ----------
   Gere automaticamente a conectividade Wi-Fi:
   1. Tenta ligar como STA à rede definida em system_config.h
   2. Se falhar após WIFI_RETRY_ATTEMPTS tentativas, cria AP
   3. Mantém estado interno acessível via getters públicos

   Integração com outros módulos:
   --------------------------------
   - display_manager usa wifi_is_connected() e wifi_get_ip()
     para actualizar ícone e label IP no ecrã
   - udp_manager usa wifi_get_ip() para filtrar pacotes
     enviados pelo próprio poste
   - comm_manager usa wifi_is_connected() e wifi_is_ap_active()
     para verificar se a comunicação está operacional

   Dependências:
   -------------
   - wifi_manager.h
   - system_config.h  (credenciais e parâmetros)
   - esp_wifi, esp_event, esp_netif, freertos
   - NVS já inicializada por app_main()
============================================================ */

#include "wifi_manager.h"
#include "system_config.h"

#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WIFI_MGR";

/* -----------------------------------------------------------
   Estado interno do módulo
   ----------------------------------------------------------- */
static bool  s_connected   = false;      /* STA ligado com IP   */
static int   s_retry_num   = 0;          /* Contador de retries */
static char  s_ip_addr[16] = "0.0.0.0";  /* IP actual           */
static bool  s_initialized = false;      /* Guarda dupla init   */

static esp_netif_t *ap_netif = NULL;

/* ============================================================
   EVENT HANDLER — Eventos Wi-Fi e IP
   ============================================================ */

static void event_handler(void            *arg,
                           esp_event_base_t event_base,
                           int32_t          event_id,
                           void            *event_data)
{
    /* STA iniciou — tenta ligar */
    if (event_base == WIFI_EVENT &&
        event_id   == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }

    /* STA perdeu ligação — tenta reconectar */
    else if (event_base == WIFI_EVENT &&
             event_id   == WIFI_EVENT_STA_DISCONNECTED)
    {
        s_connected = false;
        strcpy(s_ip_addr, "0.0.0.0");

        if (s_retry_num < WIFI_RETRY_ATTEMPTS)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Reconectar Wi-Fi (%d/%d)",
                     s_retry_num, WIFI_RETRY_ATTEMPTS);
        }
        else
        {
            ESP_LOGW(TAG, "Rede não encontrada — a criar AP");
        }
    }

    /* STA obteve IP */
    else if (event_base == IP_EVENT &&
             event_id   == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        esp_ip4addr_ntoa(&event->ip_info.ip,
                         s_ip_addr,
                         sizeof(s_ip_addr));

        s_connected = true;
        s_retry_num = 0;

        ESP_LOGI(TAG, "STA ligado | IP=%s", s_ip_addr);
    }
}

/* ============================================================
   ACCESS POINT — Rede própria do poste
   ============================================================ */

static void wifi_start_ap(void)
{
    ESP_LOGW(TAG, "Criar rede própria (AP)");

    ap_netif = esp_netif_create_default_wifi_ap();

    wifi_config_t ap_config = {0};

    strcpy((char *)ap_config.ap.ssid,     NETWORK_SSID);
    strcpy((char *)ap_config.ap.password, NETWORK_PASS);

    ap_config.ap.ssid_len       = strlen(NETWORK_SSID);
    ap_config.ap.max_connection = 10;
    ap_config.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(NETWORK_PASS) == 0)
        ap_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    strcpy(s_ip_addr, NETWORK_IP);

    ESP_LOGI(TAG, "AP activo | SSID=%s | IP=%s",
             NETWORK_SSID, NETWORK_IP);
}

/* ============================================================
   INICIALIZAÇÃO DO STACK WI-FI
   -----------------------------------------------------------
   NOTA: nvs_flash_init() foi REMOVIDO desta função.
   A NVS é inicializada em app_main() antes de qualquer módulo.
   Esta função assume que a NVS já está pronta.
   ============================================================ */

void wifi_init(void)
{
    /* Guarda contra inicialização dupla */
    if (s_initialized) return;

    /* Inicializa stack de rede */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Cria event loop (ignora se já existir — pode ter sido
       criado por outro módulo antes desta chamada)          */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(ret);

    /* Cria interface STA por defeito */
    esp_netif_create_default_wifi_sta();

    /* Inicializa driver Wi-Fi com configuração por defeito */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Regista handler para todos os eventos Wi-Fi */
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &event_handler,
            NULL,
            NULL
        )
    );

    /* Regista handler para evento de obtenção de IP */
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &event_handler,
            NULL,
            NULL
        )
    );

    s_initialized = true;
    ESP_LOGI(TAG, "Wi-Fi inicializado");
}

/* ============================================================
   LIGAÇÃO STA — Liga à rede externa
   ============================================================ */

void wifi_connect(void)
{
    wifi_config_t wifi_config = {0};

    strcpy((char *)wifi_config.sta.ssid,     WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    s_retry_num = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_connect();
}

/* ============================================================
   MODO AUTOMÁTICO — STA com fallback para AP
   ============================================================ */

void wifi_start_auto(void)
{
    wifi_init();
    wifi_connect();

    /* Aguarda até WIFI_RETRY_ATTEMPTS segundos pela ligação STA */
    for (int i = 0; i < WIFI_RETRY_ATTEMPTS; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (wifi_is_connected())
        {
            ESP_LOGI(TAG, "Ligado à rede existente");
            return;
        }
    }

    /* STA falhou — cria AP próprio */
    wifi_start_ap();
}

/* ============================================================
   GETTERS DE ESTADO
   ============================================================ */

bool wifi_is_connected(void)
{
    return s_connected;
}

bool wifi_is_ap_active(void)
{
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    return (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
}

const char *wifi_get_ip(void)
{
    return s_ip_addr;
}