/* ============================================================
   WIFI MANAGER — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Gere conectividade Wi-Fi com fallback AP e reconexao
   em background. Corrige o problema de IP apagado quando
   o AP esta activo e eventos STA continuam a disparar.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_wifi.html
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
   Estado interno do modulo
   ----------------------------------------------------------- */
static bool  s_connected   = false;
static int   s_retry_num   = 0;
static char  s_ip[16]      = "0.0.0.0";
static bool  s_initialized = false;

/* Flag que indica que o AP proprio esta activo.
   Quando true, eventos STA_DISCONNECTED sao ignorados
   para nao apagarem o IP do AP. */
static bool  s_ap_active   = false;

/* ===========================================================
   EVENT HANDLER — Eventos Wi-Fi e IP
   =========================================================== */

static void event_handler(void *arg,
                           esp_event_base_t base,
                           int32_t          id,
                           void            *data)
{
    /* STA iniciou -- tenta ligar */
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }

    /* STA desligou -- tenta reconectar se AP nao activo */
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        /* Se AP ja esta activo, ignora eventos STA.
           Sem esta guarda, o IP do AP seria apagado
           a cada evento de desconexao STA. */
        if (s_ap_active)
        {
            ESP_LOGD(TAG, "STA evento ignorado -- AP activo");
            return;
        }

        s_connected = false;
        strcpy(s_ip, "0.0.0.0");

        if (s_retry_num < WIFI_RETRY_ATTEMPTS)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Reconectar Wi-Fi (%d/%d)",
                     s_retry_num, WIFI_RETRY_ATTEMPTS);
        }
        else
        {
            ESP_LOGW(TAG, "Rede nao encontrada -- a criar AP");
        }
    }

    /* STA obteve IP -- ligacao bem sucedida */
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&ev->ip_info.ip, s_ip, sizeof(s_ip));
        s_connected = true;
        s_retry_num = 0;
        s_ap_active = false; /* STA ligou -- AP ja nao e necessario */
        ESP_LOGI(TAG, "STA ligado | IP=%s", s_ip);
    }
}

/* ===========================================================
   ACCESS POINT — Rede propria do poste
   =========================================================== */

static void wifi_start_ap(void)
{
    ESP_LOGW(TAG, "Criar AP proprio");

    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_cfg = {0};
    strcpy((char*)ap_cfg.ap.ssid,     NETWORK_SSID);
    strcpy((char*)ap_cfg.ap.password, NETWORK_PASS);
    ap_cfg.ap.ssid_len       = strlen(NETWORK_SSID);
    ap_cfg.ap.max_connection = 10;
    ap_cfg.ap.authmode       = strlen(NETWORK_PASS) == 0 ?
                               WIFI_AUTH_OPEN : WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Marca AP activo ANTES de guardar IP.
       Assim eventos STA posteriores nao apagam o IP. */
    s_ap_active = true;
    strcpy(s_ip, NETWORK_IP);

    ESP_LOGI(TAG, "AP activo | SSID=%s | IP=%s", NETWORK_SSID, NETWORK_IP);
}

/* ===========================================================
   INICIALIZACAO DO STACK WI-FI
   =========================================================== */

void wifi_init(void)
{
    /* Guarda contra inicializacao dupla */
    if (s_initialized) return;

    ESP_ERROR_CHECK(esp_netif_init());

    /* Cria event loop -- ignora se ja existir */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(ret);

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    s_initialized = true;
    ESP_LOGI(TAG, "Wi-Fi inicializado");
}

/* ===========================================================
   LIGACAO STA
   =========================================================== */

void wifi_connect(void)
{
    wifi_config_t cfg = {0};
    strcpy((char*)cfg.sta.ssid,     WIFI_SSID);
    strcpy((char*)cfg.sta.password, WIFI_PASS);

    s_retry_num = 0;
    s_ap_active = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
}

/* ===========================================================
   MODO AUTOMATICO — STA com fallback AP
   =========================================================== */

void wifi_start_auto(void)
{
    wifi_init();
    wifi_connect();

    /* Aguarda ate WIFI_RETRY_ATTEMPTS segundos pela ligacao STA */
    for (int i = 0; i < WIFI_RETRY_ATTEMPTS; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (wifi_is_connected())
        {
            ESP_LOGI(TAG, "Ligado a rede externa");
            return;
        }
    }

    /* STA falhou -- cria AP proprio */
    wifi_start_ap();
}

/* ===========================================================
   RECONEXAO EM BACKGROUND
   -----------------------------------------------------------
   Chamada periodicamente pela wifi_reconnect_task.
   Tenta ligar como STA se nao estiver ligado.
   Nao interfere se AP activo -- permite coexistencia APSTA.
   =========================================================== */

void wifi_try_reconnect(void)
{
    /* Nao tenta se ja ligado */
    if (s_connected) return;

    ESP_LOGI(TAG, "Tentativa reconexao STA em background");

    /* Tenta ligar sem recriar o stack -- apenas reconecta */
    wifi_config_t cfg = {0};
    strcpy((char*)cfg.sta.ssid,     WIFI_SSID);
    strcpy((char*)cfg.sta.password, WIFI_PASS);

    /* Se estiver em modo AP, muda para APSTA para tentar STA */
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    if (mode == WIFI_MODE_AP)
    {
        esp_wifi_set_mode(WIFI_MODE_APSTA);
    }

    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_connect();
}

/* ===========================================================
   GETTERS DE ESTADO
   =========================================================== */

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
    return s_ip;
}