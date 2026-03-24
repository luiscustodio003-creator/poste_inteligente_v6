/* ============================================================
   WIFI MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      wifi_manager.c
   @brief     Gestão da ligação Wi-Fi STA (DHCP)
   @version   1.5
   @date      2026-03-23

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere a ligação Wi-Fi em modo STA com DHCP. Quando o estado
   muda (ligado/desligado/IP obtido), o main.c deteta a mudança
   via wifi_manager_is_connected() e chama display_manager_set_wifi().
   Esta versão (v1.5) remove a dependência directa do display_manager
   para eliminar a dependência circular no grafo de build do ESP-IDF.

   Alterações v1.4 → v1.5:
   ------------------------
   1. Removido #include "display_manager.h"
   2. Removidas chamadas display_manager_set_wifi() do handler
   3. Removido display_manager do REQUIRES do CMakeLists
   O main.c (que depende de ambos) faz a ponte no ciclo de 500ms.
============================================================ */

#include "wifi_manager.h"
#include "system_config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <string.h>

/* Etiqueta de log deste módulo */
static const char *TAG = "WIFI_MGR";

/* Estado interno do módulo */
static bool wifi_connected = false;
static char ip_addr[16]    = "---";
static int  retry_count    = 0;

/* ============================================================
   wifi_event_handler
   ------------------------------------------------------------
   Handler de eventos Wi-Fi e IP. Chamado pelo sistema de
   eventos do ESP-IDF quando o estado da ligação muda.
============================================================ */
static void wifi_event_handler(void *arg,
                                esp_event_base_t event_base,
                                int32_t          event_id,
                                void            *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            /* Wi-Fi iniciado — tenta primeira ligação */
            esp_wifi_connect();
            ESP_LOGI(TAG, "Wi-Fi STA iniciado, a tentar ligar...");
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            wifi_connected = false;
            strncpy(ip_addr, "---", sizeof(ip_addr));
            /* Notificação do display feita pelo main.c no ciclo de 500ms */

            if (retry_count < WIFI_RETRY_ATTEMPTS)
            {
                retry_count++;
                ESP_LOGI(TAG,
                         "Desconectado, a reconectar... (%d/%d)",
                         retry_count, WIFI_RETRY_ATTEMPTS);
                esp_wifi_connect();
            }
            else
            {
                ESP_LOGW(TAG,
                         "Falha após %d tentativas. "
                         "Chamar wifi_manager_reset_retry() para retomar.",
                         WIFI_RETRY_ATTEMPTS);
            }
        }
    }
    else if (event_base == IP_EVENT &&
             event_id   == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        /* Converte IP para string */
        const char *ip_str = esp_ip4addr_ntoa(&event->ip_info.ip,
                                               ip_addr, sizeof(ip_addr));
        if (ip_str == NULL)
        {
            /* Fallback: formatar manualmente */
            uint32_t ip = event->ip_info.ip.addr;
            snprintf(ip_addr, sizeof(ip_addr), "%d.%d.%d.%d",
                     (int)(ip & 0xFF),
                     (int)((ip >> 8)  & 0xFF),
                     (int)((ip >> 16) & 0xFF),
                     (int)((ip >> 24) & 0xFF));
        }

        ip_addr[sizeof(ip_addr) - 1] = '\0';
        wifi_connected = true;
        retry_count    = 0;

        ESP_LOGI(TAG, "Wi-Fi ligado | IP: %s", ip_addr);
        /* Notificação do display feita pelo main.c no ciclo de 500ms */
    }
}

/* ============================================================
   wifi_manager_init
   ------------------------------------------------------------
   Inicializa a stack de rede, regista handlers de evento e
   inicia a ligação Wi-Fi em modo STA com as credenciais
   definidas em system_config.h.
============================================================ */
void wifi_manager_init(void)
{
    /* NOTA: esp_netif_init() e esp_event_loop_create_default()
       são chamados em main.c antes desta função.
       Chamá-los aqui causaria ESP_ERR_INVALID_STATE.            */

    /* Cria interface STA padrão */
    esp_netif_create_default_wifi_sta();

    /* Inicializa stack Wi-Fi com configuração por omissão */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* Regista handler para todos os eventos Wi-Fi */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_any_id);

    /* Regista handler específico para obtenção de IP */
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_got_ip);

    /* Configura credenciais Wi-Fi de system_config.h */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid      = WIFI_SSID,
            .password  = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable  = true,
                .required = false,
            },
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Wi-Fi iniciado | SSID: %s", WIFI_SSID);
}

/* ============================================================
   wifi_manager_is_connected
   ------------------------------------------------------------
   Retorna true se Wi-Fi está ligado e IP obtido.
============================================================ */
bool wifi_manager_is_connected(void)
{
    return wifi_connected;
}

/* ============================================================
   wifi_manager_get_ip
   ------------------------------------------------------------
   Retorna o IP actual como string. Retorna "---" se não ligado.
   O ponteiro é válido durante toda a vida do programa.
============================================================ */
const char *wifi_manager_get_ip(void)
{
    return ip_addr;
}

/* ============================================================
   wifi_manager_reset_retry
   ------------------------------------------------------------
   Repõe o contador de tentativas e força nova tentativa
   de ligação. Útil para recuperação manual após falha.
============================================================ */
void wifi_manager_reset_retry(void)
{
    retry_count = 0;
    ESP_LOGI(TAG, "Contador de tentativas reposto. A reconectar...");
    esp_wifi_connect();
}