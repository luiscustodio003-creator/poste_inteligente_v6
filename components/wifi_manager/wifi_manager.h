/* ============================================================
   WIFI MANAGER — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Gestao da conectividade Wi-Fi com reconexao em background.
   Fluxo: tenta STA durante WIFI_RETRY_ATTEMPTS segundos.
   Se falhar, cria AP proprio (APSTA). Continua a tentar
   reconectar em background a cada WIFI_RECONNECT_MS ms.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_wifi.html

   Dependencias:
   -------------
   - system_config.h
   - esp_wifi, esp_event, esp_netif
   - NVS ja inicializada em app_main
============================================================ */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

/* Inicializa stack Wi-Fi */
void wifi_init(void);

/* Liga como STA */
void wifi_connect(void);

/* Modo automatico: STA com fallback AP. Chamada unica em app_main. */
void wifi_start_auto(void);

/* Tentativa de reconexao em background.
   Chamada periodicamente pela wifi_reconnect_task. */
void wifi_try_reconnect(void);

/* true se STA ligado com IP valido */
bool wifi_is_connected(void);

/* true se AP activo */
bool wifi_is_ap_active(void);

/* Retorna IP actual como string */
const char *wifi_get_ip(void);

#endif /* WIFI_MANAGER_H */