/* ============================================================
   WIFI MANAGER — DECLARAÇÃO
   ------------------------------------------------------------
   @file      wifi_manager.h
   @brief     Gestão automática de rede Wi-Fi para postes
   @version   3.1
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alterações (v3.0 → v3.1):
   --------------------------
   1. Documentação actualizada: nvs_flash_init() é agora
      responsabilidade exclusiva de app_main(). wifi_init()
      assume que a NVS já está inicializada quando é chamado.
   2. Sem alterações de código — apenas documentação.

   Fluxo automático (wifi_start_auto):
   ------------------------------------
   1. Inicializa stack Wi-Fi (netif, event loop) — SEM NVS
   2. Tenta ligar como STA à rede definida em system_config.h
   3. Se não conseguir após WIFI_RETRY_ATTEMPTS tentativas,
      cria Access Point próprio (modo APSTA)
   4. Mantém estado interno acessível via getters

   Ordem correcta de chamada em app_main():
   -----------------------------------------
   nvs_flash_init()       ← AQUI — uma vez, antes de tudo
   display_manager_start()
   post_config_init()
   wifi_start_auto()      ← chama wifi_init() internamente
   comm_init()
   state_machine_init()

   Integração com display_manager e udp_manager:
   -----------------------------------------------
   display_manager_set_wifi(wifi_is_connected());
   display_manager_set_ap_mode(wifi_is_ap_active(), wifi_get_ip());
   udp_manager usa wifi_get_ip() para ignorar pacotes próprios.

   Dependências:
   -------------
   - system_config.h (WIFI_SSID, WIFI_PASS, NETWORK_*,
                      WIFI_RETRY_ATTEMPTS)
   - esp_wifi, esp_event, esp_netif
   - NVS já inicializada por app_main()
============================================================ */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

/* -----------------------------------------------------------
   wifi_init — Inicializa stack Wi-Fi (sem NVS)

   Inicializa: esp_netif, event loop (se não existir),
   interface STA, driver Wi-Fi, handlers de eventos.
   Segura contra chamadas múltiplas (guarda s_initialized).

   PRÉ-REQUISITO: nvs_flash_init() deve ter sido chamado
   em app_main() antes desta função.
   ----------------------------------------------------------- */
void wifi_init(void);

/* -----------------------------------------------------------
   wifi_connect — Liga como STA à rede em system_config.h
   ----------------------------------------------------------- */
void wifi_connect(void);

/* -----------------------------------------------------------
   wifi_start_auto — Modo automático STA → AP

   1. Chama wifi_init() e wifi_connect()
   2. Aguarda WIFI_RETRY_ATTEMPTS segundos pela ligação STA
   3. Se não ligar, cria Access Point próprio (APSTA)

   Função recomendada para usar em app_main().
   ----------------------------------------------------------- */
void wifi_start_auto(void);

/* -----------------------------------------------------------
   wifi_is_connected — true se STA ligado com IP válido
   ----------------------------------------------------------- */
bool wifi_is_connected(void);

/* -----------------------------------------------------------
   wifi_is_ap_active — true se AP activo (AP ou APSTA)
   ----------------------------------------------------------- */
bool wifi_is_ap_active(void);

/* -----------------------------------------------------------
   wifi_get_ip — Retorna IP actual como string

   STA ligado  → IP do router    (ex: "192.168.1.5")
   AP activo   → IP fixo do AP   (NETWORK_IP)
   Desligado   → "0.0.0.0"
   ----------------------------------------------------------- */
const char *wifi_get_ip(void);

#endif /* WIFI_MANAGER_H */