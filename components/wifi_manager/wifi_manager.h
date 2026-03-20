/* ============================================================
   WIFI MANAGER — INTERFACE
   ------------------------------------------------------------
   @file      wifi_manager.h
   @brief     Gestão da ligação Wi-Fi STA (DHCP)
   @version   1.4
   @date      2026-03-19

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere a ligação Wi-Fi em modo STA com DHCP. Regista callbacks
   de evento para ligar, desligar e obter IP. Notifica o
   display_manager através de display_manager_set_wifi() sempre
   que o estado muda — sem acoplamento inverso.

   Dependências:
   -------------
   - system_config.h    : WIFI_SSID, WIFI_PASS, WIFI_RETRY_ATTEMPTS
   - display_manager    : display_manager_set_wifi() (notificação)
   - esp_wifi           : stack Wi-Fi ESP-IDF
   - esp_event          : sistema de eventos
   - esp_netif          : abstracção de rede

   
============================================================ */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

/**
 * @brief Inicializa o Wi-Fi em modo STA com DHCP.
 *        Regista handlers de evento e inicia a ligação.
 *        Deve ser chamada uma única vez em app_main(),
 *        após nvs_flash_init() e esp_netif_init().
 */
void wifi_manager_init(void);

/**
 * @brief Verifica se o Wi-Fi está actualmente ligado com IP.
 * @return true se ligado, false caso contrário
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Retorna o endereço IP actual como string.
 * @return String "x.x.x.x" ou "---" se não conectado.
 *         O ponteiro é válido durante toda a vida do programa.
 */
const char *wifi_manager_get_ip(void);

/**
 * @brief Repõe o contador de tentativas e força nova ligação.
 *        Útil após WIFI_RETRY_ATTEMPTS esgotadas sem sucesso.
 */
void wifi_manager_reset_retry(void);

#endif /* WIFI_MANAGER_H */