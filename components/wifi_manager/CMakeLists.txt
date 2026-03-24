# ============================================================
# CMAKE COMPONENT REGISTER: wifi_manager
# ------------------------------------------------------------
# CORRECÇÃO v1.4 → v1.5:
#   Removido "display_manager" do REQUIRES.
#   O wifi_manager chamava display_manager_set_wifi() directamente
#   no handler de eventos, criando uma dependência circular no
#   grafo de build do ESP-IDF:
#     wifi_manager → display_manager → (st7789, post_config, lvgl)
#
#   Solução: a notificação do display é feita pelo main.c que já
#   depende de ambos os módulos e chama set_wifi() no seu ciclo
#   de 500ms ao detectar mudança de estado via wifi_manager_is_connected().
#   As chamadas display_manager_set_wifi() no wifi_event_handler
#   foram removidas do wifi_manager.c (ver v1.5).
# ============================================================

idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi        # stack Wi-Fi ESP-IDF
        esp_event       # sistema de eventos
        esp_netif       # abstracção de rede
        freertos        # FreeRTOS
        log             # ESP_LOGI
        config          # system_config.h (WIFI_SSID, WIFI_PASS, etc.)
)