idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_manager # notifica display quando estado Wi-Fi muda
        esp_wifi        # stack Wi-Fi ESP-IDF
        esp_event       # sistema de eventos
        esp_netif       # abstracção de rede
        freertos        # FreeRTOS
        log             # ESP_LOGI
        config          # system_config.h (WIFI_SSID, WIFI_PASS, etc.)
)