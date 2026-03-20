idf_component_register(
    SRCS "udp_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        lwip        # sockets UDP
        esp_timer   # esp_timer_get_time
        freertos    # xTaskCreate, vTaskDelay
        log         # ESP_LOGI
        config      # system_config.h (UDP_PORT, MAX_NEIGHBORS, etc.)
)