idf_component_register(
    SRCS "udp_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        lwip        # sockets UDP, inet_ntoa_r
        esp_timer   # esp_timer_get_time (timestamps)
        freertos    # xTaskCreate, vTaskDelay
        log         # ESP_LOGI / ESP_LOGW / ESP_LOGE
        config      # system_config.h:
                    #   UDP_PORT, MAX_NEIGHBORS,
                    #   DISCOVER_INTERVAL_MS, NEIGHBOR_TIMEOUT_MS,
                    #   POSTE_ID, POST_POSITION
)