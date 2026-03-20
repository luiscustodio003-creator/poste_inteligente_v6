idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        config          # system_config.h, post_config.h
        display_manager # display_manager_init, set_wifi, set_neighbors
        wifi_manager    # wifi_manager_init, is_connected, get_ip
        udp_manager     # udp_manager_init, get_neighbors
        nvs_flash       # nvs_flash_init, nvs_flash_erase
        freertos        # xTaskCreate, vTaskDelay
        log             # ESP_LOGI, ESP_LOGW
)