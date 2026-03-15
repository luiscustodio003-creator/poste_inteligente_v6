idf_component_register(
    SRCS "udp_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        wifi_manager
        config
        lwip
        nvs_flash
        esp_timer
        log
)