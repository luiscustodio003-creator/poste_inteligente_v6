idf_component_register(
    SRCS "display_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_driver
        lvgl
        udp_manager
        freertos
        esp_timer
        log
        config
)