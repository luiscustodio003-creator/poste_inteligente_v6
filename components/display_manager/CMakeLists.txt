idf_component_register(
    SRCS "display_manager.c" "assets/wifi_off.c" "assets/wifi_on.c"
    INCLUDE_DIRS "." "assets"
    REQUIRES
        display_driver
        lvgl
        udp_manager
        freertos
        esp_timer
        log
)