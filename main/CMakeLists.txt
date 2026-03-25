idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        config
        display_manager
        wifi_manager
        comm_manager
        udp_manager
        dali_manager
        radar_manager
        state_machine      
        nvs_flash
        freertos
        esp_timer
        esp_netif
        esp_event
        log
)