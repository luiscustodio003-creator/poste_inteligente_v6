idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_manager
        wifi_manager
        comm_manager
        udp_manager
        config
        dali_manager
        radar_manager
        state_machine
        nvs_flash
        freertos
        log
)