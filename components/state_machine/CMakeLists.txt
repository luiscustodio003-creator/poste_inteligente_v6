idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "."
    REQUIRES
        dali_manager
        comm_manager
        radar_manager
        config
        esp_timer
        log
)