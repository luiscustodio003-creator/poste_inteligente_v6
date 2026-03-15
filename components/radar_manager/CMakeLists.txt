idf_component_register(
    SRCS "radar_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        log
        config
)