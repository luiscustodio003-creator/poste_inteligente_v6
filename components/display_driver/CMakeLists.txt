idf_component_register(
    SRCS "st7789.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver      # spi_master, gpio
        freertos    # vTaskDelay
        log         # ESP_LOGI
        config      # hw_config.h (pinos GPIO)
)