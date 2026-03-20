idf_component_register(
    SRCS "display_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_driver  # st7789 — driver SPI do display físico
        lvgl            # biblioteca de UI v8.3.x
        freertos        # vTaskDelay, FreeRTOS
        esp_timer       # timers de sistema
        log             # ESP_LOGI / ESP_LOGW
        config          # system_config.h (LCD_H_RES, LCD_V_RES, POSTE_NAME, LIGHT_MIN)
                        # hw_config.h     (LCD_PIN_*)
)
 