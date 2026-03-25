idf_component_register(
    SRCS "display_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_driver  # st7789 — driver SPI do display
        radar_manager   # radar_obj_t — ADICIONADO v5.1
        lvgl            # biblioteca LVGL v8.3.x
        freertos        # FreeRTOS tasks e mutexes
        esp_timer       # esp_timer_get_time
        log             # ESP_LOGI / ESP_LOGW / ESP_LOGE
        config          # system_config.h, hw_config.h
)
 
set(LV_CONF_PATH "${CMAKE_SOURCE_DIR}/components/lvgl/lv_conf.h")
 
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    "LV_CONF_INCLUDE_SIMPLE"
    "LV_CONF_SUPPRESS_DEFINE_CHECK"
    "LV_CONF_PATH=${LV_CONF_PATH}"
)
 
target_include_directories(${COMPONENT_LIB} PUBLIC "../lvgl")