idf_component_register(
    SRCS "display_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_driver
        radar_manager
        state_machine
        lvgl
        freertos
        esp_timer
        log
        config
)

set(LV_CONF_PATH "${CMAKE_SOURCE_DIR}/components/lvgl/lv_conf.h")

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    LV_CONF_INCLUDE_SIMPLE
    LV_CONF_SUPPRESS_DEFINE_CHECK
    "LV_CONF_PATH=${LV_CONF_PATH}"
)

target_include_directories(${COMPONENT_LIB} PUBLIC "../lvgl")