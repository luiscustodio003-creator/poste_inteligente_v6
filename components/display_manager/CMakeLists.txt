# ============================================================
# CMAKE COMPONENT REGISTER: display_manager
# ============================================================

idf_component_register(
    SRCS "display_manager.c"
    INCLUDE_DIRS "."
    REQUIRES 
        display_driver  # st7789
        lvgl            # UI Library
        freertos 
        esp_timer 
        log 
        config
)

# Definir o caminho para o ficheiro de configuração do LVGL
set(LV_CONF_PATH "${CMAKE_SOURCE_DIR}/components/lvgl/lv_conf.h")

# Injetar as definições de compilação apenas aqui para evitar conflitos globais
target_compile_definitions(${COMPONENT_LIB} PUBLIC 
    "LV_CONF_INCLUDE_SIMPLE"
    "LV_CONF_SUPPRESS_DEFINE_CHECK"
    "LV_CONF_PATH=${LV_CONF_PATH}"
)

# Garantir que o diretório do lvgl está nos caminhos de busca
target_include_directories(${COMPONENT_LIB} PUBLIC "../lvgl")