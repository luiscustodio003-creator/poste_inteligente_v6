cmake_minimum_required(VERSION 3.16)

# Define o caminho absoluto para o teu lv_conf.h
set(LV_CONF_PATH "${CMAKE_SOURCE_DIR}/components/lvgl/lv_conf.h")

# Exporta estas definições para TODOS os componentes
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE)
add_compile_definitions(LV_CONF_PATH=${LV_CONF_PATH})
add_compile_definitions(LV_CONF_SUPPRESS_DEFINE_CHECK)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(poste_inteligente_v6)