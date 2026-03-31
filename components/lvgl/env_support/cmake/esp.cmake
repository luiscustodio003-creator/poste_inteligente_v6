file(GLOB_RECURSE SOURCES ${LVGL_ROOT_DIR}/src/*.c)

idf_build_get_property(LV_MICROPYTHON LV_MICROPYTHON)

if(LV_MICROPYTHON)
  idf_component_register(
    SRCS
      ${SOURCES}
    INCLUDE_DIRS
      ${LVGL_ROOT_DIR}
      ${LVGL_ROOT_DIR}/src
      ${LVGL_ROOT_DIR}/../
    REQUIRES
      main)
else()

  # ============================================================
  # EXEMPLOS (DESATIVADOS PARA PROJETO FINAL)
  # ============================================================
  if(CONFIG_LV_BUILD_EXAMPLES AND EXISTS ${LVGL_ROOT_DIR}/examples)
    file(GLOB_RECURSE EXAMPLE_SOURCES ${LVGL_ROOT_DIR}/examples/*.c)
    set_source_files_properties(${EXAMPLE_SOURCES}
      COMPILE_FLAGS "-Wno-unused-variable -Wno-format")
  endif()

  # ============================================================
  # DEMOS (DESATIVADOS PARA PROJETO FINAL)
  # ============================================================
  if(CONFIG_LV_USE_DEMO_WIDGETS AND EXISTS ${LVGL_ROOT_DIR}/demos/widgets)
    file(GLOB_RECURSE DEMO_WIDGETS_SOURCES ${LVGL_ROOT_DIR}/demos/widgets/*.c)
    list(APPEND DEMO_SOURCES ${DEMO_WIDGETS_SOURCES})
  endif()

  if(CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER AND EXISTS ${LVGL_ROOT_DIR}/demos/keypad_encoder)
    file(GLOB_RECURSE DEMO_KEYPAD_AND_ENCODER_SOURCES ${LVGL_ROOT_DIR}/demos/keypad_encoder/*.c)
    list(APPEND DEMO_SOURCES ${DEMO_KEYPAD_AND_ENCODER_SOURCES})
  endif()

  if(CONFIG_LV_USE_DEMO_BENCHMARK AND EXISTS ${LVGL_ROOT_DIR}/demos/benchmark)
    file(GLOB_RECURSE DEMO_BENCHMARK_SOURCES ${LVGL_ROOT_DIR}/demos/benchmark/*.c)
    list(APPEND DEMO_SOURCES ${DEMO_BENCHMARK_SOURCES})
  endif()

  if(CONFIG_LV_USE_DEMO_STRESS AND EXISTS ${LVGL_ROOT_DIR}/demos/stress)
    file(GLOB_RECURSE DEMO_STRESS_SOURCES ${LVGL_ROOT_DIR}/demos/stress/*.c)
    list(APPEND DEMO_SOURCES ${DEMO_STRESS_SOURCES})
  endif()

  if(CONFIG_LV_USE_DEMO_MUSIC AND EXISTS ${LVGL_ROOT_DIR}/demos/music)
    file(GLOB_RECURSE DEMO_MUSIC_SOURCES ${LVGL_ROOT_DIR}/demos/music/*.c)
    list(APPEND DEMO_SOURCES ${DEMO_MUSIC_SOURCES})
    set_source_files_properties(${DEMO_MUSIC_SOURCES}
      COMPILE_FLAGS "-Wno-format")
  endif()

  # ============================================================
  # REGISTO DO COMPONENTE (SEM EXAMPLES / DEMOS)
  # ============================================================
  idf_component_register(
    SRCS
      ${SOURCES}
      ${EXAMPLE_SOURCES}
      ${DEMO_SOURCES}
    INCLUDE_DIRS
      ${LVGL_ROOT_DIR}
      ${LVGL_ROOT_DIR}/src
      ${LVGL_ROOT_DIR}/../
    REQUIRES
      esp_timer)

endif()

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLV_CONF_INCLUDE_SIMPLE")

if(CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM)
  target_compile_definitions(${COMPONENT_LIB}
    PUBLIC "-DLV_ATTRIBUTE_FAST_MEM=IRAM_ATTR")
endif()