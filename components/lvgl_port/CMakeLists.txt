idf_component_register(
    SRCS "lv_port_disp.c"
    INCLUDE_DIRS "."
    REQUIRES lvgl config display_driver log
)