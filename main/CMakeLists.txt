idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        config          # system_config.h, post_config.h
        display_manager # display_manager_*
        wifi_manager    # wifi_manager_*
        comm_manager    # comm_init, comm_*
        udp_manager     # udp_manager_get_neighbors
        ;dali_manager    # dali_init, dali_get_brightness
        ;radar_manager   # radar_init
        state_machine   # state_machine_*, sm_inject_test_car
        nvs_flash       # nvs_flash_init
        freertos        # xTaskCreate, vTaskDelay
        esp_timer       # esp_timer_get_time (teste simulado)
        log             # ESP_LOGI
)