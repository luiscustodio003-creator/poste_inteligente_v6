idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "."
    REQUIRES
        dali_manager    # dali_set_brightness, dali_safe_mode
        comm_manager    # comm_send_*, comm_*_online, comm_is_master
        radar_manager   # radar_read_data, radar_vehicle_in_range
        esp_timer       # esp_timer_get_time (timestamps)
        freertos        # FreeRTOS
        log             # ESP_LOGI / ESP_LOGW
        config          # system_config.h:
                        #   TRAFIC_TIMEOUT_MS, LIGHT_MIN/MAX/SAFE_MODE
                        #   USE_RADAR, POST_POSITION
      
)