idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash sensor_bus bmp388_sensor lsm6dsox_sensor gps_sensor sd_logger telemetry flight_logic config_receiver data_router ring_buffer discrete_in event_log flight_mode battery_monitor driver esp_timer esp_rom
)
