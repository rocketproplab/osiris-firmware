idf_component_register(
    SRCS "bmp388_sensor.c" "bmp3.c"
    INCLUDE_DIRS "."
    REQUIRES flight_mode event_log sensor_health data_router sensor_records sensor_bus driver esp_timer
)
