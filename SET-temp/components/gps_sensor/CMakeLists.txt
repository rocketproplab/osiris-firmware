idf_component_register(
    SRCS "gps_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES data_router sensor_records sensor_health event_log driver esp_timer
)
