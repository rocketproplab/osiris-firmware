idf_component_register(
    SRCS "lsm6dsox_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES flight_mode event_log sensor_health data_router sensor_records sensor_bus driver esp_timer
)
