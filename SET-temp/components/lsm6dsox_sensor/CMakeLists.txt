idf_component_register(
    SRCS "lsm6dsox_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES flight_mode event_log data_router sensor_records driver esp_timer
)
