idf_component_register(
    SRCS "gps_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES data_router sensor_records driver esp_timer
)
