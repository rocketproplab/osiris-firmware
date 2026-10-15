idf_component_register(
    SRCS "sensor_health.c"
    INCLUDE_DIRS "."
    REQUIRES event_log sensor_records esp_timer
)
