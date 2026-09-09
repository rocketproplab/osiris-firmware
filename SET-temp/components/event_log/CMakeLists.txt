idf_component_register(
    SRCS "event_log.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_records esp_timer
)
