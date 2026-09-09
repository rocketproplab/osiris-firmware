idf_component_register(
    SRCS "flight_mode.c"
    INCLUDE_DIRS "."
    REQUIRES event_log sensor_records esp_timer
)
