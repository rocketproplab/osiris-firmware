idf_component_register(
    SRCS "battery_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES flight_mode event_log data_router sensor_records esp_adc esp_timer
)
