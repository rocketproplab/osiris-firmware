idf_component_register(
    SRCS "telemetry.c" "xbee_api.c" "uplink_stream.c"
    INCLUDE_DIRS "."
    REQUIRES flight_mode event_log sensor_records ring_buffer driver esp_timer esp_driver_uart esp_driver_gpio
)
