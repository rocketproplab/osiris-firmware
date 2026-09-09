idf_component_register(
    SRCS "discrete_in.c"
    INCLUDE_DIRS "."
    REQUIRES event_log sensor_records driver esp_timer esp_driver_gpio
)
