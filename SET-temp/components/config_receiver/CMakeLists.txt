idf_component_register(
    SRCS "config_receiver.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_records event_log nvs_flash
)
