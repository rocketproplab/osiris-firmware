idf_component_register(
    SRCS "gps_config.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart
)