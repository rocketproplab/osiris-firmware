# nvs_flash -> NVS-backed deployment config
# json       -> cJSON, for parsing that config
idf_component_register(
    SRCS "flight_logic.c"
    INCLUDE_DIRS "."
    REQUIRES flight_mode event_log bmp388_sensor lsm6dsox_sensor freertos nvs_flash
)
