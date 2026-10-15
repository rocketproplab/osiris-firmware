idf_component_register(
    SRCS "adc1_shared.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc freertos
)
