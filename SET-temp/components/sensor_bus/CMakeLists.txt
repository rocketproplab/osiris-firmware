idf_component_register(SRCS "sensor_bus.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_gpio esp_driver_spi esp_driver_i2c)
