idf_component_register(
	SRCS "main.c" "spi_probe.c"
	INCLUDE_DIRS "."
	PRIV_REQUIRES ai_spi ai_lsm6dsox esp_driver_gpio
)
