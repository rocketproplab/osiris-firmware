idf_component_register(
	SRCS "rpl_xbee.c"
	INCLUDE_DIRS "include"
	PRIV_REQUIRES esp_driver_gpio esp_driver_uart
)
