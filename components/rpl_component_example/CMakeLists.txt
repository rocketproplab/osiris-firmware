idf_component_register(
	SRCS "rpl_example_component.c" "rpl_example_component_extra.c"
	INCLUDE_DIRS "include"
	PRIV_REQUIRES esp_driver_gpio esp_driver_uart
)
