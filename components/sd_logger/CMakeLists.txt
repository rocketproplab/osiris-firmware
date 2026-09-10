# fatfs           -> esp_vfs_fat.h (mount, FILE* on the card)
# sdmmc           -> sdmmc_cmd.h   (sdmmc_card_t, card info)
# esp_driver_sdmmc-> driver/sdmmc_host.h AND
#                    sd_pwr_ctrl_by_on_chip_ldo.h (the ESP32-P4 IO rail)
# vfs             -> the VFS layer fopen/fwrite go through
idf_component_register(
    SRCS "sd_logger.c"
    INCLUDE_DIRS "."
    REQUIRES event_log sensor_records driver esp_timer fatfs sdmmc esp_driver_sdmmc vfs ring_buffer
)
