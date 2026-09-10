// ai_spi.h
//
// Thin, board-agnostic wrapper over the ESP-IDF SPI master driver, aimed at
// register-mapped peripherals (sensors, EEPROMs, ...).
//
// The component knows nothing about any particular device: the caller supplies
// the pins, the clock rate and the SPI mode. Bus ownership is reference
// counted, so several devices may sit on the same host and each one may call
// ai_spi_bus_init()/ai_spi_bus_deinit() as if it were the only user.
//
// Register protocol assumed by the read/write helpers:
//   [ address byte ] [ payload ... ]
// where reads OR a flag into the address byte (0x80 on ST and Bosch parts).
// Devices that need a different framing should use the driver directly.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Largest payload a single helper call can move. The bus is opened without
// DMA, so a transfer is bounded by the hardware FIFO (64 bytes) less the
// one address byte.
#define AI_SPI_MAX_REG_LEN 63

// Default read flag: MSB of the address byte set. Matches ST and Bosch parts.
#define AI_SPI_READ_FLAG_MSB 0x80

typedef struct ai_spi_device_s *ai_spi_device_handle_t;

typedef struct {
	spi_host_device_t host;
	int sclk_gpio;
	int mosi_gpio; // device SDI / "SDA" on I2C-labelled breakouts
	int miso_gpio; // device SDO / "DO" on I2C-labelled breakouts
} ai_spi_bus_config_t;

typedef struct {
	int cs_gpio;
	int clock_speed_hz;
	uint8_t mode;             // SPI mode 0-3 (CPOL/CPHA)
	uint8_t reg_read_flag;    // OR'd into the address byte on reads; 0 -> 0x80
	uint8_t cs_ena_pretrans;  // CS setup, in SPI bit times; 0 -> driver default
	uint8_t cs_ena_posttrans; // CS hold, in SPI bit times; 0 -> driver default
	int queue_size;           // pending transactions; 0 -> 1
} ai_spi_device_config_t;

// Bring up a host. Safe to call more than once for the same host: later calls
// only take a reference, and the pin fields are ignored.
esp_err_t ai_spi_bus_init(const ai_spi_bus_config_t *config);

// Drop a reference taken by ai_spi_bus_init(). The host is freed when the last
// reference goes away.
esp_err_t ai_spi_bus_deinit(spi_host_device_t host);

// Attach a device to an already-initialised host.
esp_err_t ai_spi_device_add(spi_host_device_t host,
                            const ai_spi_device_config_t *config,
                            ai_spi_device_handle_t *out_handle);

esp_err_t ai_spi_device_remove(ai_spi_device_handle_t device);

// Register access. `reg` is the bare address; the read flag is applied
// internally. `len` must be 1..AI_SPI_MAX_REG_LEN.
esp_err_t ai_spi_read_regs(ai_spi_device_handle_t device, uint8_t reg, uint8_t *data, size_t len);
esp_err_t ai_spi_write_regs(ai_spi_device_handle_t device, uint8_t reg, const uint8_t *data, size_t len);
esp_err_t ai_spi_read_reg(ai_spi_device_handle_t device, uint8_t reg, uint8_t *value);
esp_err_t ai_spi_write_reg(ai_spi_device_handle_t device, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif
