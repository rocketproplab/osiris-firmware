// ai_spi.c

#include "ai_spi.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAG = "ai_spi";

struct ai_spi_device_s {
	spi_device_handle_t handle;
	spi_host_device_t host;
	uint8_t read_flag;
};

// One reference count per host. Init/deinit are expected to run from a single
// task during start-up, so this is deliberately lock-free; transfers
// themselves are made thread safe by the underlying driver.
static uint8_t s_bus_refs[SOC_SPI_PERIPH_NUM];

static bool host_in_range(spi_host_device_t host) {
	return (int)host >= 0 && (int)host < SOC_SPI_PERIPH_NUM;
}

esp_err_t ai_spi_bus_init(const ai_spi_bus_config_t *config) {
	if (config == NULL || !host_in_range(config->host)) {
		return ESP_ERR_INVALID_ARG;
	}

	if (s_bus_refs[config->host] > 0) {
		s_bus_refs[config->host]++;
		ESP_LOGD(TAG, "host %d already up (%u refs)", (int)config->host, s_bus_refs[config->host]);
		return ESP_OK;
	}

	const spi_bus_config_t bus_config = {
		.sclk_io_num = config->sclk_gpio,
		.mosi_io_num = config->mosi_gpio,
		.miso_io_num = config->miso_gpio,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		// Without DMA a transfer is limited to the 64-byte FIFO, which is all
		// the register helpers need and keeps buffers free of DMA constraints.
		.max_transfer_sz = 0,
	};

	esp_err_t err = spi_bus_initialize(config->host, &bus_config, SPI_DMA_DISABLED);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "spi_bus_initialize(host %d) failed: %s", (int)config->host, esp_err_to_name(err));
		return err;
	}

	s_bus_refs[config->host] = 1;
	ESP_LOGI(TAG, "host %d up: sclk=%d mosi=%d miso=%d",
	         (int)config->host, config->sclk_gpio, config->mosi_gpio, config->miso_gpio);
	return ESP_OK;
}

esp_err_t ai_spi_bus_deinit(spi_host_device_t host) {
	if (!host_in_range(host)) {
		return ESP_ERR_INVALID_ARG;
	}
	if (s_bus_refs[host] == 0) {
		return ESP_ERR_INVALID_STATE;
	}

	if (--s_bus_refs[host] > 0) {
		return ESP_OK;
	}

	esp_err_t err = spi_bus_free(host);
	if (err != ESP_OK) {
		// Put the reference back so the caller can retry once the remaining
		// devices have been removed.
		s_bus_refs[host] = 1;
		ESP_LOGE(TAG, "spi_bus_free(host %d) failed: %s", (int)host, esp_err_to_name(err));
	}
	return err;
}

esp_err_t ai_spi_device_add(spi_host_device_t host,
                            const ai_spi_device_config_t *config,
                            ai_spi_device_handle_t *out_handle) {
	if (config == NULL || out_handle == NULL || !host_in_range(host)) {
		return ESP_ERR_INVALID_ARG;
	}
	if (config->mode > 3) {
		return ESP_ERR_INVALID_ARG;
	}
	if (s_bus_refs[host] == 0) {
		ESP_LOGE(TAG, "host %d not initialised", (int)host);
		return ESP_ERR_INVALID_STATE;
	}

	struct ai_spi_device_s *device = calloc(1, sizeof(*device));
	if (device == NULL) {
		return ESP_ERR_NO_MEM;
	}

	const spi_device_interface_config_t device_config = {
		.spics_io_num = config->cs_gpio,
		.clock_speed_hz = config->clock_speed_hz,
		.mode = config->mode,
		.cs_ena_pretrans = config->cs_ena_pretrans,
		.cs_ena_posttrans = config->cs_ena_posttrans,
		.queue_size = (config->queue_size > 0) ? config->queue_size : 1,
	};

	esp_err_t err = spi_bus_add_device(host, &device_config, &device->handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "spi_bus_add_device(cs=%d) failed: %s", config->cs_gpio, esp_err_to_name(err));
		free(device);
		return err;
	}

	device->host = host;
	device->read_flag = (config->reg_read_flag != 0) ? config->reg_read_flag : AI_SPI_READ_FLAG_MSB;

	*out_handle = device;
	ESP_LOGI(TAG, "device on host %d: cs=%d mode=%u %d Hz",
	         (int)host, config->cs_gpio, config->mode, config->clock_speed_hz);
	return ESP_OK;
}

esp_err_t ai_spi_device_remove(ai_spi_device_handle_t device) {
	if (device == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = spi_bus_remove_device(device->handle);
	if (err == ESP_OK) {
		free(device);
	}
	return err;
}

esp_err_t ai_spi_transfer(ai_spi_device_handle_t device,
                          const uint8_t *tx, uint8_t *rx, size_t len) {
	if (device == NULL || len == 0 || (tx == NULL && rx == NULL)) {
		return ESP_ERR_INVALID_ARG;
	}
	if (len > AI_SPI_MAX_REG_LEN + 1) {
		return ESP_ERR_INVALID_SIZE;
	}

	spi_transaction_t transaction = {
		.length = len * 8,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};

	return spi_device_polling_transmit(device->handle, &transaction);
}

esp_err_t ai_spi_read_regs(ai_spi_device_handle_t device, uint8_t reg, uint8_t *data, size_t len) {
	if (device == NULL || data == NULL || len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	if (len > AI_SPI_MAX_REG_LEN) {
		return ESP_ERR_INVALID_SIZE;
	}

	uint8_t tx[1 + AI_SPI_MAX_REG_LEN] = {0};
	uint8_t rx[1 + AI_SPI_MAX_REG_LEN] = {0};
	tx[0] = (uint8_t)(reg | device->read_flag);

	spi_transaction_t transaction = {
		.length = (1 + len) * 8,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};

	esp_err_t err = spi_device_polling_transmit(device->handle, &transaction);
	if (err != ESP_OK) {
		return err;
	}

	memcpy(data, &rx[1], len);
	return ESP_OK;
}

esp_err_t ai_spi_write_regs(ai_spi_device_handle_t device, uint8_t reg, const uint8_t *data, size_t len) {
	if (device == NULL || data == NULL || len == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	if (len > AI_SPI_MAX_REG_LEN) {
		return ESP_ERR_INVALID_SIZE;
	}

	uint8_t tx[1 + AI_SPI_MAX_REG_LEN] = {0};
	tx[0] = reg;
	memcpy(&tx[1], data, len);

	spi_transaction_t transaction = {
		.length = (1 + len) * 8,
		.tx_buffer = tx,
		.rx_buffer = NULL,
	};

	return spi_device_polling_transmit(device->handle, &transaction);
}

esp_err_t ai_spi_read_reg(ai_spi_device_handle_t device, uint8_t reg, uint8_t *value) {
	return ai_spi_read_regs(device, reg, value, 1);
}

esp_err_t ai_spi_write_reg(ai_spi_device_handle_t device, uint8_t reg, uint8_t value) {
	return ai_spi_write_regs(device, reg, &value, 1);
}
