// spi_probe.c

#include "spi_probe.h"

#include "ai_lsm6dsox_reg.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "probe";

#define SETTLE_US 200
#define READS_PER_MODE 4

// Drive a pin both ways and read it back. A pin that will not follow its own
// output is being held by something else on the board or the breadboard.
static void probe_output_pin(const char *name, int gpio) {
	const gpio_config_t config = {
		.pin_bit_mask = 1ULL << gpio,
		.mode = GPIO_MODE_INPUT_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	if (gpio_config(&config) != ESP_OK) {
		ESP_LOGE(TAG, "  %-4s GPIO%-2d  cannot be configured as an output", name, gpio);
		return;
	}

	gpio_set_level(gpio, 1);
	esp_rom_delay_us(SETTLE_US);
	int high = gpio_get_level(gpio);

	gpio_set_level(gpio, 0);
	esp_rom_delay_us(SETTLE_US);
	int low = gpio_get_level(gpio);

	if (high == 1 && low == 0) {
		ESP_LOGI(TAG, "  %-4s GPIO%-2d  drives high and low: OK", name, gpio);
	} else {
		ESP_LOGE(TAG, "  %-4s GPIO%-2d  STUCK (reads %d when driven high, %d when driven low)"
		              " - something else is holding this pin", name, gpio, high, low);
	}

	gpio_reset_pin(gpio);
}

// Decide whether the far end is driving MISO. An internal pull-up and then a
// pull-down that both win means the line is floating: the sensor is not
// driving SDO.
static void probe_input_pin(const char *name, int gpio) {
	gpio_config_t config = {
		.pin_bit_mask = 1ULL << gpio,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	if (gpio_config(&config) != ESP_OK) {
		ESP_LOGE(TAG, "  %-4s GPIO%-2d  cannot be configured as an input", name, gpio);
		return;
	}
	esp_rom_delay_us(SETTLE_US);
	int pulled_up = gpio_get_level(gpio);

	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_ENABLE;
	gpio_config(&config);
	esp_rom_delay_us(SETTLE_US);
	int pulled_down = gpio_get_level(gpio);

	if (pulled_up == 1 && pulled_down == 0) {
		ESP_LOGW(TAG, "  %-4s GPIO%-2d  FLOATING - nothing on the other end is driving it."
		              " Idle SDO is expected to float, so this alone is not a fault,"
		              " but a broken DO wire looks exactly like this.", name, gpio);
	} else if (pulled_up == 0 && pulled_down == 0) {
		ESP_LOGW(TAG, "  %-4s GPIO%-2d  held LOW by the other end", name, gpio);
	} else if (pulled_up == 1 && pulled_down == 1) {
		ESP_LOGI(TAG, "  %-4s GPIO%-2d  held HIGH by the other end", name, gpio);
	}

	gpio_reset_pin(gpio);
}

static void probe_mode(spi_host_device_t host,
                       const ai_spi_device_config_t *base_config,
                       uint8_t mode) {
	ai_spi_device_config_t config = *base_config;
	config.mode = mode;

	ai_spi_device_handle_t device = NULL;
	esp_err_t err = ai_spi_device_add(host, &config, &device);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "  mode %u: could not attach device: %s", mode, esp_err_to_name(err));
		return;
	}

	char results[READS_PER_MODE * 6 + 1];
	int offset = 0;
	int matches = 0;

	for (int i = 0; i < READS_PER_MODE; i++) {
		uint8_t value = 0;
		err = ai_spi_read_reg(device, AI_LSM6DSOX_REG_WHO_AM_I, &value);
		if (err != ESP_OK) {
			offset += snprintf(&results[offset], sizeof(results) - offset, " err");
			continue;
		}
		offset += snprintf(&results[offset], sizeof(results) - offset, " 0x%02X", value);
		if (value == AI_LSM6DSOX_WHO_AM_I_VALUE) {
			matches++;
		}
	}

	if (matches == READS_PER_MODE) {
		ESP_LOGI(TAG, "  mode %u: WHO_AM_I%s  <-- THIS MODE WORKS", mode, results);
	} else if (matches > 0) {
		ESP_LOGW(TAG, "  mode %u: WHO_AM_I%s  (intermittent - signal integrity)", mode, results);
	} else {
		ESP_LOGI(TAG, "  mode %u: WHO_AM_I%s", mode, results);
	}

	ai_spi_device_remove(device);
}


// Clocks a pattern out and reads what comes back. Only meaningful with MOSI
// jumpered straight to MISO and the sensor's wires off: an exact echo proves
// the SPI peripheral, both pins and the driver path are all good, which
// narrows any remaining fault down to the sensor or its wiring.
static void probe_loopback(spi_host_device_t host, const ai_spi_device_config_t *base_config) {
	static const uint8_t pattern[] = {0xA5, 0x3C, 0xFF, 0x5A, 0x0F};

	ai_spi_device_handle_t device = NULL;
	esp_err_t err = ai_spi_device_add(host, base_config, &device);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "  could not attach device: %s", esp_err_to_name(err));
		return;
	}

	uint8_t received[sizeof(pattern)] = {0};
	err = ai_spi_transfer(device, pattern, received, sizeof(pattern));
	ai_spi_device_remove(device);

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "  transfer failed: %s", esp_err_to_name(err));
		return;
	}

	ESP_LOGI(TAG, "  sent     0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
	         pattern[0], pattern[1], pattern[2], pattern[3], pattern[4]);
	ESP_LOGI(TAG, "  received 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
	         received[0], received[1], received[2], received[3], received[4]);

	if (memcmp(pattern, received, sizeof(pattern)) == 0) {
		ESP_LOGI(TAG, "  exact echo: SPI peripheral, MOSI, MISO and driver path all good");
	} else {
		bool all_same = true;
		for (size_t i = 1; i < sizeof(received); i++) {
			if (received[i] != received[0]) {
				all_same = false;
			}
		}
		if (all_same) {
			ESP_LOGI(TAG, "  flat 0x%02X: no jumper fitted, or MISO is stuck", received[0]);
		} else {
			ESP_LOGW(TAG, "  garbled echo: jumper is in but the data is corrupt");
		}
	}
}

void spi_probe_run(const ai_spi_bus_config_t *bus_config,
                   const ai_spi_device_config_t *device_config) {
	ESP_LOGI(TAG, "==== SPI bring-up probe ====");

	ESP_LOGI(TAG, "1. ESP-side pin check (nothing driven by the sensor yet)");
	probe_output_pin("SCLK", bus_config->sclk_gpio);
	probe_output_pin("MOSI", bus_config->mosi_gpio);
	probe_output_pin("CS", device_config->cs_gpio);
	probe_input_pin("MISO", bus_config->miso_gpio);

	ESP_LOGI(TAG, "2. WHO_AM_I in each SPI mode (expecting 0x%02X)", AI_LSM6DSOX_WHO_AM_I_VALUE);
	esp_err_t err = ai_spi_bus_init(bus_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "  bus init failed: %s", esp_err_to_name(err));
		return;
	}
	for (uint8_t mode = 0; mode < 4; mode++) {
		probe_mode(bus_config->host, device_config, mode);
	}

	ESP_LOGI(TAG, "3. Echo check (only meaningful with MOSI jumpered to MISO)");
	probe_loopback(bus_config->host, device_config);

	ai_spi_bus_deinit(bus_config->host);

	ESP_LOGI(TAG, "==== probe done ====");
}
