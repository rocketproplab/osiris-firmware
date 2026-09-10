// spi_probe.c

#include "spi_probe.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ai_lsm6dsox_reg.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "probe";

#define SETTLE_US 500
#define READS_PER_MODE 4

typedef enum {
	WIRE_FLOATING,
	WIRE_PULLED_UP,
	WIRE_PULLED_DOWN,
	WIRE_DRIVEN,
} wire_state_t;

// Leave a pin genuinely floating. gpio_reset_pin() would re-enable the
// internal pull-up, which would corrupt the next pin's reading if the two are
// jumpered together.
static void release_pin(int gpio) {
	const gpio_config_t config = {
		.pin_bit_mask = 1ULL << gpio,
		.mode = GPIO_MODE_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&config);
}

// Fight the pin with an internal pull each way. The ESP's internal pulls are
// around 45k, so any external resistor of a few k wins and gives away what is
// attached on the far end.
static wire_state_t classify_wire(int gpio) {
	gpio_config_t config = {
		.pin_bit_mask = 1ULL << gpio,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&config);
	esp_rom_delay_us(SETTLE_US);
	int with_pull_up = gpio_get_level(gpio);

	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_ENABLE;
	gpio_config(&config);
	esp_rom_delay_us(SETTLE_US);
	int with_pull_down = gpio_get_level(gpio);

	release_pin(gpio);

	if (with_pull_up == 1 && with_pull_down == 0) {
		return WIRE_FLOATING;
	}
	if (with_pull_up == 1 && with_pull_down == 1) {
		return WIRE_PULLED_UP;
	}
	if (with_pull_up == 0 && with_pull_down == 0) {
		return WIRE_PULLED_DOWN;
	}
	return WIRE_DRIVEN;
}

// `expected` is what a correctly landed wire should look like given the
// breakout's own resistors.
static void probe_wire(const char *name, int gpio, wire_state_t expected, const char *because) {
	wire_state_t actual = classify_wire(gpio);

	static const char *const NAMES[] = {
		[WIRE_FLOATING] = "floating",
		[WIRE_PULLED_UP] = "pulled HIGH",
		[WIRE_PULLED_DOWN] = "pulled LOW",
		[WIRE_DRIVEN] = "actively driven",
	};

	if (actual == expected) {
		ESP_LOGI(TAG, "  %-4s GPIO%-2d  %-15s as expected (%s)", name, gpio, NAMES[actual], because);
	} else if (actual == WIRE_FLOATING) {
		ESP_LOGE(TAG, "  %-4s GPIO%-2d  FLOATING, expected %s - this wire is NOT reaching"
		              " the breakout pad you think it is", name, gpio, NAMES[expected]);
	} else {
		ESP_LOGW(TAG, "  %-4s GPIO%-2d  %s, expected %s", name, gpio, NAMES[actual], NAMES[expected]);
	}
}

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
		ESP_LOGE(TAG, "  %-4s GPIO%-2d  STUCK (reads %d driven high, %d driven low)",
		         name, gpio, high, low);
	}

	release_pin(gpio);
}

// Hold CS low by hand and watch SDO. In I2C mode the pad is an input sitting
// on the breakout's address pull-down; once the part enters SPI mode it takes
// the pad over as an output, so the signature should stop looking like a
// pull-down. If nothing changes, CS is not reaching the chip.
static void probe_cs_wakes_sdo(int cs_gpio, int miso_gpio) {
	wire_state_t idle = classify_wire(miso_gpio);

	const gpio_config_t config = {
		.pin_bit_mask = 1ULL << cs_gpio,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&config);
	gpio_set_level(cs_gpio, 0);
	esp_rom_delay_us(SETTLE_US * 4);

	wire_state_t selected = classify_wire(miso_gpio);

	gpio_set_level(cs_gpio, 1);
	release_pin(cs_gpio);

	ESP_LOGI(TAG, "  SDO with CS high: %s", idle == WIRE_PULLED_DOWN ? "pulled LOW (I2C mode, as expected)"
	                                       : idle == WIRE_FLOATING ? "floating" : "other");
	if (selected == idle) {
		ESP_LOGE(TAG, "  SDO with CS low : UNCHANGED - the part is not entering SPI mode."
		              " CS is not reaching the chip, or the chip is not alive.");
	} else {
		ESP_LOGI(TAG, "  SDO with CS low : CHANGED - the part does respond to CS");
	}
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

void spi_probe_run(const ai_spi_bus_config_t *bus_config,
                   const ai_spi_device_config_t *device_config) {
	ESP_LOGI(TAG, "==== SPI bring-up probe ====");

	// Expectations below assume an Adafruit-style breakout: I2C pull-ups on
	// SCL and SDA, a pull-up on CS to default the part into I2C, and a
	// pull-down on SDO/SA0 to select the low I2C address. Check the schematic
	// for your board if these look wrong.
	ESP_LOGI(TAG, "1. What is on the far end of each wire");
	probe_wire("SCLK", bus_config->sclk_gpio, WIRE_PULLED_UP, "breakout I2C pull-up");
	probe_wire("MOSI", bus_config->mosi_gpio, WIRE_PULLED_UP, "breakout I2C pull-up");
	probe_wire("CS", device_config->cs_gpio, WIRE_PULLED_UP, "breakout CS pull-up");
	probe_wire("MISO", bus_config->miso_gpio, WIRE_PULLED_DOWN, "breakout SA0 pull-down");

	ESP_LOGI(TAG, "2. Does the part respond to CS at all");
	probe_cs_wakes_sdo(device_config->cs_gpio, bus_config->miso_gpio);

	ESP_LOGI(TAG, "3. ESP-side pin drive check");
	probe_output_pin("SCLK", bus_config->sclk_gpio);
	probe_output_pin("MOSI", bus_config->mosi_gpio);
	probe_output_pin("CS", device_config->cs_gpio);

	ESP_LOGI(TAG, "4. WHO_AM_I in each SPI mode (expecting 0x%02X)", AI_LSM6DSOX_WHO_AM_I_VALUE);
	esp_err_t err = ai_spi_bus_init(bus_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "  bus init failed: %s", esp_err_to_name(err));
		return;
	}
	for (uint8_t mode = 0; mode < 4; mode++) {
		probe_mode(bus_config->host, device_config, mode);
	}
	ai_spi_bus_deinit(bus_config->host);

	ESP_LOGI(TAG, "==== probe done ====");
}
