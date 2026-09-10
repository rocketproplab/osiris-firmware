// main.c
//
// Minimal SPI bring-up for an Adafruit LSM6DSOX breakout on a Waveshare
// ESP32-P4-ETH. Resets the part, verifies WHO_AM_I, then streams accelerometer,
// gyroscope and temperature so the link can be confirmed by moving the board.

#include "ai_lsm6dsox.h"
#include "ai_spi.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor-spi";

// Board wiring. The breakout is silkscreened for I2C; in 4-wire SPI the pins
// take their alternate functions:
//   SCL -> SCLK, SDA -> MOSI (device SDI), DO -> MISO (device SDO)
#define PIN_SCLK 54
#define PIN_MOSI 19
#define PIN_MISO 18
#define PIN_CS   17

// INT1 (I1) and INT2 (I2) are wired but unused: this bring-up polls.
#define PIN_INT1 16
#define PIN_INT2 15

#define SPI_HOST_ID SPI2_HOST

// The LSM6DSOX itself handles 10 MHz, but the breakout carries 10k I2C
// pull-ups on SCL/SDA that soften the edges. Start conservative; raise once
// the link is known good.
#define SPI_CLOCK_HZ (1 * 1000 * 1000)

// The LSM6DSOX samples on the rising edge with the clock idling high.
#define SPI_MODE 3

#define SAMPLE_PERIOD_MS 250

static esp_err_t spi_read_regs(void *ctx, uint8_t reg, uint8_t *data, size_t len) {
	return ai_spi_read_regs((ai_spi_device_handle_t)ctx, reg, data, len);
}

static esp_err_t spi_write_regs(void *ctx, uint8_t reg, const uint8_t *data, size_t len) {
	return ai_spi_write_regs((ai_spi_device_handle_t)ctx, reg, data, len);
}

void app_main(void) {
	const ai_spi_bus_config_t bus_config = {
		.host = SPI_HOST_ID,
		.sclk_gpio = PIN_SCLK,
		.mosi_gpio = PIN_MOSI,
		.miso_gpio = PIN_MISO,
	};
	ESP_ERROR_CHECK(ai_spi_bus_init(&bus_config));

	const ai_spi_device_config_t device_config = {
		.cs_gpio = PIN_CS,
		.clock_speed_hz = SPI_CLOCK_HZ,
		.mode = SPI_MODE,
		.reg_read_flag = AI_SPI_READ_FLAG_MSB,
	};
	ai_spi_device_handle_t spi_device = NULL;
	ESP_ERROR_CHECK(ai_spi_device_add(SPI_HOST_ID, &device_config, &spi_device));

	const ai_lsm6dsox_transport_t transport = {
		.ctx = spi_device,
		.read_regs = spi_read_regs,
		.write_regs = spi_write_regs,
	};

	static ai_lsm6dsox_t imu;
	const ai_lsm6dsox_config_t imu_config = AI_LSM6DSOX_CONFIG_DEFAULT();

	esp_err_t err = ai_lsm6dsox_init(&imu, &transport, &imu_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "IMU init failed: %s", esp_err_to_name(err));
		ESP_LOGE(TAG, "check SCLK=%d MOSI=%d MISO=%d CS=%d and that the part is powered",
		         PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS);
		return;
	}

	ESP_LOGI(TAG, "LSM6DSOX up on SPI%d, streaming every %d ms",
	         SPI_HOST_ID + 1, SAMPLE_PERIOD_MS);

	while (true) {
		ai_lsm6dsox_data_t data;
		err = ai_lsm6dsox_read_all(&imu, &data);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "read failed: %s", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG,
			         "accel [g] %7.3f %7.3f %7.3f | gyro [dps] %8.2f %8.2f %8.2f | %5.1f C",
			         data.accel.x, data.accel.y, data.accel.z,
			         data.gyro.x, data.gyro.y, data.gyro.z,
			         data.temperature_c);
		}
		vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
	}
}
