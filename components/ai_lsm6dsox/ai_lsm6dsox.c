// ai_lsm6dsox.c

#include "ai_lsm6dsox.h"
#include "ai_lsm6dsox_reg.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ai_lsm6dsox";

// Datasheet gives 10 ms from power-on to a usable interface, and the software
// reset settles well inside that.
#define BOOT_DELAY_MS   10
#define RESET_TIMEOUT_MS 20

// Temperature output is signed, 256 LSB/degC, zeroed at 25 degC.
#define TEMP_LSB_PER_C  256.0f
#define TEMP_OFFSET_C   25.0f

// Sensitivities in g/LSB and dps/LSB, indexed by the raw full-scale encoding.
static const float ACCEL_SENSITIVITY[] = {
	[AI_LSM6DSOX_ACCEL_FS_2G]  = 0.061e-3f,
	[AI_LSM6DSOX_ACCEL_FS_16G] = 0.488e-3f,
	[AI_LSM6DSOX_ACCEL_FS_4G]  = 0.122e-3f,
	[AI_LSM6DSOX_ACCEL_FS_8G]  = 0.244e-3f,
};

static const float GYRO_SENSITIVITY[] = {
	[AI_LSM6DSOX_GYRO_FS_250DPS]  = 8.75e-3f,
	[AI_LSM6DSOX_GYRO_FS_500DPS]  = 17.50e-3f,
	[AI_LSM6DSOX_GYRO_FS_1000DPS] = 35.0e-3f,
	[AI_LSM6DSOX_GYRO_FS_2000DPS] = 70.0e-3f,
};

static esp_err_t read_regs(ai_lsm6dsox_t *device, uint8_t reg, uint8_t *data, size_t len) {
	return device->transport.read_regs(device->transport.ctx, reg, data, len);
}

static esp_err_t write_reg(ai_lsm6dsox_t *device, uint8_t reg, uint8_t value) {
	return device->transport.write_regs(device->transport.ctx, reg, &value, 1);
}

// Little-endian 16-bit output pair -> signed count.
static int16_t decode_i16(const uint8_t *bytes) {
	return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void decode_vec3(const uint8_t *bytes, float sensitivity, ai_lsm6dsox_vec3_t *out) {
	out->x = decode_i16(&bytes[0]) * sensitivity;
	out->y = decode_i16(&bytes[2]) * sensitivity;
	out->z = decode_i16(&bytes[4]) * sensitivity;
}

esp_err_t ai_lsm6dsox_reset(ai_lsm6dsox_t *device) {
	if (device == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = write_reg(device, AI_LSM6DSOX_REG_CTRL3_C, AI_LSM6DSOX_CTRL3_C_SW_RESET);
	if (err != ESP_OK) {
		return err;
	}

	// The part clears SW_RESET itself once the reset completes.
	for (int elapsed_ms = 0; elapsed_ms < RESET_TIMEOUT_MS; elapsed_ms++) {
		vTaskDelay(pdMS_TO_TICKS(1));

		uint8_t ctrl3_c = 0;
		err = read_regs(device, AI_LSM6DSOX_REG_CTRL3_C, &ctrl3_c, 1);
		if (err != ESP_OK) {
			return err;
		}
		if ((ctrl3_c & AI_LSM6DSOX_CTRL3_C_SW_RESET) == 0) {
			return ESP_OK;
		}
	}

	ESP_LOGE(TAG, "software reset did not complete within %d ms", RESET_TIMEOUT_MS);
	return ESP_ERR_TIMEOUT;
}

esp_err_t ai_lsm6dsox_who_am_i(ai_lsm6dsox_t *device, uint8_t *out_id) {
	if (device == NULL || out_id == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	return read_regs(device, AI_LSM6DSOX_REG_WHO_AM_I, out_id, 1);
}

esp_err_t ai_lsm6dsox_init(ai_lsm6dsox_t *device,
                           const ai_lsm6dsox_transport_t *transport,
                           const ai_lsm6dsox_config_t *config) {
	if (device == NULL || config == NULL ||
	    transport == NULL || transport->read_regs == NULL || transport->write_regs == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	device->transport = *transport;
	device->accel_sensitivity = ACCEL_SENSITIVITY[config->accel_fs];
	device->gyro_sensitivity = GYRO_SENSITIVITY[config->gyro_fs];

	vTaskDelay(pdMS_TO_TICKS(BOOT_DELAY_MS));

	esp_err_t err = ai_lsm6dsox_reset(device);
	if (err != ESP_OK) {
		return err;
	}

	uint8_t who_am_i = 0;
	err = ai_lsm6dsox_who_am_i(device, &who_am_i);
	if (err != ESP_OK) {
		return err;
	}
	if (who_am_i != AI_LSM6DSOX_WHO_AM_I_VALUE) {
		ESP_LOGE(TAG, "WHO_AM_I read 0x%02X, expected 0x%02X", who_am_i, AI_LSM6DSOX_WHO_AM_I_VALUE);
		return ESP_ERR_NOT_FOUND;
	}

	// IF_INC keeps burst reads walking forward through the output block, and
	// is what makes ai_lsm6dsox_read_all() a single transaction.
	uint8_t ctrl3_c = AI_LSM6DSOX_CTRL3_C_IF_INC;
	if (config->block_data_update) {
		ctrl3_c |= AI_LSM6DSOX_CTRL3_C_BDU;
	}
	err = write_reg(device, AI_LSM6DSOX_REG_CTRL3_C, ctrl3_c);
	if (err != ESP_OK) {
		return err;
	}

	err = write_reg(device, AI_LSM6DSOX_REG_CTRL1_XL,
	                (uint8_t)((config->accel_odr << AI_LSM6DSOX_ODR_SHIFT) |
	                          (config->accel_fs << AI_LSM6DSOX_FS_SHIFT)));
	if (err != ESP_OK) {
		return err;
	}

	err = write_reg(device, AI_LSM6DSOX_REG_CTRL2_G,
	                (uint8_t)((config->gyro_odr << AI_LSM6DSOX_ODR_SHIFT) |
	                          (config->gyro_fs << AI_LSM6DSOX_FS_SHIFT)));
	if (err != ESP_OK) {
		return err;
	}

	ESP_LOGI(TAG, "initialised: WHO_AM_I 0x%02X, accel %g g/LSB, gyro %g dps/LSB",
	         who_am_i, (double)device->accel_sensitivity, (double)device->gyro_sensitivity);
	return ESP_OK;
}

esp_err_t ai_lsm6dsox_data_ready(ai_lsm6dsox_t *device,
                                 bool *accel_ready,
                                 bool *gyro_ready,
                                 bool *temp_ready) {
	if (device == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t status = 0;
	esp_err_t err = read_regs(device, AI_LSM6DSOX_REG_STATUS, &status, 1);
	if (err != ESP_OK) {
		return err;
	}

	if (accel_ready != NULL) {
		*accel_ready = (status & AI_LSM6DSOX_STATUS_XLDA) != 0;
	}
	if (gyro_ready != NULL) {
		*gyro_ready = (status & AI_LSM6DSOX_STATUS_GDA) != 0;
	}
	if (temp_ready != NULL) {
		*temp_ready = (status & AI_LSM6DSOX_STATUS_TDA) != 0;
	}
	return ESP_OK;
}

esp_err_t ai_lsm6dsox_read_accel(ai_lsm6dsox_t *device, ai_lsm6dsox_vec3_t *out_accel) {
	if (device == NULL || out_accel == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t raw[6] = {0};
	esp_err_t err = read_regs(device, AI_LSM6DSOX_REG_OUTX_L_A, raw, sizeof(raw));
	if (err != ESP_OK) {
		return err;
	}

	decode_vec3(raw, device->accel_sensitivity, out_accel);
	return ESP_OK;
}

esp_err_t ai_lsm6dsox_read_gyro(ai_lsm6dsox_t *device, ai_lsm6dsox_vec3_t *out_gyro) {
	if (device == NULL || out_gyro == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t raw[6] = {0};
	esp_err_t err = read_regs(device, AI_LSM6DSOX_REG_OUTX_L_G, raw, sizeof(raw));
	if (err != ESP_OK) {
		return err;
	}

	decode_vec3(raw, device->gyro_sensitivity, out_gyro);
	return ESP_OK;
}

esp_err_t ai_lsm6dsox_read_temperature(ai_lsm6dsox_t *device, float *out_celsius) {
	if (device == NULL || out_celsius == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t raw[2] = {0};
	esp_err_t err = read_regs(device, AI_LSM6DSOX_REG_OUT_TEMP_L, raw, sizeof(raw));
	if (err != ESP_OK) {
		return err;
	}

	*out_celsius = TEMP_OFFSET_C + decode_i16(raw) / TEMP_LSB_PER_C;
	return ESP_OK;
}

esp_err_t ai_lsm6dsox_read_all(ai_lsm6dsox_t *device, ai_lsm6dsox_data_t *out_data) {
	if (device == NULL || out_data == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	// OUT_TEMP_L .. OUTZ_H_A: temperature, gyro XYZ, accel XYZ.
	uint8_t raw[14] = {0};
	esp_err_t err = read_regs(device, AI_LSM6DSOX_REG_OUT_TEMP_L, raw, sizeof(raw));
	if (err != ESP_OK) {
		return err;
	}

	out_data->temperature_c = TEMP_OFFSET_C + decode_i16(&raw[0]) / TEMP_LSB_PER_C;
	decode_vec3(&raw[2], device->gyro_sensitivity, &out_data->gyro);
	decode_vec3(&raw[8], device->accel_sensitivity, &out_data->accel);
	return ESP_OK;
}
