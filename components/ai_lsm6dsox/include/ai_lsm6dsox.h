// ai_lsm6dsox.h
//
// Driver for the ST LSM6DSOX 6-axis IMU (3-axis accelerometer + 3-axis gyro).
//
// The driver is bus agnostic: it reaches the part through a caller-supplied
// transport of two register callbacks, so the same code serves SPI and I2C.
// For SPI, ai_spi provides callbacks with a matching signature; see the
// sensor-spi project for the wiring.
//
// The caller owns the ai_lsm6dsox_t instance. Nothing here allocates.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register-level access to the part. Both callbacks take the bare register
// address; any read/write framing is the transport's business.
typedef struct {
	void *ctx;
	esp_err_t (*read_regs)(void *ctx, uint8_t reg, uint8_t *data, size_t len);
	esp_err_t (*write_regs)(void *ctx, uint8_t reg, const uint8_t *data, size_t len);
} ai_lsm6dsox_transport_t;

// Output data rate. Values are the raw ODR field encoding, shared by the
// accelerometer and the gyroscope.
typedef enum {
	AI_LSM6DSOX_ODR_POWER_DOWN = 0x0,
	AI_LSM6DSOX_ODR_12_5_HZ    = 0x1,
	AI_LSM6DSOX_ODR_26_HZ      = 0x2,
	AI_LSM6DSOX_ODR_52_HZ      = 0x3,
	AI_LSM6DSOX_ODR_104_HZ     = 0x4,
	AI_LSM6DSOX_ODR_208_HZ     = 0x5,
	AI_LSM6DSOX_ODR_416_HZ     = 0x6,
	AI_LSM6DSOX_ODR_833_HZ     = 0x7,
	AI_LSM6DSOX_ODR_1667_HZ    = 0x8,
	AI_LSM6DSOX_ODR_3333_HZ    = 0x9,
	AI_LSM6DSOX_ODR_6667_HZ    = 0xA,
} ai_lsm6dsox_odr_t;

// Accelerometer full scale. Note the encoding is not in magnitude order;
// these are the raw FS_XL values.
typedef enum {
	AI_LSM6DSOX_ACCEL_FS_2G  = 0x0,
	AI_LSM6DSOX_ACCEL_FS_16G = 0x1,
	AI_LSM6DSOX_ACCEL_FS_4G  = 0x2,
	AI_LSM6DSOX_ACCEL_FS_8G  = 0x3,
} ai_lsm6dsox_accel_fs_t;

// Gyroscope full scale, raw FS_G values.
typedef enum {
	AI_LSM6DSOX_GYRO_FS_250DPS  = 0x0,
	AI_LSM6DSOX_GYRO_FS_500DPS  = 0x1,
	AI_LSM6DSOX_GYRO_FS_1000DPS = 0x2,
	AI_LSM6DSOX_GYRO_FS_2000DPS = 0x3,
} ai_lsm6dsox_gyro_fs_t;

typedef struct {
	ai_lsm6dsox_odr_t accel_odr;
	ai_lsm6dsox_accel_fs_t accel_fs;
	ai_lsm6dsox_odr_t gyro_odr;
	ai_lsm6dsox_gyro_fs_t gyro_fs;
	bool block_data_update; // hold output registers until both bytes are read
} ai_lsm6dsox_config_t;

#define AI_LSM6DSOX_CONFIG_DEFAULT()             \
	(ai_lsm6dsox_config_t) {                     \
		.accel_odr = AI_LSM6DSOX_ODR_104_HZ,     \
		.accel_fs = AI_LSM6DSOX_ACCEL_FS_2G,     \
		.gyro_odr = AI_LSM6DSOX_ODR_104_HZ,      \
		.gyro_fs = AI_LSM6DSOX_GYRO_FS_250DPS,   \
		.block_data_update = true,               \
	}

typedef struct {
	float x;
	float y;
	float z;
} ai_lsm6dsox_vec3_t;

typedef struct {
	ai_lsm6dsox_vec3_t accel;   // g
	ai_lsm6dsox_vec3_t gyro;    // degrees per second
	float temperature_c;
} ai_lsm6dsox_data_t;

// Opaque to the caller in practice, but exposed so instances can live in
// static or stack storage.
typedef struct {
	ai_lsm6dsox_transport_t transport;
	float accel_sensitivity; // g per LSB
	float gyro_sensitivity;  // dps per LSB
} ai_lsm6dsox_t;

// Reset the part, check WHO_AM_I and apply `config`. Returns ESP_ERR_NOT_FOUND
// if WHO_AM_I does not read back 0x6C, which is the usual symptom of a wiring
// or bus-configuration problem.
esp_err_t ai_lsm6dsox_init(ai_lsm6dsox_t *device,
                           const ai_lsm6dsox_transport_t *transport,
                           const ai_lsm6dsox_config_t *config);

// Software reset, then wait for the part to clear the reset bit.
esp_err_t ai_lsm6dsox_reset(ai_lsm6dsox_t *device);

esp_err_t ai_lsm6dsox_who_am_i(ai_lsm6dsox_t *device, uint8_t *out_id);

// Per-sensor data-ready flags from STATUS_REG. Any out pointer may be NULL.
esp_err_t ai_lsm6dsox_data_ready(ai_lsm6dsox_t *device,
                                 bool *accel_ready,
                                 bool *gyro_ready,
                                 bool *temp_ready);

esp_err_t ai_lsm6dsox_read_accel(ai_lsm6dsox_t *device, ai_lsm6dsox_vec3_t *out_accel);
esp_err_t ai_lsm6dsox_read_gyro(ai_lsm6dsox_t *device, ai_lsm6dsox_vec3_t *out_gyro);
esp_err_t ai_lsm6dsox_read_temperature(ai_lsm6dsox_t *device, float *out_celsius);

// Single burst covering temperature, gyro and accelerometer.
esp_err_t ai_lsm6dsox_read_all(ai_lsm6dsox_t *device, ai_lsm6dsox_data_t *out_data);

#ifdef __cplusplus
}
#endif
