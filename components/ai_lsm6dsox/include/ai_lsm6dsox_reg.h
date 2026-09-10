// ai_lsm6dsox_reg.h
//
// LSM6DSOX register map, user bank 0. Only the registers this driver touches
// are listed; add more here rather than open-coding addresses.

#pragma once

#define AI_LSM6DSOX_REG_FUNC_CFG_ACCESS 0x01
#define AI_LSM6DSOX_REG_WHO_AM_I        0x0F
#define AI_LSM6DSOX_REG_CTRL1_XL        0x10
#define AI_LSM6DSOX_REG_CTRL2_G         0x11
#define AI_LSM6DSOX_REG_CTRL3_C         0x12
#define AI_LSM6DSOX_REG_CTRL4_C         0x13
#define AI_LSM6DSOX_REG_CTRL5_C         0x14
#define AI_LSM6DSOX_REG_CTRL6_C         0x15
#define AI_LSM6DSOX_REG_CTRL7_G         0x16
#define AI_LSM6DSOX_REG_CTRL8_XL        0x17
#define AI_LSM6DSOX_REG_CTRL9_XL        0x18
#define AI_LSM6DSOX_REG_CTRL10_C        0x19
#define AI_LSM6DSOX_REG_STATUS          0x1E

// Output block. Reading from OUT_TEMP_L with IF_INC set walks temperature,
// gyro XYZ and accel XYZ in one burst (14 bytes).
#define AI_LSM6DSOX_REG_OUT_TEMP_L      0x20
#define AI_LSM6DSOX_REG_OUTX_L_G        0x22
#define AI_LSM6DSOX_REG_OUTX_L_A        0x28

// WHO_AM_I fixed value.
#define AI_LSM6DSOX_WHO_AM_I_VALUE      0x6C

// CTRL3_C bits
#define AI_LSM6DSOX_CTRL3_C_BOOT        0x80
#define AI_LSM6DSOX_CTRL3_C_BDU         0x40
#define AI_LSM6DSOX_CTRL3_C_H_LACTIVE   0x20
#define AI_LSM6DSOX_CTRL3_C_PP_OD       0x10
#define AI_LSM6DSOX_CTRL3_C_SIM         0x08 // 0 = 4-wire SPI, 1 = 3-wire
#define AI_LSM6DSOX_CTRL3_C_IF_INC      0x04
#define AI_LSM6DSOX_CTRL3_C_SW_RESET    0x01

// STATUS_REG bits
#define AI_LSM6DSOX_STATUS_XLDA         0x01
#define AI_LSM6DSOX_STATUS_GDA          0x02
#define AI_LSM6DSOX_STATUS_TDA          0x04

// Field placement shared by CTRL1_XL and CTRL2_G
#define AI_LSM6DSOX_ODR_SHIFT           4
#define AI_LSM6DSOX_FS_SHIFT            2
