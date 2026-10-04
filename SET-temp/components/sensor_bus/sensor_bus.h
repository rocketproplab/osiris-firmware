/*
 * sensor_bus -- the one place that knows how the BMP388 and LSM6DSOX are wired.
 *
 * The two sensors share three wires. Depending on a build-time switch, those
 * wires run as an I2C bus or an SPI bus:
 *
 *     idf.py menuconfig  ->  SET sensor bus  ->  Interface
 *
 * SPI IS NOT IMPLEMENTED YET. Every function below returns
 * ESP_ERR_NOT_SUPPORTED in an SPI build; I2C is the default and the one
 * that works. See README.md and the stubs at the top of sensor_bus.c.
 *
 *   wire     SPI build          I2C build
 *   GPIO54   clock (SCK)        clock (SCL)
 *   GPIO19   data in (MOSI)     data, both directions (SDA)
 *   GPIO18   data out (MISO)    address select -- driven LOW by this code
 *   GPIO14   BMP388 CS          held HIGH (HIGH = "use I2C")
 *   GPIO17   LSM6DSOX CS        held HIGH
 *
 * Drivers never call spi_* or i2c_* themselves. They call the functions
 * below, which mean the same thing on either bus.
 *
 * AFTER SWITCHING BUILDS, REMOVE ALL POWER BEFORE TESTING. The BMP388 locks
 * itself into SPI the first time its CS goes low and stays there until it
 * loses power. Resetting the ESP32 is not enough.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSOR_BUS_BMP388 = 0,
    SENSOR_BUS_LSM6DSOX,
    SENSOR_BUS_DEV_COUNT
} sensor_bus_dev_t;

/* Largest single transfer, in bytes, counting the register-address byte and
 * (BMP388 on SPI) its dummy byte. 64 lets the SPI bus run without DMA. */
#define SENSOR_BUS_MAX_XFER 64

/* Bring the bus up. Call once, before any sensor driver's init. Safe to call
 * again (returns ESP_OK). The I2C build can also scan and log every address
 * that answers -- that scan ships commented out; see README.md. */
esp_err_t sensor_bus_init(void);

bool        sensor_bus_is_spi(void);
const char *sensor_bus_name(void);   /* "SPI" or "I2C", for logs */

/*
 * REGISTER READ: "give me `len` bytes starting at register `reg`."
 * Applies each bus's rules for you -- on SPI it sets the read bit (0x80) and,
 * for the BMP388, throws away the dummy byte that chip sends first. On I2C
 * there are no such rules. `reg` must be 0x00..0x7F.
 *
 * Use this from the LSM6DSOX driver and anything else that is NOT going
 * through Bosch's bmp3.c.
 */
esp_err_t sensor_bus_read(sensor_bus_dev_t dev, uint8_t reg,
                          uint8_t *buf, size_t len);

/* REGISTER WRITE: sends `reg`, then `buf` exactly as given. Identical on both
 * buses (on SPI a write's top bit is already 0). `reg` must be 0x00..0x7F. */
esp_err_t sensor_bus_write(sensor_bus_dev_t dev, uint8_t reg,
                           const uint8_t *buf, size_t len);

/*
 * RAW READ: sends `addr_byte` exactly as given, returns the next `len` bytes
 * exactly as they arrived. Applies NO rules.
 *
 * This exists for one caller: the read callback you hand to Bosch's bmp3.c.
 * That library already sets the read bit and strips the dummy byte itself
 * (when its dev->intf is BMP3_SPI_INTF). If its callback did either again,
 * the chip would be asked for the wrong register. One layer, one rule.
 */
esp_err_t sensor_bus_raw_read(sensor_bus_dev_t dev, uint8_t addr_byte,
                              uint8_t *buf, size_t len);

/* Read BMP388 CHIP_ID (0x00) and LSM6DSOX WHO_AM_I (0x0F), log both with a
 * plain-English verdict. ESP_OK only if both are right. The definitive
 * "is the bus working" test -- run it right after sensor_bus_init(). */
esp_err_t sensor_bus_check_ids(void);

/*
 * SPI build only (no-op on I2C, and a no-op in practice while SPI is
 * unimplemented): sets the LSM6DSOX's I2C_disable bit so it
 * stops listening for I2C while the BMP388 is being talked to. Called by
 * sensor_bus_init(). A software reset of the LSM6DSOX (CTRL3_C SW_RESET or
 * BOOT) clears that bit -- call this again after any such reset.
 */
esp_err_t sensor_bus_harden(void);

typedef struct {
    uint32_t xfers;          /* transfers attempted                          */
    uint32_t errors;         /* transfers the driver reported as failed       */
    uint32_t lock_timeouts;  /* gave up waiting for the other task's transfer */
    uint32_t i2c_resets;     /* I2C bus-recovery resets (I2C build only)      */
} sensor_bus_diag_t;

void sensor_bus_get_diag(sensor_bus_diag_t *out);

#ifdef __cplusplus
}
#endif
