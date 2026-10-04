/* ===================================================================
 *  Sensor SPI bus bring-up diagnostic.
 *
 *  Temporary. Delete this file, its entry in main/CMakeLists.txt, and
 *  the call in app_main() once the bus is alive.
 *
 *  Both sensors on SPI2 read back a clean 0x00 while every transaction
 *  reports ESP_OK. Two different chips on two different chip selects
 *  failing identically means the fault is common to the bus, not to
 *  either device. This narrows that down without a logic analyser.
 *
 *  Three phases, cheapest first:
 *
 *    1. PIN TOGGLE. Squares waves on SCLK, MOSI and both CS lines, one
 *       pin at a time, so a multimeter or scope can confirm the signal
 *       actually appears on the header pad the wiring doc claims. If a
 *       pin is silent at the pad, the GPIO mapping is wrong and nothing
 *       downstream matters.
 *
 *    2. BIT-BANGED WHO_AM_I. Reads the LSM6DSOX and BMP388 ID registers
 *       by driving the four pins as plain GPIOs at ~50 kHz, bypassing
 *       the SPI peripheral, its clock divider, DMA and CS timing
 *       entirely. This is the phase that splits the search space: if
 *       bit-banging reads the right IDs, the wiring and the chips are
 *       fine and the bug is in the peripheral setup. If it also reads
 *       0x00, the problem is power, ground, pin mapping or the parts.
 *
 *    3. LOOPBACK (optional, needs a jumper from J1.2 MOSI to J1.4 MISO
 *       with both breakouts unplugged). Proves the SPI peripheral can
 *       hear itself, which isolates MISO's input path from everything
 *       the sensors do.
 * =================================================================== */

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t sclk;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t cs_imu;      /* LSM6DSOX, expects WHO_AM_I 0x0F == 0x6C */
    gpio_num_t cs_baro;     /* BMP388,   expects CHIP_ID  0x00 == 0x50 */
    bool       run_loopback;/* true only when the MOSI->MISO jumper is fitted */
} bus_bringup_cfg_t;

/* Runs all phases and logs the results. Leaves every pin back in a
 * high-impedance state, so the normal sensor_bus_init() can run after
 * it. Never returns an error -- it is a measurement, not a gate. */
void bus_bringup_run(const bus_bringup_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
