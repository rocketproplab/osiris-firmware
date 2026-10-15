#ifndef BMP388_SENSOR_H
#define BMP388_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmp3.h"

#define BMP388_MAX_INSTANCES    1
#define BMP388_STACK_WORDS      3072
#define BMP388_IRQ_TIMEOUT_MS   100   /* fallback poll period if INT stalls (FLIGHT, 50 Hz) */
/* STANDBY runs at 1.5 Hz, a sample every ~667 ms. A 100 ms timeout there
 * "times out" six times between two perfectly healthy interrupts, so the
 * slow profile gets its own: three missed samples' worth. */
#define BMP388_IRQ_TIMEOUT_SLOW_MS 2000

/* ===================================================================
 *  Public data -- plain value types. Nothing to hide here; these are
 *  just data you're meant to read and write directly.
 * =================================================================== */

typedef struct {
    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;    // relative to whatever bmp388_set_reference_pressure() last set

    uint64_t timestamp_us;  // esp_timer time captured in the ISR, not in the task
    uint32_t seq;           // increments once per successful sample
} bmp388_data_t;

/* No bus fields: which bus, which pins and what clock are decided by
 * sensor_bus (menuconfig -> SET sensor bus). */
typedef struct {
    gpio_num_t        int_pin;

    uint8_t           osr_pressure;   // BMP3_OVERSAMPLING_Xx
    uint8_t           osr_temp;       // BMP3_OVERSAMPLING_Xx
    uint8_t           odr;            // BMP3_ODR_xx_HZ
    uint8_t           iir_filter;     // BMP3_IIR_FILTER_COEFF_x

    float             ref_pressure_pa; // 0 => defaults to 101325 Pa
} bmp388_config_t;

typedef struct {
    uint32_t samples;
    uint32_t read_errors;
    uint32_t irq_count;
    uint32_t irq_timeouts;
    /* Contact with the chip -- see sensor_health.h. */
    bool     lost;             // true right now: not publishing, retrying
    uint32_t lost_count;       // times contact was lost
    uint32_t recoveries;       // times it came back
    uint32_t retry_attempts;   // re-init attempts across all outages
    uint32_t last_outage_ms;   // length of the most recent outage
} bmp388_diag_t;

/* ===================================================================
 *  Opaque handle
 *
 *  struct bmp388_dev is intentionally never defined in this header --
 *  only in bmp388_sensor.c. Code that includes this header can hold a
 *  bmp388_handle_t, store it, pass it to these functions, but cannot
 *  see or touch a single field of it. That's what makes it safe for
 *  flight_logic.c (or anyone else) to hold one: there's no way to
 *  accidentally read dev->latest directly and skip the spinlock, or
 *  poke dev->in_use, because the compiler doesn't know those fields
 *  exist from outside this driver.
 * =================================================================== */

typedef struct bmp388_dev *bmp388_handle_t;

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

/* Claims a slot from the static pool and brings the BMP388 up via Bosch's
 * official BMP3 API, over sensor_bus (call sensor_bus_init() first).
 * Arms the INT pin, but does not start the reader task yet.
 * On success, *out is a handle for every other call to use. */
esp_err_t bmp388_init(const bmp388_config_t *cfg, bmp388_handle_t *out);

/* Spawns the FreeRTOS task that sleeps until BMP388's INT pin fires, then
 * reads and compensates a sample via bmp3_get_sensor_data(). */
esp_err_t bmp388_start(bmp388_handle_t dev, UBaseType_t priority, BaseType_t core_id);

/* Tears down the task and ISR, and releases the pool slot. */
esp_err_t bmp388_stop(bmp388_handle_t dev);

/* ===================================================================
 *  Data access
 * =================================================================== */

void bmp388_get_latest(bmp388_handle_t dev, bmp388_data_t *out);

esp_err_t bmp388_wait_for_sample(bmp388_handle_t dev, uint32_t last_seq,
                                 bmp388_data_t *out, TickType_t timeout);

esp_err_t bmp388_read_once(bmp388_handle_t dev, bmp388_data_t *out);

void bmp388_set_reference_pressure(bmp388_handle_t dev, float pressure_pa);

void bmp388_zero_altitude(bmp388_handle_t dev);

void bmp388_get_diagnostics(bmp388_handle_t dev, bmp388_diag_t *out);

/* ===================================================================
 *  Rate control -- standby vs flight
 * =================================================================== */

/* fast=true  : the configured flight profile (50 Hz, 8x pressure OSR)
 * fast=false : 1.5 Hz with pressure oversampling dropped to 2x
 *
 * Oversampling drops with the rate, not just the rate: this part's
 * current is dominated by conversions averaged per sample, so 8x OSR at
 * 1.5 Hz would give up most of the saving. Standby altitude is noisier
 * as a result -- the right trade for a vehicle sitting on a pad.
 *
 * MUST be called from this sensor's own task -- Bosch's dev struct is not
 * safe to share. bmp388_task() does this itself by watching flight_mode. */
esp_err_t bmp388_set_rate(bmp388_handle_t dev, bool fast);

bool bmp388_is_fast(bmp388_handle_t dev);

#endif // BMP388_SENSOR_H
