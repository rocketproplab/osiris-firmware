#ifndef LSM6DSOX_SENSOR_H
#define LSM6DSOX_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LSM6DSOX_MAX_INSTANCES    1
#define LSM6DSOX_STACK_WORDS      3072
#define LSM6DSOX_IRQ_TIMEOUT_MS   100   /* fallback poll period if INT stalls */

/* Wake-up (liftoff) defaults. The threshold register is 6 bits with an
 * LSB of FS_XL/64, so at +/-16 g one count is 0.25 g and the usable
 * range is 0.25 g to 15.75 g. 3 g sits well above anything handling the
 * airframe produces and well below any motor worth flying. */
#define LSM6DSOX_LIFTOFF_THRESHOLD_G   3.0f
#define LSM6DSOX_LIFTOFF_DUR_SAMPLES   2     /* at 104 Hz, ~19 ms sustained */

/* ===================================================================
 *  Public data -- plain value types, nothing to hide.
 * =================================================================== */

typedef struct {
    float    ax, ay, az;    // m/s^2
    float    gx, gy, gz;    // deg/s

    uint64_t timestamp_us;  // esp_timer time captured in the ISR, not in the task
    uint32_t seq;           // increments once per successful sample
} lsm6_data_t;

/* Hardware liftoff detection -- see the wake-up block notes in
 * lsm6dsox_sensor.c. Populated entirely from the INT2 ISR. */
typedef struct {
    bool     detected;
    uint64_t timestamp_us;  // ISR-captured, so this is the edge time, not
                             // the time some task got around to noticing
    uint32_t count;         // edges seen; >1 means the latch was re-armed
} lsm6dsox_liftoff_t;

typedef struct {
    spi_host_device_t host;
    gpio_num_t        cs_pin;
    gpio_num_t        int_pin;  // wired to LSM6DSOX INT1 (data-ready)
    gpio_num_t        int2_pin; // wired to LSM6DSOX INT2 (wake-up / liftoff).
                                 // GPIO_NUM_NC disables the whole feature.
    uint32_t          spi_freq_hz;

    /* Wake-up tuning. 0 means "use the LSM6DSOX_LIFTOFF_* default". */
    float             liftoff_threshold_g;
    uint8_t           liftoff_dur_samples;

    /* No configurable ODR/range fields -- this driver hardcodes
     * CTRL1_XL/CTRL2_G at accel 104 Hz/+-16 g, gyro 104 Hz/+-2000 dps.
     * The ranges were widened from the original +-4 g / 250 dps because
     * both would saturate through boost and roll on a real flight. */
} lsm6dsox_config_t;

typedef struct {
    uint32_t samples;
    uint32_t read_errors;
    uint32_t irq_count;
    uint32_t irq_timeouts;
    uint32_t liftoff_events;   // INT2 edges seen
} lsm6dsox_diag_t;

/* ===================================================================
 *  Opaque handle -- struct lsm6dsox_dev is only ever defined inside
 *  lsm6dsox_sensor.c. Same idiom as bmp388_handle_t: code elsewhere can
 *  hold and pass this pointer around, but the compiler refuses any
 *  attempt to read or write its fields directly.
 * =================================================================== */

typedef struct lsm6dsox_dev *lsm6dsox_handle_t;

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

/* Claims a slot from the static pool, adds the LSM6DSOX as a device on
 * the given SPI bus (already initialized by the caller), and configures
 * it: WHO_AM_I check, software reset, BDU, I3C disable, accel/gyro
 * ODR + range, INT1 routed to data-ready. Does not start the reader
 * task yet. On success, *out is a handle for every other call to use. */
esp_err_t lsm6dsox_init(const lsm6dsox_config_t *cfg, lsm6dsox_handle_t *out);

/* Spawns the FreeRTOS task that sleeps until INT1 fires, then reads and
 * scales one accel+gyro sample. */
esp_err_t lsm6dsox_start(lsm6dsox_handle_t dev, UBaseType_t priority, BaseType_t core_id);

/* Tears down the task and ISR, and releases the pool slot. */
esp_err_t lsm6dsox_stop(lsm6dsox_handle_t dev);

/* ===================================================================
 *  Data access
 * =================================================================== */

void lsm6dsox_get_latest(lsm6dsox_handle_t dev, lsm6_data_t *out);

esp_err_t lsm6dsox_wait_for_sample(lsm6dsox_handle_t dev, uint32_t last_seq,
                                   lsm6_data_t *out, TickType_t timeout);

esp_err_t lsm6dsox_read_once(lsm6dsox_handle_t dev, lsm6_data_t *out);

void lsm6dsox_get_diagnostics(lsm6dsox_handle_t dev, lsm6dsox_diag_t *out);

/* ===================================================================
 *  Rate control -- standby vs flight
 * =================================================================== */

/* fast=true  : accel 104 Hz, gyro 104 Hz          (flight)
 * fast=false : accel 12.5 Hz, GYRO POWERED DOWN   (standby)
 *
 * The gyro is the bulk of this part's current draw, and the wake-up
 * block only uses the accelerometer -- so powering the gyro down in
 * standby saves most of the IMU budget while leaving the backup liftoff
 * trigger fully working.
 *
 * While the gyro is down, gx/gy/gz are published as exactly 0. That is
 * not a reading. Cross-reference EV_MODE_CHANGED in event.bin to see
 * which samples were taken with the gyro off.
 *
 * MUST be called from the task that owns the SPI device -- it issues bus
 * transactions. lsm6dsox_task() does this itself by watching
 * flight_mode; nothing else should call it. */
esp_err_t lsm6dsox_set_rate(lsm6dsox_handle_t dev, bool fast);

/* Which rate the driver is currently configured for. */
bool lsm6dsox_is_fast(lsm6dsox_handle_t dev);

/* ===================================================================
 *  Hardware liftoff
 * =================================================================== */

/* True once the sensor's own wake-up block has fired on INT2. This is
 * independent of the software liftoff detector in flight_logic.c on
 * purpose -- the two disagreeing is itself worth knowing. */
bool lsm6dsox_liftoff_detected(lsm6dsox_handle_t dev);

void lsm6dsox_get_liftoff(lsm6dsox_handle_t dev, lsm6dsox_liftoff_t *out);

#endif // LSM6DSOX_SENSOR_H
