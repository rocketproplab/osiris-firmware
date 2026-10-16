#include <stdio.h>
#include "lsm6dsox_sensor.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "data_router.h"
#include "event_log.h"
#include "flight_mode.h"
#include "sensor_bus.h"
#include "sensor_health.h"

static const char *TAG = "LSM6DSOX";

/* ===================================================================
 *  Registers -- identical addresses to the I2C version.
 * =================================================================== */

#define LSM6DSOX_WHO_AM_I_REG   0x0F
#define LSM6DSOX_WHO_AM_I_VAL   0x6C
#define LSM6DSOX_CTRL1_XL       0x10    // Accelerometer control
#define LSM6DSOX_CTRL2_G        0x11    // Gyroscope control
#define LSM6DSOX_CTRL3_C        0x12    // Control register 3 (SW_RESET, BDU)
#define LSM6DSOX_CTRL9_XL       0x18    // Control register 9 (I3C disable)
#define LSM6DSOX_CTRL9_I3C_DISABLE 0x02 // bit 1. Bit 0 is not-used and must stay 0.
#define LSM6DSOX_INT1_CTRL      0x0D    // Routes DRDY_XL / DRDY_G to INT1
#define LSM6DSOX_STATUS_REG     0x1E    // XLDA bit 0, GDA bit 1, TDA bit 2
#define LSM6DSOX_OUTX_L_G       0x22    // Gyro output start
#define LSM6DSOX_OUTX_L_A       0x28    // Accel output start

/* Wake-up (activity) block -- used here as a hardware liftoff detector. */
#define LSM6DSOX_WAKE_UP_SRC    0x1B    // WU_IA and per-axis flags
#define LSM6DSOX_TAP_CFG0       0x56    // SLOPE_FDS, LIR
#define LSM6DSOX_TAP_CFG2       0x58    // INTERRUPTS_ENABLE
#define LSM6DSOX_WAKE_UP_THS    0x5B    // WK_THS[5:0]
#define LSM6DSOX_WAKE_UP_DUR    0x5C    // WAKE_DUR[6:5]
#define LSM6DSOX_MD2_CFG        0x5F    // Routes wake-up to INT2

/* CTRL1_XL = 0x44: ODR_XL 0100 (104 Hz), FS_XL 01 (+/-16 g).
 * Note the FS_XL encoding is not in numeric order on this part --
 * 00 = +/-2 g, 01 = +/-16 g, 10 = +/-4 g, 11 = +/-8 g. Reading 01 as
 * "the second smallest range" is the easy way to get this wrong. */
#define LSM6DSOX_CTRL1_XL_VAL   0x44
/* CTRL2_G = 0x4C: ODR_G 0100 (104 Hz), FS_G 11 (+/-2000 dps). */
#define LSM6DSOX_CTRL2_G_VAL    0x4C

/* Standby values. ODR_XL 0001 = 12.5 Hz, with the SAME +/-16 g full
 * scale, so the wake-up threshold scaling (FS/64 per count) is unchanged
 * across the transition and the 3 g trip point means 3 g in both modes.
 * ODR_G 0000 = power-down: the gyro stops drawing current entirely. */
#define LSM6DSOX_CTRL1_XL_SLOW  0x14
#define LSM6DSOX_CTRL2_G_SLOW   0x00

/* Accel +/-16g: 0.488 mg/LSB -> m/s^2.
 * Widened from +/-4 g: a hobby motor pulls well past 4 g, and a clipped
 * boost plateau destroys burnout timing and any velocity obtained by
 * integration. The resolution given up is not real -- 0.488 mg/LSB is
 * already below this part's own ~0.5 mg RMS noise at 104 Hz, so the
 * measurement stays noise-limited either way. */
#define ACCEL_SCALE             (0.488f * 9.80665f / 1000.0f)
/* Gyro +/-2000 dps: 70 mdps/LSB -> dps.
 * Widened from 250 dps, which is only 42 rpm -- rockets routinely roll
 * several revolutions per second, and a saturated roll rate makes any
 * attitude integration meaningless. */
#define GYRO_SCALE              (70.0f / 1000.0f)

/* Wake-up threshold LSB, in g. The register is 6 bits with an LSB of
 * FS_XL/64; at +/-16 g that is 0.25 g per count. */
#define WAKE_THS_LSB_G          (16.0f / 64.0f)

/* Bus framing lives in sensor_bus, not here. For the record, because it
 * was wrong here before: on the LSM6DSOX the SPI address byte is bit7 = R/W
 * and bits 6..0 = the register address. There is NO "MS" auto-increment bit
 * -- that is the convention of older ST parts (LIS3MDL, L3GD20). On this
 * chip bit 6 is part of the address (TAP_CFG0 at 0x56 needs it), and
 * auto-increment is CTRL3_C IF_INC, which is on by default. Setting 0x40 on
 * a gyro read turned 0x22 into 0x62 (I3C_BUS_AVB) and the accel read 0x28
 * into 0x68 (unused), so every sample came from the wrong registers. */

/* ===================================================================
 *  The real struct -- private to this file, same idiom as bmp388_dev.
 *  Everything that used to be scattered file-scope statics now lives
 *  in one place, one per instance.
 * =================================================================== */

struct lsm6dsox_dev {
    bool                in_use;
    lsm6dsox_config_t   cfg;

    bool                initialized;

    lsm6_data_t         latest;
    portMUX_TYPE        latest_lock;

    StaticTask_t        tcb;
    StackType_t         stack[LSM6DSOX_STACK_WORDS];
    TaskHandle_t        task;

    volatile uint64_t   isr_timestamp_us;
    bool                isr_installed;

    /* INT2 / wake-up. Written only by the INT2 ISR, read by anyone. */
    bool                fast;      /* current rate configuration */
    uint8_t             gyro_check_left;  /* samples to dump after the gyro powers up */
    volatile bool       liftoff_detected;
    volatile uint64_t   liftoff_timestamp_us;
    volatile uint32_t   liftoff_count;
    bool                int2_isr_installed;

    lsm6dsox_diag_t     diag;

    /* Lost / recovered bookkeeping. Touched only by lsm6dsox_task after
     * init; read by lsm6dsox_get_diagnostics() for the console. */
    sensor_health_t     health;
};

/* ===================================================================
 *  Static instance pool
 * =================================================================== */

static struct lsm6dsox_dev s_pool[LSM6DSOX_MAX_INSTANCES];
static portMUX_TYPE        s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* ===================================================================
 *  Register access -- through sensor_bus, so the same two functions work
 *  whether the build is SPI or I2C. The bus layer adds the SPI read bit;
 *  pass plain register addresses here.
 * =================================================================== */

static esp_err_t lsm6dsox_read_regs(lsm6dsox_handle_t dev, uint8_t reg,
                                    uint8_t *dst, size_t len)
{
    if (!dev || !dst || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = sensor_bus_read(SENSOR_BUS_LSM6DSOX, reg, dst, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s read 0x%02X failed: %s", sensor_bus_name(), reg,
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t lsm6dsox_write_reg(lsm6dsox_handle_t dev, uint8_t reg, uint8_t val)
{
    (void)dev;
    esp_err_t err = sensor_bus_write(SENSOR_BUS_LSM6DSOX, reg, &val, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s write 0x%02X failed: %s", sensor_bus_name(), reg,
                 esp_err_to_name(err));
    }
    return err;
}

/* ===================================================================
 *  ISR
 * =================================================================== */

static void IRAM_ATTR lsm6dsox_isr(void *arg)
{
    lsm6dsox_handle_t dev = (lsm6dsox_handle_t)arg;
    BaseType_t hp_task_woken = pdFALSE;

    dev->isr_timestamp_us = (uint64_t)esp_timer_get_time();
    dev->diag.irq_count++;

    vTaskNotifyGiveFromISR(dev->task, &hp_task_woken);
    portYIELD_FROM_ISR(hp_task_woken);
}

/* ===================================================================
 *  Is the chip still there?  (see sensor_health.h for the why)
 * =================================================================== */

static esp_err_t lsm6dsox_configure_sensor(lsm6dsox_handle_t dev, bool boot);

/* WHO_AM_I (0x0F) and CTRL1_XL (0x10) are adjacent, and register
 * auto-increment (CTRL3_C IF_INC) is on by default -- including straight
 * after a reset -- so one two-byte read answers both questions: is the
 * chip there, and is it still configured? A reset puts CTRL1_XL back to
 * 0x00, accelerometer powered down, which no configuration of ours ever
 * writes: standby is 12.5 Hz, not off. */
static bool lsm6dsox_probe(lsm6dsox_handle_t dev, bool *configured)
{
    uint8_t r[2] = {0};
    if (sensor_bus_read(SENSOR_BUS_LSM6DSOX, LSM6DSOX_WHO_AM_I_REG, r, 2) != ESP_OK) {
        return false;
    }
    if (r[0] != LSM6DSOX_WHO_AM_I_VAL) {
        return false;
    }
    if (configured) {
        *configured = (r[1] & 0xF0) != 0;   /* ODR_XL non-zero */
    }
    return true;
}

/* Bring a chip that just reappeared back to the state the running
 * firmware expects: software reset, the boot configuration WITHOUT its
 * two 200 ms settling delays (a recovery in flight cannot afford 400 ms
 * of nothing; the first few samples after an ODR change are merely
 * less accurate), the INT2 wake-up block re-armed, then the rate for
 * the CURRENT flight mode.
 *
 * The software liftoff latch (dev->liftoff_detected) lives in the ESP,
 * not the chip, so a reset cannot clear it. Re-arming the wake-up block
 * mid-flight may latch INT2 again on the next acceleration; the ISR
 * only sets a flag that is already set, so that is harmless. */
static bool lsm6dsox_recover(lsm6dsox_handle_t dev)
{
    if (!lsm6dsox_probe(dev, NULL)) {
        return false;
    }
    if (lsm6dsox_configure_sensor(dev, false) != ESP_OK) {
        return false;
    }
    if (!flight_mode_is_fast() && lsm6dsox_set_rate(dev, false) != ESP_OK) {
        return false;
    }
    /* DRDY on INT1 stays high until the data is read, and the ISR is
     * edge-triggered: a sample that became ready during the re-init
     * would hold the line high and no edge would ever come. Reading the
     * accelerometer once lets the line fall so the next sample rises. */
    uint8_t discard[6];
    (void)sensor_bus_read(SENSOR_BUS_LSM6DSOX, LSM6DSOX_OUTX_L_A, discard, 6);
    return true;
}

/* ===================================================================
 *  Reader task
 * =================================================================== */

static void lsm6dsox_task(void *arg)
{
    lsm6dsox_handle_t dev = (lsm6dsox_handle_t)arg;

    ESP_LOGI(TAG, "reader task running on core %d", (int)xPortGetCoreID());

    for (;;) {
        TickType_t timeout = pdMS_TO_TICKS(LSM6DSOX_IRQ_TIMEOUT_MS);
        if (dev->health.lost) {
            timeout = pdMS_TO_TICKS(sensor_health_retry_wait_ms(
                                        &dev->health, (uint64_t)esp_timer_get_time()));
        }
        uint32_t notified = ulTaskNotifyTake(pdTRUE, timeout);
        uint64_t now_us   = (uint64_t)esp_timer_get_time();

        /* Reconfiguration happens HERE, in this sensor's own task --
         * never from whatever task changed the mode. The telemetry task
         * accepting an ARM command must not reach into this sensor. (The
         * bus itself is locked by sensor_bus; this rule keeps `dev->fast`
         * and the chip's registers owned by one task.) Polling costs one
         * comparison per sample. */
        /* The hardware liftoff latch is the backup path into FLIGHT --
         * it is what makes the vehicle switch to flight rates even if
         * the radio never worked. Called from the task, not the ISR:
         * flight_mode_note_liftoff() ends in a ring-buffer push, which
         * is not ISR-safe. It is idempotent and the latch is level-
         * triggered, so calling it every pass is the intended usage. */
        /* Before anything that can bail out: the latch is set by the
         * INT2 ISR, which needs no SPI, so a liftoff latched just before
         * the chip dropped off the bus must still move the mode. */
        if (dev->liftoff_detected) {
            flight_mode_note_liftoff(MODE_SRC_IMU_WAKEUP);
        }

        /* ---- LOST: retry on schedule, publish nothing ----------------
         * See bmp388_task: an absent chip reads back as zeros or 0xFF,
         * and an IMU sample of all zeros looks exactly like free fall. */
        if (dev->health.lost) {
            if (sensor_health_retry_due(&dev->health, now_us)) {
                bool ok = lsm6dsox_recover(dev);
                sensor_health_retry_result(&dev->health, ok,
                                           (uint64_t)esp_timer_get_time());
            }
            continue;
        }

        bool want_fast = flight_mode_is_fast();
        if (want_fast != dev->fast) {
            lsm6dsox_set_rate(dev, want_fast);
        }

        /* ---- Health: on every missed interrupt, and once a second ----
         * Same three-way split as the barometer. Chip fine but INT1
         * quiet still falls through to the polling read, as before. */
        if (notified == 0 || sensor_health_check_due(&dev->health, now_us)) {
            bool configured = false;
            if (!lsm6dsox_probe(dev, &configured)) {
                sensor_health_lost(&dev->health, SENSOR_LOST_NO_RESPONSE, now_us);
                continue;
            }
            if (!configured) {
                sensor_health_lost(&dev->health, SENSOR_LOST_RESET, now_us);
                continue;
            }
        }

        uint64_t stamp;
        if (notified == 0) {
            dev->diag.irq_timeouts++;
            event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_IRQ_TIMEOUT, ESP_ERR_TIMEOUT, dev->diag.irq_timeouts);
            stamp = now_us;
        } else {
            stamp = dev->isr_timestamp_us;
        }

        uint8_t gyro_raw[6], accel_raw[6];
        esp_err_t err = lsm6dsox_read_regs(dev, LSM6DSOX_OUTX_L_G, gyro_raw, 6);
        if (err == ESP_OK) {
            err = lsm6dsox_read_regs(dev, LSM6DSOX_OUTX_L_A, accel_raw, 6);
        }
        if (err != ESP_OK) {
            dev->diag.read_errors++;
            event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_READ_FAILED, err, dev->diag.read_errors);
            /* SPI never gets here; on I2C this is a NACK, which is what
             * a missing chip looks like on that bus. */
            if (!lsm6dsox_probe(dev, NULL)) {
                sensor_health_lost(&dev->health, SENSOR_LOST_BUS_ERROR, now_us);
            }
            continue;
        }

        /* Every accelerometer byte identical at 0x00 or 0xFF is what an
         * open data line reads back while DRDY keeps pulsing. A real
         * sample hitting exactly that is vanishingly unlikely -- even in
         * coast, drag and noise put a few counts on some axis -- but it
         * only triggers a check, never a verdict on its own. */
        bool all00 = true, allFF = true;
        for (int i = 0; i < 6; i++) {
            if (accel_raw[i] != 0x00) { all00 = false; }
            if (accel_raw[i] != 0xFF) { allFF = false; }
        }
        if ((all00 || allFF) && !lsm6dsox_probe(dev, NULL)) {
            sensor_health_lost(&dev->health, SENSOR_LOST_NO_RESPONSE, now_us);
            continue;
        }

        /* The first few samples after the gyro is powered up (STANDBY ->
         * ARMED/FLIGHT), printed raw with the chip's own data-ready flags.
         * GDA=1 with non-zero bytes: the gyro is converting. GDA=0 or every
         * byte 0x00: it is not, whatever CTRL2_G says. */
        if (dev->gyro_check_left) {
            uint8_t st = 0;
            (void)lsm6dsox_read_regs(dev, LSM6DSOX_STATUS_REG, &st, 1);
            ESP_LOGW(TAG, "gyro check %u/3: raw %02X %02X %02X %02X %02X %02X  "
                          "STATUS 0x%02X (GDA=%d XLDA=%d)  fast=%d",
                     (unsigned)(4 - dev->gyro_check_left),
                     gyro_raw[0], gyro_raw[1], gyro_raw[2], gyro_raw[3],
                     gyro_raw[4], gyro_raw[5], st, (st >> 1) & 1, st & 1, (int)dev->fast);
            dev->gyro_check_left--;
        }

        int16_t gx = (int16_t)((gyro_raw[1] << 8) | gyro_raw[0]);
        int16_t gy = (int16_t)((gyro_raw[3] << 8) | gyro_raw[2]);
        int16_t gz = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]);
        int16_t ax = (int16_t)((accel_raw[1] << 8) | accel_raw[0]);
        int16_t ay = (int16_t)((accel_raw[3] << 8) | accel_raw[2]);
        int16_t az = (int16_t)((accel_raw[5] << 8) | accel_raw[4]);

        lsm6_data_t sample;
        sample.ax = ax * ACCEL_SCALE;
        sample.ay = ay * ACCEL_SCALE;
        sample.az = az * ACCEL_SCALE;
        /* With the gyro powered down its output registers hold whatever
         * was last converted. Publishing that would put a stale reading
         * in the log wearing the same units as a real one, so standby
         * samples carry an explicit zero instead. EV_MODE_CHANGED in
         * event.bin is what tells a decoder which samples those are. */
        if (dev->fast) {
            sample.gx = gx * GYRO_SCALE;
            sample.gy = gy * GYRO_SCALE;
            sample.gz = gz * GYRO_SCALE;
        } else {
            sample.gx = 0.0f;
            sample.gy = 0.0f;
            sample.gz = 0.0f;
        }
        sample.timestamp_us = stamp;
        sample.seq           = ++dev->diag.samples;

        portENTER_CRITICAL(&dev->latest_lock);
        dev->latest = sample;
        portEXIT_CRITICAL(&dev->latest_lock);

        /* Same reasoning as bmp388_task: pushed here, right where the
         * sample already exists, instead of a separate bridge task
         * re-fetching it via a second lock+copy. */
        imu_record_t rec = {
            .seq          = sample.seq,
            .timestamp_us = sample.timestamp_us,
            .ax = sample.ax, .ay = sample.ay, .az = sample.az,
            .gx = sample.gx, .gy = sample.gy, .gz = sample.gz,
        };
        data_router_publish(STREAM_IMU, &rec, sizeof(rec));
    }
}

/* INT2 ISR. The wake-up event is latched in the sensor, so this fires
 * once and then the line stays high. No bus access here -- reading WAKE_UP_SRC
 * to learn which axis tripped would mean a bus transaction from
 * interrupt context, and the axis is not worth that. */
static void IRAM_ATTR lsm6dsox_liftoff_isr(void *arg)
{
    struct lsm6dsox_dev *dev = (struct lsm6dsox_dev *)arg;

    if (!dev->liftoff_detected) {
        dev->liftoff_timestamp_us = (uint64_t)esp_timer_get_time();
        dev->liftoff_detected     = true;
    }
    dev->liftoff_count++;
}

/* ===================================================================
 *  Init -- same sequence as the working I2C version.
 * =================================================================== */

/* ===================================================================
 *  Hardware liftoff detection -- the LSM6DSOX's wake-up block.
 *
 *  This runs entirely inside the sensor. The accelerometer stream goes
 *  through a filter that removes the DC 1 g of gravity; what remains is
 *  compared against a 6-bit threshold (WAKE_UP_THS), and WAKE_DUR
 *  requires the threshold to hold for N consecutive ODR samples before
 *  the event asserts. That duration counter is what stops a knock on
 *  the pad from firing it.
 *
 *  SLOPE_FDS = 1 selects the high-pass filter rather than the slope
 *  filter, and that choice matters more than it looks. The slope filter
 *  compares consecutive samples, so it measures rate of change: a motor
 *  that takes ~100 ms to reach full thrust only changes by about 1 g per
 *  sample at 104 Hz, and a 3 g slope threshold would never fire. The
 *  high-pass filter instead passes the full step and decays with its own
 *  time constant, so the ~11 g jump at ignition stays above threshold
 *  long enough for the duration counter to confirm it.
 *
 *  LIR = 1 latches the event. Nothing clears it, which is deliberate:
 *  liftoff happens once, the latch means INT2 stays asserted for the
 *  rest of the flight (probe-able on the bench), and no further edges
 *  means the ISR cannot be re-entered by vibration. Reading
 *  WAKE_UP_SRC would re-arm it if that were ever wanted.
 *
 *  UNVERIFIED: the register values below follow the LSM6DSOX datasheet
 *  but have not been confirmed against this board. Bench-test before
 *  flight -- a sharp downward jerk by hand should exceed 3 g and latch
 *  INT2 high. If it never fires, suspect SLOPE_FDS or the threshold
 *  scaling first.
 * =================================================================== */

static esp_err_t lsm6dsox_configure_wakeup(lsm6dsox_handle_t dev)
{
    float thr_g = dev->cfg.liftoff_threshold_g > 0.0f
                    ? dev->cfg.liftoff_threshold_g
                    : LSM6DSOX_LIFTOFF_THRESHOLD_G;
    uint8_t dur = dev->cfg.liftoff_dur_samples ? dev->cfg.liftoff_dur_samples
                                               : LSM6DSOX_LIFTOFF_DUR_SAMPLES;

    int code = (int)((thr_g / WAKE_THS_LSB_G) + 0.5f);
    if (code < 1)  { code = 1; }
    if (code > 63) { code = 63; }
    if (dur > 3)   { dur = 3; }     /* WAKE_DUR is 2 bits */

    /* High-pass filter for the wake-up path, and latch the event. */
    esp_err_t err = lsm6dsox_write_reg(dev, LSM6DSOX_TAP_CFG0, 0x11);
    if (err != ESP_OK) { ESP_LOGE(TAG, "TAP_CFG0 write failed"); return err; }

    err = lsm6dsox_write_reg(dev, LSM6DSOX_WAKE_UP_THS, (uint8_t)code);
    if (err != ESP_OK) { ESP_LOGE(TAG, "WAKE_UP_THS write failed"); return err; }

    err = lsm6dsox_write_reg(dev, LSM6DSOX_WAKE_UP_DUR, (uint8_t)((dur & 0x03) << 5));
    if (err != ESP_OK) { ESP_LOGE(TAG, "WAKE_UP_DUR write failed"); return err; }

    /* Route wake-up to INT2. Must happen before INTERRUPTS_ENABLE, so
     * the event never has a window where it is armed but unrouted. */
    err = lsm6dsox_write_reg(dev, LSM6DSOX_MD2_CFG, 0x20);
    if (err != ESP_OK) { ESP_LOGE(TAG, "MD2_CFG write failed"); return err; }

    err = lsm6dsox_write_reg(dev, LSM6DSOX_TAP_CFG2, 0x80);
    if (err != ESP_OK) { ESP_LOGE(TAG, "TAP_CFG2 write failed"); return err; }

    ESP_LOGI(TAG, "liftoff detect armed: %.2f g (WK_THS=%d, 0.25 g/count), "
                  "%u samples sustained, latched on INT2",
             (double)(code * WAKE_THS_LSB_G), code, (unsigned)dur);
    return ESP_OK;
}

/* boot = true is the original power-on path, with its settling delays
 * and its log lines. boot = false is lsm6dsox_recover(): same register
 * writes, no 200 ms waits, quiet. */
static esp_err_t lsm6dsox_configure_sensor(lsm6dsox_handle_t dev, bool boot)
{
    uint8_t who_am_i = 0;
    esp_err_t err = lsm6dsox_read_regs(dev, LSM6DSOX_WHO_AM_I_REG, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to read WHO_AM_I");
        return err;
    }
    if (who_am_i != LSM6DSOX_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who_am_i, LSM6DSOX_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    if (boot) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (OK)", who_am_i);
    }

    lsm6dsox_write_reg(dev, LSM6DSOX_CTRL3_C, 0x01);

    /* Same pdMS_TO_TICKS(1)-rounds-to-zero trap as in
     * lsm6dsox_wait_for_sample(): without this clamp the loop spun 100
     * times with no delay at all, so "software reset complete" could be
     * logged before the chip had actually finished resetting. */
    TickType_t reset_poll = pdMS_TO_TICKS(1);
    if (reset_poll == 0) {
        reset_poll = 1;
    }
    uint8_t ctrl3 = 0x01;
    for (int i = 0; i < 100 && (ctrl3 & 0x01); i++) {
        vTaskDelay(reset_poll);
        lsm6dsox_read_regs(dev, LSM6DSOX_CTRL3_C, &ctrl3, 1);
    }
    if (ctrl3 & 0x01) {
        ESP_LOGE(TAG, "software reset timed out");
        return ESP_ERR_TIMEOUT;
    }
    if (boot) {
        ESP_LOGI(TAG, "software reset complete");
    }

    /* The reset put CTRL4_C back to defaults, which on an SPI build
     * re-enables the chip's I2C listener. sensor_bus turns it back off
     * (no-op on an I2C build). */
    err = sensor_bus_harden();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sensor_bus_harden after reset failed: %s", esp_err_to_name(err));
    }

    lsm6dsox_read_regs(dev, LSM6DSOX_CTRL3_C, &ctrl3, 1);
    lsm6dsox_write_reg(dev, LSM6DSOX_CTRL3_C, ctrl3 | 0x40);

    uint8_t ctrl9 = 0;
    lsm6dsox_read_regs(dev, LSM6DSOX_CTRL9_XL, &ctrl9, 1);
    lsm6dsox_write_reg(dev, LSM6DSOX_CTRL9_XL,
                       (uint8_t)((ctrl9 | LSM6DSOX_CTRL9_I3C_DISABLE) & ~0x01));

    err = lsm6dsox_write_reg(dev, LSM6DSOX_CTRL1_XL, LSM6DSOX_CTRL1_XL_VAL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "CTRL1_XL write failed"); return err; }
    if (boot) { vTaskDelay(pdMS_TO_TICKS(200)); }

    err = lsm6dsox_write_reg(dev, LSM6DSOX_CTRL2_G, LSM6DSOX_CTRL2_G_VAL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "CTRL2_G write failed"); return err; }
    if (boot) { vTaskDelay(pdMS_TO_TICKS(200)); }

    err = lsm6dsox_write_reg(dev, LSM6DSOX_INT1_CTRL, 0x03);
    if (err != ESP_OK) { ESP_LOGE(TAG, "INT1_CTRL write failed"); return err; }

    dev->fast = true;
    if (boot) {
        ESP_LOGI(TAG, "configured: accel 104Hz/+-16g, gyro 104Hz/+-2000dps, INT1=DRDY");
    }

    if (dev->cfg.int2_pin != GPIO_NUM_NC) {
        err = lsm6dsox_configure_wakeup(dev);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t lsm6dsox_setup_interrupt(lsm6dsox_handle_t dev)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << dev->cfg.int_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }

    /* INT1's handler is attached in lsm6dsox_start(), AFTER the reader
     * task exists -- lsm6dsox_isr() notifies dev->task, which is NULL
     * until then. INT2 (liftoff) stays here: its ISR only sets a flag. */

    if (dev->cfg.int2_pin != GPIO_NUM_NC) {
        gpio_config_t io2 = {
            .pin_bit_mask = 1ULL << dev->cfg.int2_pin,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_POSEDGE,
        };
        err = gpio_config(&io2);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "INT2 gpio_config failed: %s", esp_err_to_name(err));
            return err;
        }

        err = gpio_isr_handler_add(dev->cfg.int2_pin, lsm6dsox_liftoff_isr, (void *)dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "INT2 gpio_isr_handler_add failed: %s", esp_err_to_name(err));
            return err;
        }

        dev->int2_isr_installed = true;
        ESP_LOGI(TAG, "INT2 armed on GPIO%d (liftoff, rising edge)",
                 (int)dev->cfg.int2_pin);
    }
    return ESP_OK;
}

/* ===================================================================
 *  Public API
 * =================================================================== */

esp_err_t lsm6dsox_init(const lsm6dsox_config_t *cfg, lsm6dsox_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->int_pin < 0) {
        ESP_LOGE(TAG, "need a real INT pin for interrupt mode");
        return ESP_ERR_INVALID_ARG;
    }

    struct lsm6dsox_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < LSM6DSOX_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", LSM6DSOX_MAX_INSTANCES);
        return ESP_ERR_NO_MEM;
    }

    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    dev->latest_lock = tmp;

    dev->cfg = *cfg;   /* shallow copy -- cfg can die after this call returns */

    /* No bus setup here: sensor_bus_init() already did it, for whichever
     * bus this build uses. */
    /* Same rule as the barometer: a chip that does not answer at boot
     * starts in the LOST state and is retried, rather than stopping boot
     * before the radio can say so. */
    bool present = (lsm6dsox_configure_sensor(dev, true) == ESP_OK);

    esp_err_t err = lsm6dsox_setup_interrupt(dev);
    if (err != ESP_OK) {
        dev->in_use = false;
        return err;     /* a GPIO problem: that one IS fatal */
    }

    uint64_t now_us = (uint64_t)esp_timer_get_time();
    sensor_health_init(&dev->health, ERR_SRC_LSM6DSOX, now_us);

    dev->initialized = true;
    *out = dev;
    if (!present) {
        sensor_health_lost(&dev->health, SENSOR_LOST_NO_RESPONSE, now_us);
        ESP_LOGE(TAG, "LSM6DSOX NOT ANSWERING -- driver started anyway, retrying "
                      "every %d ms", SENSOR_HEALTH_RETRY_MS);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "LSM6DSOX initialized (%s, interrupt-driven)", sensor_bus_name());
    return ESP_OK;
}

esp_err_t lsm6dsox_start(lsm6dsox_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->task) {
        ESP_LOGW(TAG, "task already running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->task = xTaskCreateStaticPinnedToCore(
        lsm6dsox_task, "lsm6dsox", LSM6DSOX_STACK_WORDS,
        (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        return ESP_FAIL;
    }

    /* Task before ISR handler, not after -- same rule as discrete_in. */
    esp_err_t err = gpio_isr_handler_add(dev->cfg.int_pin, lsm6dsox_isr, (void *)dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        vTaskDelete(dev->task);
        dev->task = NULL;
        return err;
    }
    dev->isr_installed = true;
    ESP_LOGI(TAG, "INT1 armed on GPIO%d (rising edge)", (int)dev->cfg.int_pin);

    return ESP_OK;
}

esp_err_t lsm6dsox_stop(lsm6dsox_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->isr_installed) {
        gpio_isr_handler_remove(dev->cfg.int_pin);
        dev->isr_installed = false;
    }
    if (dev->int2_isr_installed) {
        gpio_isr_handler_remove(dev->cfg.int2_pin);
        dev->int2_isr_installed = false;
    }
    if (dev->task) {
        vTaskDelete(dev->task);
        dev->task = NULL;
    }
    dev->initialized = false;
    dev->in_use      = false;
    return ESP_OK;
}

void lsm6dsox_get_latest(lsm6dsox_handle_t dev, lsm6_data_t *out)
{
    if (!dev || !out) {
        return;
    }
    portENTER_CRITICAL(&dev->latest_lock);
    *out = dev->latest;
    portEXIT_CRITICAL(&dev->latest_lock);
}

esp_err_t lsm6dsox_wait_for_sample(lsm6dsox_handle_t dev, uint32_t last_seq,
                                   lsm6_data_t *out, TickType_t timeout)
{
    if (!dev || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    const TickType_t start = xTaskGetTickCount();
    /* At least one tick, always. pdMS_TO_TICKS(1) is integer division by
     * the tick rate, so it rounds DOWN to ZERO at any tick rate below
     * 1000 Hz -- and vTaskDelay(0) does not block. It yields, and a yield
     * only runs tasks of equal or higher priority, never the idle task.
     * The result is this core pegged at 100% and the task watchdog
     * tripping on IDLE. Clamping here means no sdkconfig change can bring
     * that back. */
    TickType_t poll = pdMS_TO_TICKS(1);
    if (poll == 0) {
        poll = 1;
    }

    for (;;) {
        lsm6dsox_get_latest(dev, out);
        if (out->seq != last_seq) {
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(poll);
    }
}

esp_err_t lsm6dsox_read_once(lsm6dsox_handle_t dev, lsm6_data_t *out)
{
    if (!dev || !out || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t gyro_raw[6], accel_raw[6];
    esp_err_t err = lsm6dsox_read_regs(dev, LSM6DSOX_OUTX_L_G, gyro_raw, 6);
    if (err != ESP_OK) return err;
    err = lsm6dsox_read_regs(dev, LSM6DSOX_OUTX_L_A, accel_raw, 6);
    if (err != ESP_OK) return err;

    int16_t gx = (int16_t)((gyro_raw[1] << 8) | gyro_raw[0]);
    int16_t gy = (int16_t)((gyro_raw[3] << 8) | gyro_raw[2]);
    int16_t gz = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]);
    int16_t ax = (int16_t)((accel_raw[1] << 8) | accel_raw[0]);
    int16_t ay = (int16_t)((accel_raw[3] << 8) | accel_raw[2]);
    int16_t az = (int16_t)((accel_raw[5] << 8) | accel_raw[4]);

    out->ax = ax * ACCEL_SCALE;
    out->ay = ay * ACCEL_SCALE;
    out->az = az * ACCEL_SCALE;
    out->gx = gx * GYRO_SCALE;
    out->gy = gy * GYRO_SCALE;
    out->gz = gz * GYRO_SCALE;
    out->timestamp_us = (uint64_t)esp_timer_get_time();
    out->seq = 0;

    return ESP_OK;
}

void lsm6dsox_get_diagnostics(lsm6dsox_handle_t dev, lsm6dsox_diag_t *out)
{
    if (!dev || !out) {
        return;
    }
    *out = dev->diag;
    /* Mirrored out of the ISR-owned counter rather than incremented in
     * the ISR itself, so the interrupt path stays a timestamp and two
     * stores. */
    out->liftoff_events = dev->liftoff_count;
    /* Same unlocked-copy reasoning as bmp388_get_diagnostics(). */
    out->lost           = dev->health.lost;
    out->lost_count     = dev->health.lost_count;
    out->recoveries     = dev->health.recoveries;
    out->retry_attempts = dev->health.retry_attempts;
    out->last_outage_ms = dev->health.last_outage_ms;
}

/* ===================================================================
 *  Hardware liftoff accessors
 * =================================================================== */

bool lsm6dsox_liftoff_detected(lsm6dsox_handle_t dev)
{
    return dev ? dev->liftoff_detected : false;
}

/* ===================================================================
 *  Rate control -- standby vs flight
 * =================================================================== */

esp_err_t lsm6dsox_set_rate(lsm6dsox_handle_t dev, bool fast)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->fast == fast) {
        return ESP_OK;
    }

    /* Accelerometer first, gyro second, in this order both ways. Going
     * to standby that means the accel is already slow before the gyro
     * stops; going to flight it means the accel is fast before the gyro
     * starts. Either way the wake-up block -- which runs off the accel
     * alone -- is never left looking at a rate nobody configured. */
    esp_err_t err = lsm6dsox_write_reg(dev, LSM6DSOX_CTRL1_XL,
                        fast ? LSM6DSOX_CTRL1_XL_VAL : LSM6DSOX_CTRL1_XL_SLOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CTRL1_XL rate write failed");
        event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_CONFIG_INVALID, err, (uint32_t)fast);
        return err;
    }

    err = lsm6dsox_write_reg(dev, LSM6DSOX_CTRL2_G,
                        fast ? LSM6DSOX_CTRL2_G_VAL : LSM6DSOX_CTRL2_G_SLOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CTRL2_G rate write failed");
        event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_CONFIG_INVALID, err, (uint32_t)fast);
        return err;
    }

    /* WAKE_UP_DUR counts ODR SAMPLES, so one register value means a
     * different wall-clock time at each rate. At 104 Hz, 2 samples is
     * ~19 ms of noise rejection. At 12.5 Hz the same 2 samples would be
     * 160 ms -- and every millisecond of that is boost being recorded at
     * 12.5 Hz instead of 104. So standby uses 0: fire on the first
     * sample over threshold.
     *
     * That trades noise rejection for latency, which is the right way
     * round here. The threshold is 3 g, far above anything handling the
     * airframe produces, and a false trigger only moves the vehicle into
     * flight mode early -- the safe direction. */
    uint8_t dur = fast ? (uint8_t)((LSM6DSOX_LIFTOFF_DUR_SAMPLES & 0x03) << 5) : 0x00;
    err = lsm6dsox_write_reg(dev, LSM6DSOX_WAKE_UP_DUR, dur);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WAKE_UP_DUR rate write failed");
        event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_CONFIG_INVALID, err, (uint32_t)fast);
        return err;
    }

    dev->fast = fast;

    /* Read both rate registers back. A write the chip silently ignored
     * would otherwise look exactly like success: the accelerometer would
     * change rate and the gyro would not, or the other way round. */
    uint8_t xl = 0xFF, g = 0xFF;
    esp_err_t rb = lsm6dsox_read_regs(dev, LSM6DSOX_CTRL1_XL, &xl, 1);
    if (rb == ESP_OK) {
        rb = lsm6dsox_read_regs(dev, LSM6DSOX_CTRL2_G, &g, 1);
    }
    const uint8_t want_xl = fast ? LSM6DSOX_CTRL1_XL_VAL : LSM6DSOX_CTRL1_XL_SLOW;
    const uint8_t want_g  = fast ? LSM6DSOX_CTRL2_G_VAL  : LSM6DSOX_CTRL2_G_SLOW;
    ESP_LOGW(TAG, "rate -> %s (accel %s, gyro %s)  read back CTRL1_XL=0x%02X CTRL2_G=0x%02X",
             fast ? "FLIGHT" : "STANDBY",
             fast ? "104 Hz" : "12.5 Hz",
             fast ? "104 Hz" : "POWERED DOWN", xl, g);
    if (rb != ESP_OK || xl != want_xl || g != want_g) {
        ESP_LOGE(TAG, "rate registers did NOT take: wanted CTRL1_XL=0x%02X CTRL2_G=0x%02X, "
                      "chip says 0x%02X 0x%02X (%s)", want_xl, want_g, xl, g,
                 esp_err_to_name(rb));
        event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_CONFIG_INVALID, ESP_ERR_INVALID_STATE,
                        ((uint32_t)xl << 8) | g);
    }
    dev->gyro_check_left = fast ? 3 : 0;
    return ESP_OK;
}

bool lsm6dsox_is_fast(lsm6dsox_handle_t dev)
{
    return dev ? dev->fast : true;
}

void lsm6dsox_get_liftoff(lsm6dsox_handle_t dev, lsm6dsox_liftoff_t *out)
{
    if (!dev || !out) {
        return;
    }
    /* No lock. Each field is a single aligned word written once by the
     * ISR and never modified afterwards, so the worst a concurrent read
     * can see is the previous consistent value -- not a torn one. */
    out->detected     = dev->liftoff_detected;
    out->timestamp_us = dev->liftoff_timestamp_us;
    out->count        = dev->liftoff_count;
}
