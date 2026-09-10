#include "lsm6dsox_sensor.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "data_router.h"
#include "event_log.h"
#include "flight_mode.h"

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
#define LSM6DSOX_INT1_CTRL      0x0D    // Routes DRDY_XL / DRDY_G to INT1
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

/* SPI framing, LSM6DSOX convention:
 *   bit7   = R/W  (1 = read, 0 = write)
 *   bit6:0 = register address -- all seven bits
 *
 * There is NO MS/auto-increment bit on this part. That bit exists on the
 * older ST parts (LIS3DH, L3GD20) and setting it here would land bit 6
 * inside the address: a 6-byte read of OUTX_L_G (0x22) would go to 0x62
 * instead. Multi-byte transfers auto-increment because IF_INC in CTRL3_C
 * is set, which is the reset default and is preserved below.
 *
 * No dummy byte here -- that was BMP388-specific. */
#define LSM6DSOX_SPI_READ_BIT   0x80

/* ===================================================================
 *  The real struct -- private to this file, same idiom as bmp388_dev.
 *  Everything that used to be scattered file-scope statics now lives
 *  in one place, one per instance.
 * =================================================================== */

struct lsm6dsox_dev {
    bool                in_use;
    lsm6dsox_config_t   cfg;

    spi_device_handle_t spi;
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
    volatile bool       liftoff_detected;
    volatile uint64_t   liftoff_timestamp_us;
    volatile uint32_t   liftoff_count;
    bool                int2_isr_installed;

    lsm6dsox_diag_t     diag;
};

/* ===================================================================
 *  Static instance pool
 * =================================================================== */

static struct lsm6dsox_dev s_pool[LSM6DSOX_MAX_INSTANCES];
static portMUX_TYPE        s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* ===================================================================
 *  SPI primitives -- now take dev explicitly instead of reaching for a
 *  fixed global SPI handle.
 * =================================================================== */

static esp_err_t lsm6dsox_read_regs(lsm6dsox_handle_t dev, uint8_t reg,
                                    uint8_t *dst, size_t len)
{
    if (!dev || !dst || len == 0 || len > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t addr = reg | LSM6DSOX_SPI_READ_BIT;

    uint8_t tx[33] = {0};
    uint8_t rx[33] = {0};
    tx[0] = addr;

    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .rxlength  = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI read 0x%02X failed: %s", reg, esp_err_to_name(err));
        return err;
    }

    /* No dummy byte to skip here -- rx[0] is the only garbage byte. */
    memcpy(dst, &rx[1], len);
    return ESP_OK;
}

static esp_err_t lsm6dsox_write_reg(lsm6dsox_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
    };

    esp_err_t err = spi_device_polling_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI write 0x%02X failed: %s", reg, esp_err_to_name(err));
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
 *  Reader task
 * =================================================================== */

static void lsm6dsox_task(void *arg)
{
    lsm6dsox_handle_t dev = (lsm6dsox_handle_t)arg;
    const TickType_t timeout = pdMS_TO_TICKS(LSM6DSOX_IRQ_TIMEOUT_MS);

    ESP_LOGI(TAG, "reader task running on core %d", (int)xPortGetCoreID());

    for (;;) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, timeout);

        /* Reconfiguration happens HERE, in the task that owns the SPI
         * device -- never from whatever task changed the mode. The
         * telemetry task accepting an ARM command must not reach into
         * this sensor's bus. Polling costs one comparison per sample. */
        /* The hardware liftoff latch is the backup path into FLIGHT --
         * it is what makes the vehicle switch to flight rates even if
         * the radio never worked. Called from the task, not the ISR:
         * flight_mode_note_liftoff() ends in a ring-buffer push, which
         * is not ISR-safe. It is idempotent and the latch is level-
         * triggered, so calling it every pass is the intended usage. */
        if (dev->liftoff_detected) {
            flight_mode_note_liftoff(MODE_SRC_IMU_WAKEUP);
        }

        bool want_fast = flight_mode_is_fast();
        if (want_fast != dev->fast) {
            lsm6dsox_set_rate(dev, want_fast);
        }

        uint64_t stamp;
        if (notified == 0) {
            dev->diag.irq_timeouts++;
            event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_IRQ_TIMEOUT, ESP_ERR_TIMEOUT, dev->diag.irq_timeouts);
            stamp = (uint64_t)esp_timer_get_time();
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
            continue;
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
 * once and then the line stays high. No SPI here -- reading WAKE_UP_SRC
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

static esp_err_t lsm6dsox_configure_sensor(lsm6dsox_handle_t dev)
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
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (OK)", who_am_i);

    lsm6dsox_write_reg(dev, LSM6DSOX_CTRL3_C, 0x01);

    /* pdMS_TO_TICKS(1) is 0 at CONFIG_FREERTOS_HZ=100, which made this a
     * yield-only loop -- it happened to work because 100 SPI reads take
     * far longer than the ~50 us the reset needs, but it was not the
     * bounded wait it looks like. */
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
    ESP_LOGI(TAG, "software reset complete");

    lsm6dsox_read_regs(dev, LSM6DSOX_CTRL3_C, &ctrl3, 1);
    lsm6dsox_write_reg(dev, LSM6DSOX_CTRL3_C, ctrl3 | 0x40);

    /* CTRL9_XL bit 1 is I3C_disable; bit 0 is reserved. Writing 0x01 set
     * the reserved bit and left the I3C block enabled -- with SPI wired
     * up, an I3C reset pattern on the shared pins can then knock the
     * digital interface over mid-flight. */
    uint8_t ctrl9 = 0;
    lsm6dsox_read_regs(dev, LSM6DSOX_CTRL9_XL, &ctrl9, 1);
    lsm6dsox_write_reg(dev, LSM6DSOX_CTRL9_XL, ctrl9 | 0x02);

    err = lsm6dsox_write_reg(dev, LSM6DSOX_CTRL1_XL, LSM6DSOX_CTRL1_XL_VAL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "CTRL1_XL write failed"); return err; }
    vTaskDelay(pdMS_TO_TICKS(200));

    err = lsm6dsox_write_reg(dev, LSM6DSOX_CTRL2_G, LSM6DSOX_CTRL2_G_VAL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "CTRL2_G write failed"); return err; }
    vTaskDelay(pdMS_TO_TICKS(200));

    err = lsm6dsox_write_reg(dev, LSM6DSOX_INT1_CTRL, 0x03);
    if (err != ESP_OK) { ESP_LOGE(TAG, "INT1_CTRL write failed"); return err; }

    dev->fast = true;
    ESP_LOGI(TAG, "configured: accel 104Hz/+-16g, gyro 104Hz/+-2000dps, INT1=DRDY");

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

    err = gpio_isr_handler_add(dev->cfg.int_pin, lsm6dsox_isr, (void *)dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return err;
    }

    dev->isr_installed = true;
    ESP_LOGI(TAG, "INT1 armed on GPIO%d (rising edge)", (int)dev->cfg.int_pin);

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

    /* Hand the CS pad back to the HP digital path before the SPI driver
     * takes it. On the ESP32-P4, GPIO0-15 are LP_IO pads (SOC_RTCIO_PIN_COUNT
     * is 16) and spicommon_cs_initialize() -- unlike gpio_config() -- never
     * calls rtc_gpio_deinit(). A CS assigned down there can end up never
     * driven, which leaves the part sitting in I2C mode: it answers nothing
     * on SPI and its SDO stays a high-Z address-select input. Costs nothing
     * on a pad that was already digital. */
    gpio_reset_pin(cfg->cs_pin);

    /* Mode 3 (CPOL=1, CPHA=1) is what the LSM6DSOX datasheet specifies.
     * Mode 0 also worked in practice -- both modes sample on the rising
     * edge and shift on the falling one, so they differ only in the idle
     * level of SCLK between transactions -- but there is no reason to
     * run off-datasheet. The BMP388 stays on mode 0 (it accepts 0 or 3);
     * the driver reprograms the mode per device, so the two sharing this
     * bus is fine. */
    spi_device_interface_config_t devcfg = {
        .mode           = 3,
        .clock_speed_hz = cfg->spi_freq_hz,
        .spics_io_num   = cfg->cs_pin,
        .queue_size     = 1,
    };

    esp_err_t err = spi_bus_add_device(cfg->host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        dev->in_use = false;
        return err;
    }

    err = lsm6dsox_configure_sensor(dev);
    if (err != ESP_OK) {
        spi_bus_remove_device(dev->spi);
        dev->in_use = false;
        return err;
    }

    err = lsm6dsox_setup_interrupt(dev);
    if (err != ESP_OK) {
        spi_bus_remove_device(dev->spi);
        dev->in_use = false;
        return err;
    }

    dev->initialized = true;
    *out = dev;
    ESP_LOGI(TAG, "LSM6DSOX initialized (SPI, interrupt-driven)");
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
    if (dev->spi) {
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
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
    /* See bmp388_wait_for_sample(): at 100 Hz tick, pdMS_TO_TICKS(1) is
     * 0 and vTaskDelay(0) only yields, turning this into a busy-spin. */
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
    ESP_LOGW(TAG, "rate -> %s (accel %s, gyro %s)",
             fast ? "FLIGHT" : "STANDBY",
             fast ? "104 Hz" : "12.5 Hz",
             fast ? "104 Hz" : "POWERED DOWN");
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
