/*
 * sensor_bus.c -- SPI or I2C for the BMP388 + LSM6DSOX, chosen in menuconfig.
 * See sensor_bus.h for the wiring table and the rules.
 */
#include "sensor_bus.h"

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if CONFIG_SET_SENSOR_BUS_I2C
#include "driver/i2c_master.h"
#define SB_IS_SPI 0
#else
#include "driver/spi_master.h"
#define SB_IS_SPI 1
#endif

static const char *TAG = "sensor_bus";

/* ---- Pins: claude/PCB_v1_wiring.md. Board facts, not settings. ---------- */
#define PIN_SCK_SCL    54   /* J1.1 -> J3.4 SCL, J4.4 SCL                   */
#define PIN_MOSI_SDA   19   /* J1.2 -> J3.6 SDA, J4.5 SDA                   */
#define PIN_MISO_ADDR  18   /* J1.4 -> J3.5 SDO, J4.6 DO                    */
#define PIN_CS_BMP     14   /* J1.9 -> J3.7 CS                              */
#define PIN_CS_LSM     17   /* J1.5 -> J4.7 CS                              */

/* ---- Register facts from the two datasheets ----------------------------- */
#define BMP_REG_CHIP_ID       0x00
#define BMP388_CHIP_ID        0x50
#define BMP390_CHIP_ID        0x60
#define LSM_REG_WHO_AM_I      0x0F
#define LSM_WHO_AM_I_VALUE    0x6C
#define LSM_REG_CTRL4_C       0x13
#define LSM_CTRL4_I2C_DISABLE 0x04
#define SPI_READ_BIT          0x80

/* ---- I2C addresses --------------------------------------------------------
 * In I2C mode GPIO18 stops being a data line: it becomes the address-select
 * input of BOTH sensors at once (BMP388 "SDO", LSM6DSOX "SA0"). The two
 * breakouts may pull that shared wire in opposite directions, so we drive it
 * ourselves and the addresses are no longer a guess:  LOW -> 0x76 and 0x6A.
 */
#define I2C_ADDR_PIN_LEVEL  0
#define BMP_I2C_ADDR        0x76   /* would be 0x77 if GPIO18 were HIGH */
#define BMP_I2C_ADDR_ALT    0x77
#define LSM_I2C_ADDR        0x6A   /* would be 0x6B if GPIO18 were HIGH */
#define LSM_I2C_ADDR_ALT    0x6B
#define I2C_XFER_TIMEOUT_MS 10

/* ---- Shared state --------------------------------------------------------
 * The baro task and the IMU task both use this bus. One mutex makes each
 * transfer finish before the next one starts, on either bus type.
 */
#define LOCK_WAIT pdMS_TO_TICKS(20)

/* Failure escalation. A single NACK is not interesting -- a sensor can miss
 * one and the next read is fine. A RUN of them means the bus is stuck, and
 * stuck buses do not unstick themselves. See bus_note_result(). */
#define RECOVER_AFTER_ERRORS    3    /* consecutive failures before acting   */
#define RECOVER_MIN_INTERVAL_MS 250  /* floor between two recovery attempts  */

static uint32_t           s_consec_errors;
static uint32_t           s_last_recover_ms;
static bool               s_recover_seen;

static SemaphoreHandle_t  s_lock;
static StaticSemaphore_t  s_lock_buf;
static bool               s_ready;
static sensor_bus_diag_t  s_diag;
static portMUX_TYPE       s_diag_mux = portMUX_INITIALIZER_UNLOCKED;

#define DIAG_INC(field) do { portENTER_CRITICAL(&s_diag_mux); \
                             s_diag.field++;                   \
                             portEXIT_CRITICAL(&s_diag_mux); } while (0)

/* Both CS lines HIGH before anything else touches the bus. In the I2C build
 * this is what keeps both chips in I2C mode; in the SPI build it is simply
 * "nobody selected" until the SPI driver takes the pins over. The level is
 * set before AND after switching to output so the pin never glitches low. */
static esp_err_t cs_lines_high(void)
{
    gpio_set_level(PIN_CS_BMP, 1);
    gpio_set_level(PIN_CS_LSM, 1);
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_CS_BMP) | (1ULL << PIN_CS_LSM),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    gpio_set_level(PIN_CS_BMP, 1);
    gpio_set_level(PIN_CS_LSM, 1);
    return err;
}

/* ========================================================================= */
#if SB_IS_SPI
/* ========================================================================= */

/* -------------------------------------------------------------------------
 *  SPI IS NOT IMPLEMENTED YET.
 *
 *  These four stubs exist so the common code below (locked_read,
 *  sensor_bus_write, sensor_bus_init) still compiles and links in an SPI
 *  build. They touch no hardware and fail cleanly with
 *  ESP_ERR_NOT_SUPPORTED rather than pretending a transfer happened.
 *
 *  The previous transfer code is preserved verbatim in the #if 0 block at
 *  the end of this section. It has never been confirmed working on the
 *  board -- both chips read back 0x00 while every transaction reported
 *  ESP_OK -- which is why it is parked rather than trusted. Whoever
 *  finishes SPI should start from it, not from scratch.
 * ------------------------------------------------------------------------- */

static esp_err_t bus_open(void)
{
    ESP_LOGE(TAG, "SPI sensor bus is not implemented -- run menuconfig and "
                  "choose I2C (SET sensor bus -> Interface), or set "
                  "CONFIG_SET_SENSOR_BUS_I2C=y.");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t bus_read(sensor_bus_dev_t dev, uint8_t addr_byte,
                          size_t skip, uint8_t *buf, size_t len)
{
    (void)dev; (void)addr_byte; (void)skip; (void)buf; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t bus_write(sensor_bus_dev_t dev, uint8_t reg,
                           const uint8_t *buf, size_t len)
{
    (void)dev; (void)reg; (void)buf; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t bus_recover(void)
{
    return ESP_ERR_NOT_SUPPORTED;   /* nothing was ever opened */
}

static esp_err_t bus_reopen(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

/* -------------------------------------------------------------------------
 *  Parked reference implementation -- NOT COMPILED.
 *  Delete this block once the stubs above are replaced for real.
 * ------------------------------------------------------------------------- */
#if 0
static spi_device_handle_t s_spi[SENSOR_BUS_DEV_COUNT];

static esp_err_t bus_open(void)
{
    const spi_bus_config_t bus = {
        .sclk_io_num     = PIN_SCK_SCL,
        .mosi_io_num     = PIN_MOSI_SDA,
        .miso_io_num     = PIN_MISO_ADDR,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = SENSOR_BUS_MAX_XFER,
    };
    /* No DMA. Every transfer here fits in the SPI peripheral's own 64-byte
     * buffer, and without DMA there is no CPU cache that can disagree with
     * RAM about what was received -- one ESP32-P4 pitfall removed. */
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    /* Mode 0: the clock idles LOW and the data line only changes while the
     * clock is low. An I2C listener treats "data changes while clock is HIGH"
     * as start/stop, so in mode 0 a sensor that is still listening for I2C
     * never sees a start condition in the other sensor's SPI traffic. */
    spi_device_interface_config_t dev = {
        .mode           = 0,
        .clock_speed_hz = CONFIG_SET_SENSOR_SPI_FREQ_HZ,
        .queue_size     = 1,
    };
    dev.spics_io_num = PIN_CS_BMP;
    err = spi_bus_add_device(SPI2_HOST, &dev, &s_spi[SENSOR_BUS_BMP388]);
    if (err == ESP_OK) {
        dev.spics_io_num = PIN_CS_LSM;
        err = spi_bus_add_device(SPI2_HOST, &dev, &s_spi[SENSOR_BUS_LSM6DSOX]);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
    }
    return err;
}

/* One full-duplex SPI transaction of `n` bytes. Every byte clocked out is
 * also a byte clocked in, so tx and rx are the same size, and the address
 * byte is part of the count. `length` is in BITS. */
static esp_err_t bus_read(sensor_bus_dev_t dev, uint8_t addr_byte,
                          size_t skip, uint8_t *buf, size_t len)
{
    const size_t n = 1 + skip + len;
    uint8_t tx[SENSOR_BUS_MAX_XFER];
    uint8_t rx[SENSOR_BUS_MAX_XFER];
    memset(tx, 0, n);
    memset(rx, 0, n);
    tx[0] = addr_byte;

    spi_transaction_t t = {
        .length    = n * 8,
        .rxlength  = n * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi[dev], &t);
    if (err == ESP_OK) {
        /* rx[0] arrived while the address was going out -- meaningless.
         * Then `skip` bytes to discard (the BMP388's dummy byte), then data. */
        memcpy(buf, &rx[1 + skip], len);
    }
    return err;
}

static esp_err_t bus_write(sensor_bus_dev_t dev, uint8_t reg,
                           const uint8_t *buf, size_t len)
{
    const size_t n = 1 + len;
    uint8_t tx[SENSOR_BUS_MAX_XFER];
    uint8_t rx[SENSOR_BUS_MAX_XFER];
    tx[0] = reg;
    memcpy(&tx[1], buf, len);

    spi_transaction_t t = {
        .length    = n * 8,
        .rxlength  = n * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(s_spi[dev], &t);
}

static void bus_recover(void) { /* SPI: the next CS edge resynchronises */ }
#endif  /* parked SPI reference implementation */

/* ========================================================================= */
#else  /* I2C */
/* ========================================================================= */

static const char *dev_name(sensor_bus_dev_t dev)
{
    return dev == SENSOR_BUS_BMP388 ? "BMP388" : "LSM6DSOX";
}

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c[SENSOR_BUS_DEV_COUNT];
static bool                    s_scanned;

static bool i2c_answers(uint8_t addr)
{
    return i2c_master_probe(s_i2c_bus, addr, I2C_XFER_TIMEOUT_MS) == ESP_OK;
}

/* Log every address that ACKs. If this prints nothing, the problem is
 * wiring or power, not firmware. */
static void i2c_scan_log(void)
{
    char line[128];
    size_t used = 0;
    int found = 0;
    line[0] = '\0';
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_answers(a)) {
            found++;
            if (used < sizeof line - 6) {
                used += (size_t)snprintf(&line[used], sizeof line - used,
                                         " 0x%02X", a);
            }
        }
    }
    if (found) {
        ESP_LOGI(TAG, "I2C scan: %d device(s):%s", found, line);
    } else {
        ESP_LOGE(TAG, "I2C scan: nothing answered. Check 3V3 at J3/J4, and "
                      "that nothing else is driving GPIO54/GPIO19.");
    }
}

/* Use the expected address; if only the other one answers, say so loudly
 * and use that. If neither answers, keep the expected one so the chip-ID
 * check fails with a clear message instead of init failing silently. */
static uint8_t pick_addr(sensor_bus_dev_t dev, uint8_t want, uint8_t alt)
{
    if (i2c_answers(want)) {
        return want;
    }
    if (i2c_answers(alt)) {
        ESP_LOGW(TAG, "%s answered at 0x%02X, not 0x%02X -- GPIO18 is not "
                      "reaching its address pin at the level we drive.",
                 dev_name(dev), alt, want);
        return alt;
    }
    ESP_LOGE(TAG, "%s: nothing at 0x%02X or 0x%02X%s", dev_name(dev), want, alt,
             dev == SENSOR_BUS_BMP388
                 ? " -- if an SPI build ran since power-on, the BMP388 is still"
                   " locked in SPI: remove ALL power and retry" : "");
    return want;
}

static esp_err_t bus_open(void)
{
    /* GPIO18: address select for both sensors. Level first, then output. */
    gpio_set_level(PIN_MISO_ADDR, I2C_ADDR_PIN_LEVEL);
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_MISO_ADDR,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    gpio_set_level(PIN_MISO_ADDR, I2C_ADDR_PIN_LEVEL);
    if (err != ESP_OK) {
        return err;
    }

    const i2c_master_bus_config_t bus = {
        .i2c_port          = I2C_NUM_0,       /* fixed port, not -1 */
        .sda_io_num        = PIN_MOSI_SDA,
        .scl_io_num        = PIN_SCK_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* The breakouts carry their own pull-ups; the internal ones are weak
         * (~45k) and only a backup. */
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&bus, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    /* Scan once at boot only. bus_reopen() calls this same function during
     * fault recovery and a 112-address scan there would hold the bus lock
     * for the better part of a second while the sensor tasks starve. */
    if (!s_scanned) {
        s_scanned = true;
        i2c_scan_log();
    }

    const uint8_t addr[SENSOR_BUS_DEV_COUNT] = {
        [SENSOR_BUS_BMP388]   = pick_addr(SENSOR_BUS_BMP388,
                                          BMP_I2C_ADDR, BMP_I2C_ADDR_ALT),
        [SENSOR_BUS_LSM6DSOX] = pick_addr(SENSOR_BUS_LSM6DSOX,
                                          LSM_I2C_ADDR, LSM_I2C_ADDR_ALT),
    };
    for (int d = 0; d < SENSOR_BUS_DEV_COUNT; d++) {
        const i2c_device_config_t dc = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = addr[d],
            .scl_speed_hz    = CONFIG_SET_SENSOR_I2C_FREQ_HZ,
        };
        err = i2c_master_bus_add_device(s_i2c_bus, &dc, &s_i2c[d]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add %s: %s", dev_name(d), esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "%s at 0x%02X", dev_name(d), addr[d]);
    }
    return ESP_OK;
}

/* Write the register address, repeated start, read `len` bytes. I2C has no
 * read bit in the register byte and no dummy byte, so `skip` is always 0. */
static esp_err_t bus_read(sensor_bus_dev_t dev, uint8_t addr_byte,
                          size_t skip, uint8_t *buf, size_t len)
{
    (void)skip;
    return i2c_master_transmit_receive(s_i2c[dev], &addr_byte, 1, buf, len,
                                       I2C_XFER_TIMEOUT_MS);
}

static esp_err_t bus_write(sensor_bus_dev_t dev, uint8_t reg,
                           const uint8_t *buf, size_t len)
{
    uint8_t tx[SENSOR_BUS_MAX_XFER];
    tx[0] = reg;
    memcpy(&tx[1], buf, len);
    return i2c_master_transmit(s_i2c[dev], tx, 1 + len, I2C_XFER_TIMEOUT_MS);
}

/* A sensor reset mid-byte can leave SDA held low and the whole bus stuck.
 * The driver's reset clocks SCL until the sensor lets go.
 *
 * This FAILS ("clear bus failed" in the log) when the slave keeps holding
 * SDA through all nine clocks. When it does, the ESP-IDF driver's own state
 * machine stays wedged and every later transfer returns
 * ESP_ERR_INVALID_STATE -- which is why the caller escalates to
 * bus_reopen() rather than giving up here. */
static esp_err_t bus_recover(void)
{
    esp_err_t err = i2c_master_bus_reset(s_i2c_bus);
    if (err == ESP_OK) {
        DIAG_INC(i2c_resets);
    }
    return err;
}

/* Throw the whole bus away and build it again: device handles, master bus,
 * GPIO18's address level, the lot. This is the only thing that clears an
 * ESP_ERR_INVALID_STATE out of the ESP-IDF driver -- i2c_master_bus_reset()
 * does not, because it leaves the same bus object in place.
 *
 * Returns ESP_OK only if the rebuilt bus is usable. If this keeps failing,
 * the fault is off-chip: power, pull-ups, or a sensor still clamping SDA.
 * No amount of firmware will clear those. */
static esp_err_t bus_reopen(void)
{
    for (int d = 0; d < SENSOR_BUS_DEV_COUNT; d++) {
        if (s_i2c[d] != NULL) {
            (void)i2c_master_bus_rm_device(s_i2c[d]);
            s_i2c[d] = NULL;
        }
    }
    if (s_i2c_bus != NULL) {
        (void)i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    esp_err_t err = bus_open();
    if (err == ESP_OK) {
        DIAG_INC(i2c_resets);
    }
    return err;
}

#endif /* SB_IS_SPI */

/* ========================================================================= */
/* Common: argument checks, locking, counting                                */
/* ========================================================================= */

/*
 * Called with the lock held, after EVERY transfer. This is the whole
 * fault-handling policy for the bus, in one place.
 *
 * What this replaced, and why: the old code recovered only when a transfer
 * returned ESP_ERR_TIMEOUT, and recovered only by calling
 * i2c_master_bus_reset(). That is a one-way trip. When a slave is still
 * clamping SDA, the reset's nine clock pulses do not free it, the reset
 * returns an error ("clear bus failed"), the driver's state machine stays
 * wedged, and every transfer from then on returns ESP_ERR_INVALID_STATE --
 * a code the old policy did not treat as recoverable at all. One transient
 * glitch therefore bricked both sensors until the next reboot.
 *
 * The policy now:
 *
 *   - a lone failure is counted and ignored; a sensor is allowed to miss one
 *   - RECOVER_AFTER_ERRORS in a row -> clock the bus free (bus_recover)
 *   - if that fails -> tear the bus down and rebuild it (bus_reopen)
 *   - any success resets the count
 *   - attempts are rate limited to one per RECOVER_MIN_INTERVAL_MS, so a
 *     genuinely dead bus degrades into slow retries instead of starving the
 *     sensor tasks behind a lock held through probe timeouts
 *
 * Recovering does NOT mean the underlying fault is fixed. It means the
 * firmware stays able to talk to a sensor that comes back.
 */
static void bus_note_result(esp_err_t err)
{
    if (err == ESP_OK) {
        s_consec_errors = 0;
        return;
    }

    DIAG_INC(errors);
    if (++s_consec_errors < RECOVER_AFTER_ERRORS) {
        return;
    }

#if SB_IS_SPI
    return;   /* the SPI branch is stubs -- there is nothing to recover */
#else
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (s_recover_seen &&
        (uint32_t)(now_ms - s_last_recover_ms) < RECOVER_MIN_INTERVAL_MS) {
        return;
    }
    s_recover_seen    = true;
    s_last_recover_ms = now_ms;

    if (bus_recover() == ESP_OK) {
        ESP_LOGW(TAG, "bus clocked free after %u consecutive errors",
                 (unsigned)s_consec_errors);
        s_consec_errors = 0;
        return;
    }

    ESP_LOGE(TAG, "bus reset failed after %u consecutive errors -- rebuilding",
             (unsigned)s_consec_errors);

    esp_err_t rerr = bus_reopen();
    if (rerr == ESP_OK) {
        ESP_LOGW(TAG, "bus rebuilt; retrying from the next sample");
        s_consec_errors = 0;
    } else {
        ESP_LOGE(TAG, "bus rebuild failed: %s -- the fault is off-chip "
                      "(power, pull-ups, or a sensor still holding SDA low)",
                 esp_err_to_name(rerr));
    }
#endif
}

static esp_err_t locked_read(sensor_bus_dev_t dev, uint8_t addr_byte,
                             size_t skip, uint8_t *buf, size_t len)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((unsigned)dev >= SENSOR_BUS_DEV_COUNT || buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (1 + skip + len > SENSOR_BUS_MAX_XFER) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (xSemaphoreTake(s_lock, LOCK_WAIT) != pdTRUE) {
        DIAG_INC(lock_timeouts);
        return ESP_ERR_TIMEOUT;
    }
    DIAG_INC(xfers);
    esp_err_t err = bus_read(dev, addr_byte, skip, buf, len);
    bus_note_result(err);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t sensor_bus_read(sensor_bus_dev_t dev, uint8_t reg,
                          uint8_t *buf, size_t len)
{
    if (reg > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }
#if SB_IS_SPI
    /* SPI not implemented -- see the stubs above. Kept as an explicit
     * failure so a caller cannot mistake silence for a good read. */
    (void)dev; (void)reg; (void)buf; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
#else
    return locked_read(dev, reg, 0, buf, len);
#endif
}

esp_err_t sensor_bus_raw_read(sensor_bus_dev_t dev, uint8_t addr_byte,
                              uint8_t *buf, size_t len)
{
    return locked_read(dev, addr_byte, 0, buf, len);
}

esp_err_t sensor_bus_write(sensor_bus_dev_t dev, uint8_t reg,
                           const uint8_t *buf, size_t len)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((unsigned)dev >= SENSOR_BUS_DEV_COUNT || reg > 0x7F ||
        buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (1 + len > SENSOR_BUS_MAX_XFER) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (xSemaphoreTake(s_lock, LOCK_WAIT) != pdTRUE) {
        DIAG_INC(lock_timeouts);
        return ESP_ERR_TIMEOUT;
    }
    DIAG_INC(xfers);
    esp_err_t err = bus_write(dev, reg, buf, len);
    bus_note_result(err);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t sensor_bus_harden(void)
{
#if SB_IS_SPI
    uint8_t c4 = 0;
    esp_err_t err = sensor_bus_read(SENSOR_BUS_LSM6DSOX, LSM_REG_CTRL4_C, &c4, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (c4 & LSM_CTRL4_I2C_DISABLE) {
        return ESP_OK;
    }
    c4 |= LSM_CTRL4_I2C_DISABLE;   /* read-modify-write: other bits untouched */
    return sensor_bus_write(SENSOR_BUS_LSM6DSOX, LSM_REG_CTRL4_C, &c4, 1);
#else
    return ESP_OK;  /* on I2C, setting that bit would switch off the bus we use */
#endif
}

esp_err_t sensor_bus_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);

    esp_err_t err = cs_lines_high();
    if (err == ESP_OK) {
        err = bus_open();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s bus did not come up: %s",
                 sensor_bus_name(), esp_err_to_name(err));
        return err;
    }
    s_ready = true;

#if SB_IS_SPI
    ESP_LOGE(TAG, "sensor bus: SPI selected but not implemented -- no sensor "
                  "will come up. Choose I2C in menuconfig.");
#else
    ESP_LOGI(TAG, "sensor bus: I2C, %d Hz", CONFIG_SET_SENSOR_I2C_FREQ_HZ);
#endif
    return ESP_OK;
}

bool sensor_bus_is_spi(void)       { return SB_IS_SPI; }
const char *sensor_bus_name(void)  { return SB_IS_SPI ? "SPI" : "I2C"; }

/* What a wrong ID most often means, in words. */
static const char *id_hint(uint8_t v)
{
    if (v == 0x00) return "all zeros: nothing drove the data line and something "
                          "is holding it low";
    if (v == 0xFF) return "all ones: nothing drove the data line (floating or "
                          "pulled high)";
    return "some other chip, or bits shifted -- check SPI mode / clock speed";
}

esp_err_t sensor_bus_check_ids(void)
{
    uint8_t bmp = 0, lsm = 0;
    esp_err_t eb = sensor_bus_read(SENSOR_BUS_BMP388, BMP_REG_CHIP_ID, &bmp, 1);
    esp_err_t el = sensor_bus_read(SENSOR_BUS_LSM6DSOX, LSM_REG_WHO_AM_I, &lsm, 1);

    const bool bmp_ok = (eb == ESP_OK) &&
                        (bmp == BMP388_CHIP_ID || bmp == BMP390_CHIP_ID);
    const bool lsm_ok = (el == ESP_OK) && (lsm == LSM_WHO_AM_I_VALUE);

    if (eb != ESP_OK) {
        ESP_LOGE(TAG, "[%s] BMP3xx chip id: transfer failed (%s)",
                 sensor_bus_name(), esp_err_to_name(eb));
    } else if (bmp_ok) {
        ESP_LOGI(TAG, "[%s] BMP3xx chip id 0x%02X = %s  OK", sensor_bus_name(),
                 bmp, bmp == BMP390_CHIP_ID ? "BMP390" : "BMP388");
    } else {
        ESP_LOGE(TAG, "[%s] BMP3xx chip id 0x%02X, expected 0x50 or 0x60 -- %s",
                 sensor_bus_name(), bmp, id_hint(bmp));
    }

    if (el != ESP_OK) {
        ESP_LOGE(TAG, "[%s] LSM6DSOX WHO_AM_I: transfer failed (%s)",
                 sensor_bus_name(), esp_err_to_name(el));
    } else if (lsm_ok) {
        ESP_LOGI(TAG, "[%s] LSM6DSOX WHO_AM_I 0x%02X  OK", sensor_bus_name(), lsm);
    } else {
        ESP_LOGE(TAG, "[%s] LSM6DSOX WHO_AM_I 0x%02X, expected 0x6C -- %s",
                 sensor_bus_name(), lsm, id_hint(lsm));
    }

    return (bmp_ok && lsm_ok) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

void sensor_bus_get_diag(sensor_bus_diag_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_diag_mux);
    *out = s_diag;
    portEXIT_CRITICAL(&s_diag_mux);
}
