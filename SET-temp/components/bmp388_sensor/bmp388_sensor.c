#include "bmp388_sensor.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "data_router.h"
#include "event_log.h"
#include "flight_mode.h"

static const char *TAG = "BMP388";

/* ===================================================================
 *  The real struct -- private to this file. bmp388_sensor.h only knows
 *  this pointer type exists, never what's inside it.
 * =================================================================== */

struct bmp388_dev {
    bool                in_use;
    bmp388_config_t     cfg;

    spi_device_handle_t spi;
    struct bmp3_dev     bmp;       // Bosch's own device handle
    bool                initialized;

    bmp388_data_t       latest;
    portMUX_TYPE        latest_lock;

    StaticTask_t        tcb;
    StackType_t         stack[BMP388_STACK_WORDS];
    TaskHandle_t        task;

    volatile uint64_t   isr_timestamp_us;
    bool                isr_installed;

    /* Current rate configuration. Read by the task each loop against
     * flight_mode_is_fast() so a mode change reconfigures the chip
     * exactly once rather than on every pass. */
    bool                fast;

    bmp388_diag_t       diag;
};

/* ===================================================================
 *  Static instance pool
 * =================================================================== */

static struct bmp388_dev s_pool[BMP388_MAX_INSTANCES];
static portMUX_TYPE      s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* ===================================================================
 *  SPI HAL callbacks for the Bosch driver
 * =================================================================== */

static BMP3_INTF_RET_TYPE bmp388_spi_read(uint8_t reg_addr, uint8_t *data,
                                          uint32_t len, void *intf_ptr)
{
    bmp388_handle_t dev = (bmp388_handle_t)intf_ptr;

    if (!dev || len > 32) {
        return BMP3_E_COMM_FAIL;
    }

    uint8_t tx[34] = {0};
    uint8_t rx[34] = {0};
    tx[0] = reg_addr;   /* read bit (0x80) already set by bmp3.c */

    spi_transaction_t t = {
        .length   = (len + 1) * 8,
        .rxlength = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI read 0x%02X failed: %s", reg_addr, esp_err_to_name(err));
        return BMP3_E_COMM_FAIL;
    }

    /* rx[0] is garbage clocked in while we were sending the address byte */
    memcpy(data, &rx[1], len);
    return BMP3_INTF_RET_SUCCESS;
}

static BMP3_INTF_RET_TYPE bmp388_spi_write(uint8_t reg_addr, const uint8_t *data,
                                           uint32_t len, void *intf_ptr)
{
    bmp388_handle_t dev = (bmp388_handle_t)intf_ptr;

    if (!dev || len > 32) {
        return BMP3_E_COMM_FAIL;
    }

    uint8_t tx[33];
    tx[0] = reg_addr;   /* bmp3.c already masked off the read bit */
    memcpy(&tx[1], data, len);

    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
    };

    esp_err_t err = spi_device_polling_transmit(dev->spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI write 0x%02X failed: %s", reg_addr, esp_err_to_name(err));
        return BMP3_E_COMM_FAIL;
    }
    return BMP3_INTF_RET_SUCCESS;
}

static void bmp388_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    uint32_t ticks = pdMS_TO_TICKS((period + 999) / 1000);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

/* ===================================================================
 *  Altitude -- pure math
 * =================================================================== */

static float bmp388_altitude_from_pressure(float pressure_pa, float ref_pa)
{
    if (ref_pa <= 0.0f) {
        ref_pa = 101325.0f;
    }
    return 44330.0f * (1.0f - powf(pressure_pa / ref_pa, 1.0f / 5.255f));
}

/* ===================================================================
 *  ISR
 * =================================================================== */

static void IRAM_ATTR bmp388_isr(void *arg)
{
    bmp388_handle_t dev = (bmp388_handle_t)arg;
    BaseType_t hp_task_woken = pdFALSE;

    dev->isr_timestamp_us = (uint64_t)esp_timer_get_time();
    dev->diag.irq_count++;

    vTaskNotifyGiveFromISR(dev->task, &hp_task_woken);
    portYIELD_FROM_ISR(hp_task_woken);
}

/* ===================================================================
 *  Reader task
 * =================================================================== */

static void bmp388_task(void *arg)
{
    bmp388_handle_t dev = (bmp388_handle_t)arg;
    const TickType_t timeout = pdMS_TO_TICKS(BMP388_IRQ_TIMEOUT_MS);

    ESP_LOGI(TAG, "reader task running on core %d", (int)xPortGetCoreID());

    for (;;) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, timeout);

        /* Reconfigure from the task that owns the SPI device, never from
         * whichever task changed the mode. */
        bool want_fast = flight_mode_is_fast();
        if (want_fast != dev->fast) {
            bmp388_set_rate(dev, want_fast);
        }

        uint64_t stamp;
        if (notified == 0) {
            dev->diag.irq_timeouts++;
            event_log_error(ERR_SRC_BMP388, ERR_CODE_IRQ_TIMEOUT, ESP_ERR_TIMEOUT, dev->diag.irq_timeouts);
            stamp = (uint64_t)esp_timer_get_time();
        } else {
            stamp = dev->isr_timestamp_us;
        }

        struct bmp3_data raw;
        int8_t rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &raw, &dev->bmp);
        if (rslt != BMP3_OK) {
            dev->diag.read_errors++;
            event_log_error(ERR_SRC_BMP388, ERR_CODE_READ_FAILED, ESP_FAIL, dev->diag.read_errors);
            continue;
        }

        bmp388_data_t sample;
        sample.pressure_pa   = (float)raw.pressure;
        sample.temperature_c = (float)raw.temperature;
        sample.altitude_m    = bmp388_altitude_from_pressure(sample.pressure_pa,
                                                              dev->cfg.ref_pressure_pa);
        sample.timestamp_us  = stamp;
        sample.seq           = ++dev->diag.samples;

        portENTER_CRITICAL(&dev->latest_lock);
        dev->latest = sample;
        portEXIT_CRITICAL(&dev->latest_lock);

        /* Pushed here, not by a separate bridge task -- `sample` is
         * already built and in scope, so this avoids the old
         * lock-and-copy round trip where a different task had to
         * re-fetch what this task just published. Moving this here
         * means bmp388_sensor.c is no longer SD/telemetry-agnostic;
         * that coupling was a deliberate, discussed trade-off, not an
         * oversight -- see data_router.h for the reasoning. */
        baro_record_t rec = {
            .seq           = sample.seq,
            .timestamp_us  = sample.timestamp_us,
            .pressure_pa   = sample.pressure_pa,
            .temperature_c = sample.temperature_c,
            .altitude_m    = sample.altitude_m,
        };
        data_router_publish(STREAM_BARO, &rec, sizeof(rec));
    }
}

/* ===================================================================
 *  Sensor configuration (Bosch API calls)
 * =================================================================== */

/* Standby profile: 1.5 Hz with 2x pressure oversampling.
 *
 * Oversampling drops alongside the rate rather than the rate alone. This
 * part's current draw is dominated by how many conversions it averages
 * per sample, so leaving 8x oversampling on at 1.5 Hz would give up most
 * of the saving. Standby altitude is noisier as a result -- the correct
 * trade for a vehicle sitting still on a pad. */
#define BMP388_ODR_SLOW        BMP3_ODR_1_5_HZ
#define BMP388_OSR_PRESS_SLOW  BMP3_OVERSAMPLING_2X

/* Rate/oversampling only. The interrupt configuration is deliberately
 * NOT touched here: it is set once at init and must survive every mode
 * change, or a reconfiguration would silently unhook DRDY and the task
 * would fall back to polling for the rest of the flight. */
static esp_err_t bmp388_apply_rate(bmp388_handle_t dev, bool fast)
{
    struct bmp3_settings settings = {0};
    settings.press_en = BMP3_ENABLE;
    settings.temp_en  = BMP3_ENABLE;
    settings.odr_filter.press_os   = fast ? dev->cfg.osr_pressure
                                          : BMP388_OSR_PRESS_SLOW;
    settings.odr_filter.temp_os    = dev->cfg.osr_temp;
    settings.odr_filter.iir_filter = dev->cfg.iir_filter;
    settings.odr_filter.odr        = fast ? dev->cfg.odr : BMP388_ODR_SLOW;

    uint32_t sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN
                 | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS
                 | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;

    int8_t rslt = bmp3_set_sensor_settings(sel, &settings, &dev->bmp);
    if (rslt != BMP3_OK) {
        ESP_LOGE(TAG, "rate set_sensor_settings failed (%d)", rslt);
        event_log_error(ERR_SRC_BMP388, ERR_CODE_CONFIG_INVALID, ESP_FAIL,
                        (uint32_t)fast);
        return ESP_FAIL;
    }
    dev->fast = fast;
    return ESP_OK;
}

esp_err_t bmp388_set_rate(bmp388_handle_t dev, bool fast)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->fast == fast) {
        return ESP_OK;
    }
    esp_err_t err = bmp388_apply_rate(dev, fast);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "rate -> %s (%s)", fast ? "FLIGHT" : "STANDBY",
                 fast ? "50 Hz, 8x OSR" : "1.5 Hz, 2x OSR");
    }
    return err;
}

bool bmp388_is_fast(bmp388_handle_t dev)
{
    return dev ? dev->fast : true;
}

static esp_err_t bmp388_configure_sensor(bmp388_handle_t dev)
{
    struct bmp3_settings settings = {0};
    settings.press_en = BMP3_ENABLE;
    settings.temp_en  = BMP3_ENABLE;
    settings.odr_filter.press_os   = dev->cfg.osr_pressure;
    settings.odr_filter.temp_os    = dev->cfg.osr_temp;
    settings.odr_filter.iir_filter = dev->cfg.iir_filter;
    settings.odr_filter.odr        = dev->cfg.odr;

    settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
    settings.int_settings.level       = BMP3_INT_PIN_ACTIVE_HIGH;
    settings.int_settings.latch       = BMP3_INT_PIN_LATCH;
    settings.int_settings.drdy_en     = BMP3_ENABLE;

    uint32_t sel = BMP3_SEL_PRESS_EN   | BMP3_SEL_TEMP_EN
                 | BMP3_SEL_PRESS_OS   | BMP3_SEL_TEMP_OS
                 | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR
                 | BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_LATCH
                 | BMP3_SEL_DRDY_EN;

    int8_t rslt = bmp3_set_sensor_settings(sel, &settings, &dev->bmp);
    if (rslt != BMP3_OK) {
        ESP_LOGE(TAG, "set_sensor_settings failed (%d)", rslt);
        return ESP_FAIL;
    }

    dev->fast = true;   /* boots in the flight profile; main() slows it */

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev->bmp);
    if (rslt != BMP3_OK) {
        ESP_LOGE(TAG, "set_op_mode failed (%d)", rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t bmp388_setup_interrupt(bmp388_handle_t dev)
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

    err = gpio_isr_handler_add(dev->cfg.int_pin, bmp388_isr, (void *)dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return err;
    }

    dev->isr_installed = true;
    ESP_LOGI(TAG, "INT armed on GPIO%d (rising edge)", (int)dev->cfg.int_pin);
    return ESP_OK;
}

/* ===================================================================
 *  Public API
 * =================================================================== */

esp_err_t bmp388_init(const bmp388_config_t *cfg, bmp388_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->int_pin < 0) {
        ESP_LOGE(TAG, "need a real INT pin for interrupt mode");
        return ESP_ERR_INVALID_ARG;
    }

    /* Claim a pool slot -- same idea as the XBee's inline scan, kept
     * inside a critical section since, unlike the screenshot's version,
     * more than one task in this project could plausibly call an init
     * function around boot time. Cheap insurance against two callers
     * racing for the same slot. */
    struct bmp388_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < BMP388_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", BMP388_MAX_INSTANCES);
        return ESP_ERR_NO_MEM;
    }

    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    dev->latest_lock = tmp;

    dev->cfg = *cfg;   /* shallow copy -- cfg can die after this call returns */
    if (dev->cfg.ref_pressure_pa <= 0.0f) {
        dev->cfg.ref_pressure_pa = 101325.0f;
    }

    spi_device_interface_config_t devcfg = {
        .mode           = 0,
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

    dev->bmp.intf     = BMP3_SPI_INTF;
    dev->bmp.intf_ptr = dev;    /* callbacks find this exact instance */
    dev->bmp.read     = bmp388_spi_read;
    dev->bmp.write    = bmp388_spi_write;
    dev->bmp.delay_us = bmp388_delay_us;

    int8_t rslt = bmp3_init(&dev->bmp);
    if (rslt != BMP3_OK) {
        ESP_LOGE(TAG, "bmp3_init failed (%d) -- check MISO/MOSI/CS wiring", rslt);
        spi_bus_remove_device(dev->spi);
        dev->in_use = false;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP388/390 found, chip ID 0x%02X", dev->bmp.chip_id);

    if (bmp388_configure_sensor(dev) != ESP_OK) {
        spi_bus_remove_device(dev->spi);
        dev->in_use = false;
        return ESP_FAIL;
    }

    err = bmp388_setup_interrupt(dev);
    if (err != ESP_OK) {
        spi_bus_remove_device(dev->spi);
        dev->in_use = false;
        return err;
    }

    dev->initialized = true;
    *out = dev;
    ESP_LOGI(TAG, "BMP388 initialized (SPI, interrupt-driven)");
    return ESP_OK;
}

esp_err_t bmp388_start(bmp388_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->task) {
        ESP_LOGW(TAG, "task already running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->task = xTaskCreateStaticPinnedToCore(
        bmp388_task, "bmp388", BMP388_STACK_WORDS,
        (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        return ESP_FAIL;
    }

    struct bmp3_status status;
    (void)bmp3_get_status(&status, &dev->bmp);

    return ESP_OK;
}

esp_err_t bmp388_stop(bmp388_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->isr_installed) {
        gpio_isr_handler_remove(dev->cfg.int_pin);
        dev->isr_installed = false;
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

void bmp388_get_latest(bmp388_handle_t dev, bmp388_data_t *out)
{
    if (!dev || !out) {
        return;
    }
    portENTER_CRITICAL(&dev->latest_lock);
    *out = dev->latest;
    portEXIT_CRITICAL(&dev->latest_lock);
}

esp_err_t bmp388_wait_for_sample(bmp388_handle_t dev, uint32_t last_seq,
                                 bmp388_data_t *out, TickType_t timeout)
{
    if (!dev || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    const TickType_t start = xTaskGetTickCount();
    const TickType_t poll  = pdMS_TO_TICKS(1);

    for (;;) {
        bmp388_get_latest(dev, out);
        if (out->seq != last_seq) {
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(poll);
    }
}

esp_err_t bmp388_read_once(bmp388_handle_t dev, bmp388_data_t *out)
{
    if (!dev || !out || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    struct bmp3_data raw;
    int8_t rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &raw, &dev->bmp);
    if (rslt != BMP3_OK) {
        return ESP_FAIL;
    }

    out->pressure_pa   = (float)raw.pressure;
    out->temperature_c = (float)raw.temperature;
    out->altitude_m    = bmp388_altitude_from_pressure(out->pressure_pa,
                                                       dev->cfg.ref_pressure_pa);
    out->timestamp_us  = (uint64_t)esp_timer_get_time();
    out->seq           = 0;
    return ESP_OK;
}

void bmp388_set_reference_pressure(bmp388_handle_t dev, float pressure_pa)
{
    if (!dev || pressure_pa <= 0.0f) {
        return;
    }
    dev->cfg.ref_pressure_pa = pressure_pa;
    ESP_LOGI(TAG, "reference pressure set to %.1f Pa", pressure_pa);
}

void bmp388_zero_altitude(bmp388_handle_t dev)
{
    if (!dev) {
        return;
    }
    bmp388_data_t d;
    bmp388_get_latest(dev, &d);
    if (d.seq != 0 && d.pressure_pa > 0.0f) {
        bmp388_set_reference_pressure(dev, d.pressure_pa);
    } else {
        ESP_LOGW(TAG, "no sample yet; altitude not zeroed");
    }
}

void bmp388_get_diagnostics(bmp388_handle_t dev, bmp388_diag_t *out)
{
    if (!dev || !out) {
        return;
    }
    *out = dev->diag;
}
