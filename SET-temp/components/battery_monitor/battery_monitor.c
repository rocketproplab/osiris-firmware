#include <stdio.h>
#include "battery_monitor.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "data_router.h"
#include "flight_mode.h"
#include "event_log.h"

static const char *TAG = "BATTERY";

#define BATTERY_DEFAULT_RATE_HZ   10
#define BATTERY_ADC_ATTEN         ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH      ADC_BITWIDTH_DEFAULT

/* ===================================================================
 *  Private instance -- same idiom as the sensor drivers.
 * =================================================================== */

struct battery_dev {
    bool                  in_use;
    battery_config_t      cfg;
    bool                  initialized;

    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t         cali;
    bool                      cali_enabled;

    battery_data_t        latest;
    portMUX_TYPE          latest_lock;

    StaticTask_t          tcb;
    StackType_t           stack[BATTERY_STACK_WORDS];
    TaskHandle_t          task;
    volatile bool         task_should_run;

    portMUX_TYPE          diag_lock;
    battery_diag_t        diag;
};

static struct battery_dev s_pool[BATTERY_MAX_INSTANCES];
static portMUX_TYPE       s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* ===================================================================
 *  One conversion set -> volts.
 *
 *  Averaging happens on the RAW counts, not on converted volts. The
 *  calibration curve is non-linear, so averaging after conversion would
 *  bias the result by the curvature across the sampled spread. Averaging
 *  first, converting once, is both cheaper and more correct.
 * =================================================================== */

static esp_err_t battery_sample(struct battery_dev *dev, float *pin_v, float *pack_v)
{
    uint8_t n = dev->cfg.oversample ? dev->cfg.oversample : BATTERY_OVERSAMPLE;
    int32_t accum = 0;
    int     good  = 0;

    for (uint8_t i = 0; i < n; i++) {
        int raw = 0;
        if (adc_oneshot_read(dev->adc, dev->cfg.channel, &raw) == ESP_OK) {
            accum += raw;
            good++;
        }
    }

    if (good == 0) {
        return ESP_FAIL;
    }

    int mean_raw = (int)(accum / good);
    int mv = 0;

    if (dev->cali_enabled) {
        esp_err_t err = adc_cali_raw_to_voltage(dev->cali, mean_raw, &mv);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        /* No calibration fuses. Fall back to the nominal full-scale of
         * the 12 dB attenuation range. This is good to maybe +/-5%, so
         * the number is still useful for spotting a sagging pack, just
         * not for a precise state-of-charge estimate. adc_v is logged
         * alongside battery_v precisely so this can be re-scaled
         * offline if a calibrated reference is measured later. */
        mv = (mean_raw * 3100) / 4095;
    }

    float ratio = dev->cfg.divider_ratio > 0.0f
                    ? dev->cfg.divider_ratio
                    : BATTERY_DIVIDER_RATIO_V1;

    *pin_v  = (float)mv / 1000.0f;
    *pack_v = *pin_v / ratio;
    return ESP_OK;
}

static void battery_record_sample(struct battery_dev *dev, float pin_v, float pack_v)
{
    uint64_t now = (uint64_t)esp_timer_get_time();

    portENTER_CRITICAL(&dev->latest_lock);
    dev->latest.adc_v        = pin_v;
    dev->latest.battery_v    = pack_v;
    dev->latest.timestamp_us = now;
    dev->latest.seq++;
    uint32_t seq = dev->latest.seq;
    portEXIT_CRITICAL(&dev->latest_lock);

    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.samples++;
    if (pack_v < dev->diag.min_volts || dev->diag.min_volts == 0.0f) {
        dev->diag.min_volts = pack_v;
    }
    portEXIT_CRITICAL(&dev->diag_lock);

    batt_record_t rec = {
        .seq          = seq,
        .timestamp_us = now,
        .battery_v    = pack_v,
        .adc_v        = pin_v,
    };
    data_router_publish(STREAM_BATT, &rec, sizeof(rec));

    /* Threshold logging only -- this never changes what the vehicle
     * does. A monitor that could alter flight behaviour would need a
     * far more careful debounce story than one comparison. */
    if (dev->cfg.critical_volts > 0.0f && pack_v < dev->cfg.critical_volts) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.critical_events++;
        portEXIT_CRITICAL(&dev->diag_lock);
        ESP_LOGE(TAG, "pack CRITICAL: %.2f V (< %.2f V)",
                 pack_v, dev->cfg.critical_volts);
    } else if (dev->cfg.warn_volts > 0.0f && pack_v < dev->cfg.warn_volts) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.warn_events++;
        portEXIT_CRITICAL(&dev->diag_lock);
        ESP_LOGW(TAG, "pack low: %.2f V (< %.2f V)", pack_v, dev->cfg.warn_volts);
    }
}

/* ===================================================================
 *  Task -- a plain fixed-cadence sampler.
 *
 *  vTaskDelayUntil, not vTaskDelay: the cadence must not drift by the
 *  execution time of each pass, otherwise the "10 Hz" in the record
 *  rate slowly becomes a lie and offline rate reconstruction skews.
 * =================================================================== */

static void battery_task(void *arg)
{
    struct battery_dev *dev = (struct battery_dev *)arg;

    uint32_t fast_hz = dev->cfg.sample_rate_hz ? dev->cfg.sample_rate_hz
                                                : BATTERY_DEFAULT_RATE_HZ;
    bool was_fast = flight_mode_is_fast();
    TickType_t period = pdMS_TO_TICKS(1000 / (was_fast ? fast_hz
                                                       : BATTERY_STANDBY_RATE_HZ));
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "battery task running on core %d (%lu Hz flight / %d Hz standby)",
             (int)xPortGetCoreID(), (unsigned long)fast_hz, BATTERY_STANDBY_RATE_HZ);

    while (dev->task_should_run) {
        float pin_v = 0.0f, pack_v = 0.0f;
        if (battery_sample(dev, &pin_v, &pack_v) == ESP_OK) {
            battery_record_sample(dev, pin_v, pack_v);
        } else {
            portENTER_CRITICAL(&dev->diag_lock);
            dev->diag.read_errors++;
            event_log_error(ERR_SRC_BATTERY, ERR_CODE_READ_FAILED, ESP_FAIL, dev->diag.read_errors);
            portEXIT_CRITICAL(&dev->diag_lock);
        }
        /* Re-derive the period when the mode changes. vTaskDelayUntil
         * keeps the cadence honest within a mode; this is what makes the
         * two modes' rates both true rather than drifting. */
        bool now_fast = flight_mode_is_fast();
        if (now_fast != was_fast) {
            was_fast = now_fast;
            period = pdMS_TO_TICKS(1000 / (now_fast ? fast_hz
                                                    : BATTERY_STANDBY_RATE_HZ));
            last_wake = xTaskGetTickCount();
            ESP_LOGW(TAG, "rate -> %s (%lu Hz)", now_fast ? "FLIGHT" : "STANDBY",
                     (unsigned long)(now_fast ? fast_hz : BATTERY_STANDBY_RATE_HZ));
        }
        vTaskDelayUntil(&last_wake, period > 0 ? period : 1);
    }

    dev->task = NULL;
    vTaskDelete(NULL);
}

/* ===================================================================
 *  Public API
 * =================================================================== */

esp_err_t battery_init(const battery_config_t *cfg, battery_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    struct battery_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < BATTERY_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", BATTERY_MAX_INSTANCES);
        return ESP_ERR_NO_MEM;
    }

    portMUX_TYPE tmp_a = portMUX_INITIALIZER_UNLOCKED;
    portMUX_TYPE tmp_b = portMUX_INITIALIZER_UNLOCKED;
    dev->latest_lock = tmp_a;
    dev->diag_lock   = tmp_b;
    dev->cfg         = *cfg;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,        /* GPIO20 is ADC1_CH4 -- fixed by silicon */
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &dev->adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        dev->in_use = false;
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    err = adc_oneshot_config_channel(dev->adc, dev->cfg.channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(dev->adc);
        dev->in_use = false;
        return err;
    }

    /* Calibration is best-effort. A chip without the eFuse calibration
     * block still returns usable raw counts, and a rough voltage beats
     * refusing to boot over a housekeeping stream. */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = dev->cfg.channel,
        .atten    = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &dev->cali) == ESP_OK) {
        dev->cali_enabled = true;
        ESP_LOGI(TAG, "ADC curve-fitting calibration active");
    } else {
        ESP_LOGW(TAG, "no ADC calibration available -- voltages are approximate");
    }
#else
    ESP_LOGW(TAG, "ADC calibration scheme unsupported -- voltages are approximate");
#endif

    dev->initialized = true;
    *out = dev;

    float ratio = dev->cfg.divider_ratio > 0.0f
                    ? dev->cfg.divider_ratio
                    : BATTERY_DIVIDER_RATIO_V1;
    ESP_LOGI(TAG, "battery monitor on ADC1_CH%d, divider %.4f (full scale %.2f V)",
             (int)dev->cfg.channel, ratio, 3.3f / ratio);
    return ESP_OK;
}

esp_err_t battery_start(battery_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->task) {
        ESP_LOGW(TAG, "task already running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->task_should_run = true;
    dev->task = xTaskCreateStaticPinnedToCore(
        battery_task, "battery", BATTERY_STACK_WORDS,
        (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        dev->task_should_run = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t battery_stop(battery_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->task_should_run = false;
    for (int i = 0; i < 100 && dev->task; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (dev->cali_enabled) {
        adc_cali_delete_scheme_curve_fitting(dev->cali);
        dev->cali_enabled = false;
    }
#endif
    if (dev->adc) {
        adc_oneshot_del_unit(dev->adc);
        dev->adc = NULL;
    }

    dev->initialized = false;
    dev->in_use      = false;
    return ESP_OK;
}

void battery_get_latest(battery_handle_t dev, battery_data_t *out)
{
    if (!dev || !out) {
        return;
    }
    portENTER_CRITICAL(&dev->latest_lock);
    *out = dev->latest;
    portEXIT_CRITICAL(&dev->latest_lock);
}

esp_err_t battery_read_once(battery_handle_t dev, battery_data_t *out)
{
    if (!dev || !dev->initialized || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    float pin_v = 0.0f, pack_v = 0.0f;
    esp_err_t err = battery_sample(dev, &pin_v, &pack_v);
    if (err != ESP_OK) {
        return err;
    }

    battery_record_sample(dev, pin_v, pack_v);
    battery_get_latest(dev, out);
    return ESP_OK;
}

void battery_get_diagnostics(battery_handle_t dev, battery_diag_t *out)
{
    if (!dev || !out) {
        return;
    }
    portENTER_CRITICAL(&dev->diag_lock);
    *out = dev->diag;
    portEXIT_CRITICAL(&dev->diag_lock);
}
