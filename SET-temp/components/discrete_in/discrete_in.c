#include <stdio.h>
#include "discrete_in.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "event_log.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#ifdef CONFIG_DISCRETE_IN_MODE_ANALOG
#include "discrete_threshold.h"
#include "adc1_shared.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* Usable top of the ADC at 12 dB attenuation, as a PIN voltage. */
#define DISCRETE_ADC_FULL_SCALE_MV  3100
#define DISCRETE_ADC_OVERSAMPLE     4
#define DISCRETE_IDLE_EMA_SHIFT     6      /* idle average: ~64 samples */

/* The divider: line_mv = pin_mv * (top + bottom) / bottom. A bottom of 0
 * means no resistor to ground: the pin draws no current, so it reads the
 * line 1:1 whatever the series resistor is. */
#define DISCRETE_DIV_TOP     ((uint32_t)CONFIG_DISCRETE_IN_DIVIDER_TOP_OHMS)
#define DISCRETE_DIV_BOTTOM  ((uint32_t)CONFIG_DISCRETE_IN_DIVIDER_BOTTOM_OHMS)

static uint32_t pin_to_line_mv(uint32_t pin_mv)
{
    if (DISCRETE_DIV_BOTTOM == 0){ 
        return pin_mv; 
    } else {
        return (uint32_t)(((uint64_t)pin_mv * (DISCRETE_DIV_TOP + DISCRETE_DIV_BOTTOM))
                        / DISCRETE_DIV_BOTTOM);
    }
}

static uint32_t line_to_pin_mv(uint32_t line_mv)
{
    if (DISCRETE_DIV_BOTTOM == 0) { return line_mv; }
    return (uint32_t)(((uint64_t)line_mv * DISCRETE_DIV_BOTTOM)
                      / (DISCRETE_DIV_TOP + DISCRETE_DIV_BOTTOM));
}
#endif

static const char *TAG = "DISCRETE_IN";

struct discrete_in_dev {
    bool                in_use;
    discrete_in_config_t  cfg;
    bool                initialized;
    bool                isr_installed;

    StaticTask_t        tcb;
    StackType_t         stack[DISCRETE_IN_STACK_WORDS];
    TaskHandle_t        task;

    volatile uint64_t   isr_timestamp_us;
    /* The accepted-trigger count AT THE MOMENT OF THAT EDGE, captured
     * in the ISR. The task used to read diag.triggers twice -- once for
     * its log line, once for the event record -- and on a chattering
     * input the ISR bumped it between the two, so the console and the
     * flight log disagreed about the same edge. */
    volatile uint32_t   isr_trigger_count;
    volatile uint64_t   last_accepted_us;
    volatile bool       fired;
    discrete_in_diag_t    diag;

#ifdef CONFIG_DISCRETE_IN_MODE_ANALOG
    adc_channel_t       adc_ch;
    adc_cali_handle_t   cali;
    bool                cali_ok;
    dthr_t              thr;
#endif
};

static struct discrete_in_dev s_pool[DISCRETE_IN_MAX_INSTANCES];
static portMUX_TYPE         s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* ===================================================================
 *  ISR -- same rules as every other interrupt in this project:
 *  IRAM_ATTR, no logging, no blocking. Timestamp, count, wake the task.
 *
 *  The timestamp is taken HERE and handed to the task, not read again
 *  when the task runs. The whole value of this record is knowing when
 *  the Blue Raven actually asserted the line; a time captured after
 *  scheduling would be measuring FreeRTOS, not the vehicle.
 * =================================================================== */

/* unused in ANALOG builds, which have no interrupt */
__attribute__((unused)) static void IRAM_ATTR discrete_in_isr(void *arg)
{
    struct discrete_in_dev *dev = (struct discrete_in_dev *)arg;
    BaseType_t hp_task_woken = pdFALSE;

    uint64_t now = (uint64_t)esp_timer_get_time();

    /* Rejected here rather than in the task: a suppressed edge must not
     * cost a notify, a wake, or a console line, since the whole problem
     * being solved is this task running too often. `fired` doubles as
     * "we have accepted one already", so the first edge always gets
     * through with its exact timestamp. */
    if (DISCRETE_IN_MIN_INTERVAL_US > 0 && dev->fired &&
        (now - dev->last_accepted_us) < DISCRETE_IN_MIN_INTERVAL_US) {
        dev->diag.suppressed++;
        return;
    }

    dev->isr_timestamp_us = now;
    dev->last_accepted_us = now;
    if (!dev->fired) {
        dev->diag.first_trigger_us = now;
        dev->fired = true;
    }
    dev->diag.triggers++;
    dev->isr_trigger_count = dev->diag.triggers;

    vTaskNotifyGiveFromISR(dev->task, &hp_task_woken);
    portYIELD_FROM_ISR(hp_task_woken);
}

/* ===================================================================
 *  Task -- blocks with NO timeout, unlike the sensor tasks. There is no
 *  fallback that makes sense here: a sensor task falls back to polling
 *  if an interrupt is missed, because "read anyway" is always safe.
 *  There is no equivalent safe fallback for "assume the Blue Raven
 *  fired", so this waits indefinitely rather than guessing.
 * =================================================================== */

__attribute__((unused)) static void discrete_in_task(void *arg)
{
    struct discrete_in_dev *dev = (struct discrete_in_dev *)arg;

    ESP_LOGI(TAG, "%s task running on core %d, waiting for input edge",
             dev->cfg.name ? dev->cfg.name : "discrete", (int)xPortGetCoreID());

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Both of these come from what the ISR captured for THIS edge,
         * so the console line and the flight record cannot disagree
         * about it. They used to be two separate reads of
         * diag.triggers, which a chattering input changed in between. */
        uint64_t ts    = dev->isr_timestamp_us;
        uint32_t count = dev->isr_trigger_count;
        uint32_t sup   = dev->diag.suppressed;

        if (sup) {
            ESP_LOGW(TAG, "%s SIGNAL at %llu us (trigger #%lu, %lu edge(s) "
                          "suppressed so far -- the line is chattering)",
                     dev->cfg.name ? dev->cfg.name : "discrete",
                     (unsigned long long)ts, (unsigned long)count,
                     (unsigned long)sup);
        } else {
            ESP_LOGW(TAG, "%s SIGNAL at %llu us (trigger #%lu)",
                     dev->cfg.name ? dev->cfg.name : "discrete",
                     (unsigned long long)ts, (unsigned long)count);
        }

        /* Recorded from the task, not the ISR: event_log ends in a ring
         * buffer push, which sd_logger does not support from interrupt
         * context. Passing the ISR timestamp through keeps the recorded
         * time honest anyway. */
        event_log_event_at(dev->cfg.event, dev->cfg.channel, count, ts);

        if (dev->cfg.output_pin != GPIO_NUM_NC) {
            gpio_set_level(dev->cfg.output_pin, DISCRETE_IN_OUTPUT_ACTIVE_LEVEL);
        }
    }
}

#ifdef CONFIG_DISCRETE_IN_MODE_ANALOG
/* ===================================================================
 *  ANALOG MODE
 *
 *  The task samples the line every DISCRETE_IN_SAMPLE_MS and runs it
 *  through discrete_threshold's two-threshold, N-sample state machine.
 *  No interrupt: the ADC has none worth using here, and a periodic read
 *  is what makes a level that sits ABOVE the digital HIGH threshold
 *  when idle readable at all.
 *
 *  The minimum-interval filter still applies, to ACCEPTED firings, with
 *  the same meaning as in digital mode.
 * =================================================================== */

/* One averaged reading, as mV on the Blue Raven side of the divider. */
static bool discrete_read_line_mv(struct discrete_in_dev *dev, uint32_t *line_mv,
                                  bool *clipped)
{
    int32_t sum = 0;
    int good = 0;
    for (int i = 0; i < DISCRETE_ADC_OVERSAMPLE; i++) {
        int raw = 0;
        if (adc1_shared_read(dev->adc_ch, &raw) == ESP_OK) {
            sum += raw;
            good++;
        }
    }
    if (good == 0) {
        return false;
    }
    int raw = (int)(sum / good);
    *clipped = raw >= 4095 - 4;

    int pin_mv = 0;
    if (dev->cali_ok) {
        if (adc_cali_raw_to_voltage(dev->cali, raw, &pin_mv) != ESP_OK) {
            return false;
        }
    } else {
        pin_mv = (raw * DISCRETE_ADC_FULL_SCALE_MV) / 4095;
    }
    *line_mv = pin_to_line_mv((uint32_t)pin_mv);

//     ESP_LOGI(TAG, " %d volts", *line_mv);

    return true;
}

static void discrete_in_accept(struct discrete_in_dev *dev, uint64_t edge_us, uint32_t mv)
{
    const char *name = dev->cfg.name ? dev->cfg.name : "discrete";

    if (DISCRETE_IN_MIN_INTERVAL_US > 0 && dev->fired &&
        (edge_us - dev->last_accepted_us) < DISCRETE_IN_MIN_INTERVAL_US) {
        dev->diag.suppressed++;
        return;
    }
    dev->last_accepted_us = edge_us;
    if (!dev->fired) {
        dev->diag.first_trigger_us = edge_us;
        dev->fired = true;
    }
    uint32_t count = ++dev->diag.triggers;

    ESP_LOGW(TAG, "%s SIGNAL at %llu us (trigger #%lu, line %lu mV >= %lu mV)",
             name, (unsigned long long)edge_us, (unsigned long)count,
             (unsigned long)mv, (unsigned long)dev->thr.fire_mv);
    event_log_event_at(dev->cfg.event, dev->cfg.channel, count, edge_us);

    if (dev->cfg.output_pin != GPIO_NUM_NC) {
        gpio_set_level(dev->cfg.output_pin, DISCRETE_IN_OUTPUT_ACTIVE_LEVEL);
    }
}

static void discrete_in_analog_task(void *arg)
{
    struct discrete_in_dev *dev = (struct discrete_in_dev *)arg;
    const char *name = dev->cfg.name ? dev->cfg.name : "discrete";
    TickType_t period = pdMS_TO_TICKS(CONFIG_DISCRETE_IN_SAMPLE_MS);
    if (period == 0) { period = 1; }
    TickType_t last_wake = xTaskGetTickCount();
    bool announced = false, clip_said = false;

    ESP_LOGI(TAG, "%s task running on core %d, sampling ADC1_CH%d every %d ms",
             name, (int)xPortGetCoreID(), (int)dev->adc_ch, CONFIG_DISCRETE_IN_SAMPLE_MS);

    for (;;) {
        uint32_t mv = 0;
        bool clipped = false;
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        if (discrete_read_line_mv(dev, &mv, &clipped)) {
            dev->diag.last_mv = mv;
            if (mv > dev->diag.peak_mv) { dev->diag.peak_mv = mv; }
            if (!dev->thr.high) {
                /* Running average of the idle level: the number to set
                 * the thresholds from, and how far the Blue Raven's
                 * battery has sagged since boot. */
                if (dev->diag.idle_mv == 0) { dev->diag.idle_mv = mv; }
                else {
                    dev->diag.idle_mv += ((int32_t)mv - (int32_t)dev->diag.idle_mv)
                                         >> DISCRETE_IDLE_EMA_SHIFT;
                }
            }
            if (clipped) {
                dev->diag.clipped++;
                /* Clipping while FIRED is expected with no divider: 3.7 V
                 * reads as full scale, which is still above the threshold.
                 * Clipping while IDLE is the real problem -- idle and fired
                 * then read the same -- and it only needs saying once. */
                if (!dev->thr.high && !clip_said) {
                    clip_said = true;
                    ESP_LOGE(TAG, "%s: ADC at FULL SCALE while IDLE -- the idle level "
                                  "is above what can be measured, so a firing cannot "
                                  "be told apart. Lower the idle level at the pin.", name);
                    event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_OVERFLOW, ESP_OK, mv);
                }
            }
            if (!announced) {
                announced = true;
                ESP_LOGI(TAG, "%s line reads %lu mV at start (fire >= %lu, release <= %lu)",
                         name, (unsigned long)mv, (unsigned long)dev->thr.fire_mv,
                         (unsigned long)dev->thr.release_mv);
                if (mv >= dev->thr.fire_mv) {
                    ESP_LOGE(TAG, "%s is ALREADY at or above its fire threshold at "
                                  "start -- it will report a firing now. Check the "
                                  "threshold against the idle level.", name);
                }
            }

            uint64_t edge_us = 0;
            dthr_result_t r = dthr_step(&dev->thr, mv, now_us, &edge_us);
            if (r == DTHR_ROSE) {
                discrete_in_accept(dev, edge_us, mv);
            } else if (r == DTHR_FELL) {
                ESP_LOGI(TAG, "%s back to idle (%lu mV <= %lu mV)", name,
                         (unsigned long)mv, (unsigned long)dev->thr.release_mv);
            }
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

static esp_err_t discrete_in_analog_init(struct discrete_in_dev *dev)
{
    const char *name = dev->cfg.name ? dev->cfg.name : "discrete";
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel((int)dev->cfg.input_pin, &unit, &dev->adc_ch);
    if (err != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "%s: GPIO%d is not an ADC1 pin -- analog mode impossible here",
                 name, (int)dev->cfg.input_pin);
        return err != ESP_OK ? err : ESP_ERR_NOT_SUPPORTED;
    }
    adc_oneshot_chan_cfg_t chan = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc1_shared_config_channel(dev->adc_ch, &chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: ADC channel config failed: %s", name, esp_err_to_name(err));
        return err;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cc = {
        .unit_id  = ADC_UNIT_1,
        .chan     = dev->adc_ch,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    dev->cali_ok = (adc_cali_create_scheme_curve_fitting(&cc, &dev->cali) == ESP_OK);
#endif

    dthr_init(&dev->thr, dev->cfg.fire_mv, dev->cfg.release_mv,
              (uint16_t)CONFIG_DISCRETE_IN_CONFIRM_SAMPLES);

    /* The check that matters most: can the ADC even see the threshold?
     * With the divider bypassed (or too weak), the fire level sits above
     * full scale and idle and fired both read as the top of the range. */
    uint32_t fire_pin_mv = line_to_pin_mv(dev->thr.fire_mv);
    if (fire_pin_mv >= DISCRETE_ADC_FULL_SCALE_MV) {
        ESP_LOGE(TAG, "%s: fire threshold %lu mV puts %lu mV on the pin -- ABOVE the "
                      "ADC's ~%d mV range. It can never be seen. Fit the divider, or "
                      "set its values in menuconfig.", name,
                 (unsigned long)dev->thr.fire_mv, (unsigned long)fire_pin_mv,
                 DISCRETE_ADC_FULL_SCALE_MV);
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_CONFIG_INVALID, ESP_OK, fire_pin_mv);
    }

    ESP_LOGI(TAG, "%s armed (ANALOG): GPIO%d = ADC1_CH%d, series %lu / to-ground %lu ohm, "
                  "fire >= %lu mV, release <= %lu mV, %d samples to confirm, %s, source=%s",
             name, (int)dev->cfg.input_pin, (int)dev->adc_ch,
             (unsigned long)DISCRETE_DIV_TOP, (unsigned long)DISCRETE_DIV_BOTTOM,
             (unsigned long)dev->thr.fire_mv, (unsigned long)dev->thr.release_mv,
             CONFIG_DISCRETE_IN_CONFIRM_SAMPLES,
             dev->cali_ok ? "calibrated" : "UNCALIBRATED (+/-5%)",
             dev->cfg.source ? dev->cfg.source : "UNSPECIFIED");
    return ESP_OK;
}
#endif /* CONFIG_DISCRETE_IN_MODE_ANALOG */

/* ===================================================================
 *  Public API
 * =================================================================== */

esp_err_t discrete_in_init(const discrete_in_config_t *cfg, discrete_in_handle_t *out)
{
    if (!cfg || !out || cfg->input_pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    struct discrete_in_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < DISCRETE_IN_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", DISCRETE_IN_MAX_INSTANCES);
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_INIT_FAILED, ESP_ERR_NO_MEM, 0);
        return ESP_ERR_NO_MEM;
    }

    dev->cfg = *cfg;

    /* Output first: the idle level must be established before the input
     * interrupt is armed, so there is never a window where the output
     * floats at an undefined level. GPIO_NUM_NC means this channel has
     * no output on this board -- it observes, it does not actuate. */
    if (dev->cfg.output_pin != GPIO_NUM_NC) {
        gpio_config_t out_io = {
            .pin_bit_mask = 1ULL << dev->cfg.output_pin,
            .mode         = GPIO_MODE_OUTPUT,
        };
        esp_err_t oerr = gpio_config(&out_io);
        if (oerr != ESP_OK) {
            ESP_LOGE(TAG, "output gpio_config failed: %s", esp_err_to_name(oerr));
            event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_INIT_FAILED, oerr,
                            (uint32_t)dev->cfg.output_pin);
            dev->in_use = false;
            return oerr;
        }
        gpio_set_level(dev->cfg.output_pin, DISCRETE_IN_OUTPUT_IDLE_LEVEL);
    }

#ifdef CONFIG_DISCRETE_IN_MODE_ANALOG
    {
        esp_err_t aerr = discrete_in_analog_init(dev);
        if (aerr != ESP_OK) {
            event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_INIT_FAILED, aerr,
                            (uint32_t)dev->cfg.input_pin);
            dev->in_use = false;
            return aerr;
        }
        dev->initialized = true;
        *out = dev;
        return ESP_OK;
    }
#endif

    /* Input -- idle LOW, rising edge. No internal pulldown: the board's
     * 3.6k divider leg already holds the line low while the Blue Raven
     * output is tri-stated, and adding ~45k in parallel would only
     * raise the source voltage needed for a valid HIGH. */
    gpio_config_t in_io = {
        .pin_bit_mask = 1ULL << dev->cfg.input_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        /* OFF on PCB v1: the board's divider already holds this low, and
         * the internal ~45k in parallel would shift the ratio from
         * 0.434 to 0.415 -- making a real Blue Raven trigger HARDER to
         * reach, not easier. ON only where the divider is absent, such
         * as a bare module on a breadboard, where the pin would float
         * and generate thousands of edges a second. */
#ifdef CONFIG_DISCRETE_IN_INTERNAL_PULLDOWN
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
#else
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#endif
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&in_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "input gpio_config failed: %s", esp_err_to_name(err));
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_INIT_FAILED, err,
                        (uint32_t)dev->cfg.input_pin);
        dev->in_use = false;
        return err;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_INIT_FAILED, err, 0);
        dev->in_use = false;
        return err;
    }

    dev->initialized = true;
    *out = dev;
    ESP_LOGI(TAG, "%s armed: input=GPIO%d (rising edge, external 3k6 pulldown) "
                  "output=%d  source=%s",
             dev->cfg.name ? dev->cfg.name : "discrete",
             (int)dev->cfg.input_pin, (int)dev->cfg.output_pin,
             dev->cfg.source ? dev->cfg.source : "UNSPECIFIED");
    return ESP_OK;
}

esp_err_t discrete_in_start(discrete_in_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->task) {
        return ESP_ERR_INVALID_STATE;
    }

#ifdef CONFIG_DISCRETE_IN_MODE_ANALOG
    dev->task = xTaskCreateStaticPinnedToCore(
        discrete_in_analog_task, dev->cfg.name ? dev->cfg.name : "discrete_in",
        DISCRETE_IN_STACK_WORDS, (void *)dev, priority, dev->stack, &dev->tcb, core_id);
    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_TASK_FAILED, ESP_FAIL, 0);
        return ESP_FAIL;
    }
    return ESP_OK;
#endif

    /* Task before ISR handler, not after. The ISR calls
     * vTaskNotifyGiveFromISR(dev->task) -- if the handler were attached
     * first, an edge arriving in the gap would notify a NULL task. */
    dev->task = xTaskCreateStaticPinnedToCore(
        discrete_in_task, dev->cfg.name ? dev->cfg.name : "discrete_in",
        DISCRETE_IN_STACK_WORDS, (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_TASK_FAILED, ESP_FAIL, 0);
        return ESP_FAIL;
    }

    esp_err_t err = gpio_isr_handler_add(dev->cfg.input_pin, discrete_in_isr, (void *)dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        event_log_error(ERR_SRC_DISCRETE_IN, ERR_CODE_INIT_FAILED, err,
                        (uint32_t)dev->cfg.input_pin);
        vTaskDelete(dev->task);
        dev->task = NULL;
        return err;
    }
    dev->isr_installed = true;
    return ESP_OK;
}

esp_err_t discrete_in_stop(discrete_in_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->isr_installed) {
        gpio_isr_handler_remove(dev->cfg.input_pin);
        dev->isr_installed = false;
    }
    if (dev->task) {
        vTaskDelete(dev->task);
        dev->task = NULL;
    }
    dev->initialized = false;
    dev->in_use      = false;
    return ESP_OK;
}

bool discrete_in_fired(discrete_in_handle_t dev)
{
    return dev ? dev->fired : false;
}

void discrete_in_get_diagnostics(discrete_in_handle_t dev, discrete_in_diag_t *out)
{
    if (!dev || !out) {
        return;
    }
    *out = dev->diag;
#ifdef CONFIG_DISCRETE_IN_MODE_ANALOG
    out->high = dev->thr.high;
#endif
}
