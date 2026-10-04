#include <stdio.h>
#include "discrete_in.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "event_log.h"

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

static void IRAM_ATTR discrete_in_isr(void *arg)
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

static void discrete_in_task(void *arg)
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
}
