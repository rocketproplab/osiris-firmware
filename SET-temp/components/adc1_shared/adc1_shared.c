#include "adc1_shared.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static adc_oneshot_unit_handle_t s_unit;
static SemaphoreHandle_t         s_lock;
static StaticSemaphore_t         s_lock_buf;
static portMUX_TYPE              s_init_mux = portMUX_INITIALIZER_UNLOCKED;
static esp_err_t                 s_init_err = ESP_OK;
static bool                      s_inited;

esp_err_t adc1_shared_get(adc_oneshot_unit_handle_t *out)
{
    /* The mutex is created under a spinlock so two first callers cannot
     * both create one; the unit itself is then created under the mutex. */
    portENTER_CRITICAL(&s_init_mux);
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }
    portEXIT_CRITICAL(&s_init_mux);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_inited) {
        adc_oneshot_unit_init_cfg_t cfg = {
            .unit_id  = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        s_init_err = adc_oneshot_new_unit(&cfg, &s_unit);
        s_inited = true;
    }
    esp_err_t err = s_init_err;
    xSemaphoreGive(s_lock);

    if (out) {
        *out = s_unit;
    }
    return err;
}

esp_err_t adc1_shared_config_channel(adc_channel_t ch, const adc_oneshot_chan_cfg_t *cfg)
{
    esp_err_t err = adc1_shared_get(NULL);
    if (err != ESP_OK) {
        return err;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    err = adc_oneshot_config_channel(s_unit, ch, cfg);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t adc1_shared_read(adc_channel_t ch, int *raw)
{
    if (!s_lock || !s_unit) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = adc_oneshot_read(s_unit, ch, raw);
    xSemaphoreGive(s_lock);
    return err;
}
