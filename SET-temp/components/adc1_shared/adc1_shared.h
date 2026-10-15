#ifndef ADC1_SHARED_H
#define ADC1_SHARED_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

/* ===================================================================
 *  ADC1, shared.
 *
 *  Several things on this board read ADC1: the battery sense (GPIO20,
 *  CH4) and, in analog mode, the Blue Raven staging and recovery lines
 *  (GPIO22/23, CH6/CH7). ESP-IDF lets a unit be created ONCE -- a
 *  second adc_oneshot_new_unit() for ADC1 fails -- and its read
 *  function is documented as not thread-safe. So the unit lives here,
 *  created on first use, and every read goes through one mutex.
 *
 *  A read is a few microseconds; the lock is never held for longer.
 * =================================================================== */

/* Creates ADC1 on the first call; later calls return the same handle. */
esp_err_t adc1_shared_get(adc_oneshot_unit_handle_t *out);

esp_err_t adc1_shared_config_channel(adc_channel_t ch, const adc_oneshot_chan_cfg_t *cfg);

/* One conversion under the lock. ESP_ERR_TIMEOUT if another reader held
 * the lock for more than 10 ms, which would itself be a bug. */
esp_err_t adc1_shared_read(adc_channel_t ch, int *raw);

#endif /* ADC1_SHARED_H */
