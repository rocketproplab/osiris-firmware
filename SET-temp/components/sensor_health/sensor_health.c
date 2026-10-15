#include "sensor_health.h"

#include <string.h>

#include "esp_log.h"
#include "event_log.h"

static const char *TAG = "SENSOR_HEALTH";

static const char *source_name(err_source_t s)
{
    switch (s) {
        case ERR_SRC_BMP388:   return "BMP388";
        case ERR_SRC_LSM6DSOX: return "LSM6DSOX";
        case ERR_SRC_GPS:      return "GPS";
        case ERR_SRC_BATTERY:  return "battery monitor";
        default:               return "sensor";
    }
}

const char *sensor_health_reason_name(uint8_t reason)
{
    switch (reason) {
        case SENSOR_LOST_NO_RESPONSE: return "no response";
        case SENSOR_LOST_RESET:       return "chip reset, configuration lost";
        case SENSOR_LOST_BUS_ERROR:   return "bus error";
        default:                      return "unknown";
    }
}

void sensor_health_init(sensor_health_t *h, err_source_t source, uint64_t now_us)
{
    memset(h, 0, sizeof(*h));
    h->source        = source;
    h->next_check_us = now_us + (uint64_t)SENSOR_HEALTH_CHECK_MS * 1000ULL;
}

bool sensor_health_check_due(sensor_health_t *h, uint64_t now_us)
{
    if (h->lost || now_us < h->next_check_us) {
        return false;
    }
    h->next_check_us = now_us + (uint64_t)SENSOR_HEALTH_CHECK_MS * 1000ULL;
    return true;
}

void sensor_health_lost(sensor_health_t *h, sensor_lost_reason_t why, uint64_t now_us)
{
    if (h->lost) {
        return;
    }
    h->lost          = true;
    h->last_reason   = (uint8_t)why;
    h->lost_at_us    = now_us;
    h->next_retry_us = now_us;            /* first attempt right away */
    h->next_announce_us = now_us + (uint64_t)SENSOR_HEALTH_REANNOUNCE_MS * 1000ULL;
    h->lost_count++;

    ESP_LOGE(TAG, "%s LOST (%s) -- not publishing, retrying every %d ms",
             source_name(h->source), sensor_health_reason_name((uint8_t)why),
             SENSOR_HEALTH_RETRY_MS);
    event_log_event(EV_SENSOR_LOST, (uint8_t)h->source, (uint32_t)why);
}

void sensor_health_poll(sensor_health_t *h, uint64_t now_us)
{
    if (!h->lost || now_us < h->next_announce_us) {
        return;
    }
    h->next_announce_us = now_us + (uint64_t)SENSOR_HEALTH_REANNOUNCE_MS * 1000ULL;
    ESP_LOGW(TAG, "%s still lost (%u ms)", source_name(h->source),
             (unsigned)((now_us - h->lost_at_us) / 1000ULL));
    event_log_event(EV_SENSOR_LOST, (uint8_t)h->source, (uint32_t)h->last_reason);
}

void sensor_health_clear(sensor_health_t *h, uint64_t now_us)
{
    h->lost          = false;
    h->next_check_us = now_us + (uint64_t)SENSOR_HEALTH_CHECK_MS * 1000ULL;
}

bool sensor_health_retry_due(const sensor_health_t *h, uint64_t now_us)
{
    return h->lost && now_us >= h->next_retry_us;
}

uint32_t sensor_health_retry_wait_ms(const sensor_health_t *h, uint64_t now_us)
{
    if (!h->lost || now_us >= h->next_retry_us) {
        return 0;
    }
    /* Rounded up, so a task that sleeps this long wakes at or after the
     * deadline rather than a fraction of a millisecond before it. */
    return (uint32_t)((h->next_retry_us - now_us + 999ULL) / 1000ULL);
}

void sensor_health_retry_result(sensor_health_t *h, bool ok, uint64_t now_us)
{
    if (!h->lost) {
        return;
    }
    h->retry_attempts++;

    if (!ok) {
        h->next_retry_us = now_us + (uint64_t)SENSOR_HEALTH_RETRY_MS * 1000ULL;
        sensor_health_poll(h, now_us);
        return;
    }

    uint64_t outage_us = now_us - h->lost_at_us;
    uint32_t outage_ms = outage_us / 1000ULL > UINT32_MAX ? UINT32_MAX
                                                          : (uint32_t)(outage_us / 1000ULL);
    h->lost           = false;
    h->last_outage_ms = outage_ms;
    h->recoveries++;
    h->next_check_us  = now_us + (uint64_t)SENSOR_HEALTH_CHECK_MS * 1000ULL;

    ESP_LOGW(TAG, "%s RECOVERED after %u ms (%u attempt(s) this outage)",
             source_name(h->source), (unsigned)outage_ms, (unsigned)h->retry_attempts);
    event_log_event(EV_SENSOR_RECOVERED, (uint8_t)h->source, outage_ms);
}
