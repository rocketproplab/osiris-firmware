#include "event_log.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"


static const char *TAG = "EVENTLOG";

/* Per-source rate-limiter state. Indexed by err_source_t, so adding a
 * source to the enum automatically gets its own budget rather than
 * sharing one with an unrelated component. */
typedef struct {
    uint64_t last_us;
    uint32_t suppressed;
    uint32_t seen;
} src_state_t;

static event_log_sink_t s_sink;
static src_state_t      s_src[ERR_SRC_COUNT];
static uint32_t         s_err_seq;
static uint32_t         s_evt_seq;
static event_log_diag_t s_diag;
static portMUX_TYPE     s_lock = portMUX_INITIALIZER_UNLOCKED;

void event_log_init(event_log_sink_t sink)
{
    portENTER_CRITICAL(&s_lock);
    s_sink = sink;
    memset(s_src, 0, sizeof(s_src));
    memset(&s_diag, 0, sizeof(s_diag));
    s_err_seq = 0;
    s_evt_seq = 0;
    portEXIT_CRITICAL(&s_lock);
}

void event_log_error(err_source_t source, err_code_t code,
                     esp_err_t esp_err, uint32_t detail)
{
    if (source >= ERR_SRC_COUNT) {
        source = ERR_SRC_SYSTEM;
    }

    uint64_t now = (uint64_t)esp_timer_get_time();
    bool     emit;
    uint32_t suppressed = 0;
    uint32_t seq = 0;

    portENTER_CRITICAL(&s_lock);
    src_state_t *st = &s_src[source];
    st->seen++;

    /* Always let the first few from a source through, then throttle.
     * The burst allowance matters at startup, where several distinct
     * init failures from one component would otherwise collapse into
     * a single record and hide which of them actually happened. */
    if (st->seen <= EVENT_LOG_BURST_ALLOWANCE ||
        (now - st->last_us) >= (uint64_t)EVENT_LOG_MIN_INTERVAL_MS * 1000ULL) {
        emit           = true;
        suppressed     = st->suppressed;
        st->suppressed = 0;
        st->last_us    = now;
        seq            = ++s_err_seq;
        s_diag.errors_recorded++;
    } else {
        emit = false;
        st->suppressed++;
        s_diag.errors_suppressed++;
    }
    portEXIT_CRITICAL(&s_lock);

    if (!emit) {
        return;
    }

    err_record_t rec = {
        .seq          = seq,
        .timestamp_us = now,
        .source       = (uint8_t)source,
        .code         = (uint8_t)code,
        .esp_err      = (int16_t)esp_err,
        .detail       = detail,
        .suppressed   = suppressed,
    };
    if (s_sink) {
        s_sink(STREAM_ERR, &rec, sizeof(rec));
    } else {
        portENTER_CRITICAL(&s_lock);
        s_diag.dropped_no_sink++;
        portEXIT_CRITICAL(&s_lock);
    }
}

void event_log_event_at(event_type_t type, uint8_t channel, uint32_t detail,
                        uint64_t timestamp_us)
{
    uint32_t seq;
    portENTER_CRITICAL(&s_lock);
    seq = ++s_evt_seq;
    s_diag.events_recorded++;
    portEXIT_CRITICAL(&s_lock);

    event_record_t rec = {
        .seq          = seq,
        .timestamp_us = timestamp_us,
        .type         = (uint8_t)type,
        .channel      = channel,
        .flags        = 0,
        .detail       = detail,
    };
    if (s_sink) {
        s_sink(STREAM_EVENT, &rec, sizeof(rec));
    } else {
        portENTER_CRITICAL(&s_lock);
        s_diag.dropped_no_sink++;
        portEXIT_CRITICAL(&s_lock);
    }

    /* Events are rare and each one matters, so unlike errors they are
     * also announced on the console. */
    ESP_LOGW(TAG, "EVENT type=%d ch=%u detail=%lu at %llu us",
             (int)type, (unsigned)channel,
             (unsigned long)detail, (unsigned long long)timestamp_us);
}

void event_log_event(event_type_t type, uint8_t channel, uint32_t detail)
{
    event_log_event_at(type, channel, detail, (uint64_t)esp_timer_get_time());
}

void event_log_get_diagnostics(event_log_diag_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_diag;
    portEXIT_CRITICAL(&s_lock);
}
