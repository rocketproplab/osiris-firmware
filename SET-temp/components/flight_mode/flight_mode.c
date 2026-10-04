#include "flight_mode.h"

#include <string.h>

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"      /* CONFIG_FC_MODE_TEST; absent in host tests */
#endif
#endif

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "event_log.h"

static const char *TAG = "MODE";

static flight_mode_t      s_mode = MODE_STANDBY;
static flight_mode_diag_t s_diag;
static portMUX_TYPE       s_lock = portMUX_INITIALIZER_UNLOCKED;

const char *flight_mode_name(flight_mode_t m)
{
    switch (m) {
        case MODE_STANDBY: return "STANDBY";
        case MODE_ARMED:   return "ARMED";
        case MODE_FLIGHT:  return "FLIGHT";
        case MODE_LANDED:  return "LANDED";
        default:           return "?";
    }
}

void flight_mode_init(void)
{
    portENTER_CRITICAL(&s_lock);
    s_mode = MODE_STANDBY;
    memset(&s_diag, 0, sizeof(s_diag));
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "flight mode initialised: STANDBY");
}

flight_mode_t flight_mode_get(void)
{
    /* No lock. A single aligned word, written under the lock and read
     * everywhere; the worst a concurrent reader sees is the previous
     * consistent value, never a torn one. Drivers poll this every loop,
     * so paying for a critical section here would be a real cost for no
     * correctness gain. */
    return s_mode;
}

bool flight_mode_is_fast(void)
{
    flight_mode_t m = s_mode;
    return (m == MODE_ARMED) || (m == MODE_FLIGHT) || (m == MODE_LANDED);
}

bool flight_mode_disarm_allowed(void)
{
    return s_mode != MODE_FLIGHT;
}

/* The single place a transition is committed. Caller holds no lock. */
static bool commit(flight_mode_t to, mode_source_t src, const char *why)
{
    uint64_t now = (uint64_t)esp_timer_get_time();
    flight_mode_t from;

    portENTER_CRITICAL(&s_lock);
    from = s_mode;
    if (from == to) {
        portEXIT_CRITICAL(&s_lock);
        return false;                 /* idempotent: not an error */
    }
    s_mode = to;
    s_diag.transitions++;
    if ((to == MODE_ARMED || to == MODE_FLIGHT) && s_diag.armed_at_us == 0) {
        s_diag.armed_at_us = now;
    }
    if (to == MODE_FLIGHT && s_diag.liftoff_at_us == 0) { s_diag.liftoff_at_us = now; }
    if (to == MODE_LANDED && s_diag.landed_at_us == 0)  { s_diag.landed_at_us = now; }
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGW(TAG, "%s -> %s  (%s, src=%d)",
             flight_mode_name(from), flight_mode_name(to), why, (int)src);

    /* channel carries the NEW mode, detail carries who asked. Recorded
     * outside the critical section: event_log ends in a ring push. */
    event_log_event(EV_MODE_CHANGED, (uint8_t)to, (uint32_t)src);
    return true;
}

bool flight_mode_arm(mode_source_t src)
{
    if (s_mode != MODE_STANDBY) {
        return false;
    }
    portENTER_CRITICAL(&s_lock);
    s_diag.arm_requests++;
    portEXIT_CRITICAL(&s_lock);
    return commit(MODE_ARMED, src, "armed");
}

bool flight_mode_disarm(mode_source_t src)
{
    /* The one rule this whole component exists to enforce. */
    if (s_mode == MODE_FLIGHT) {
        portENTER_CRITICAL(&s_lock);
        s_diag.disarm_refused++;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGE(TAG, "disarm REFUSED: vehicle is in FLIGHT (src=%d)", (int)src);
        /* Logged, not merely rejected. A refusal that leaves no trace is
         * indistinguishable afterwards from a command that never
         * arrived, and those two want very different investigations. */
        event_log_event(EV_UPLINK_REJECTED, (uint8_t)MODE_FLIGHT, (uint32_t)src);
        return false;
    }
    if (s_mode == MODE_STANDBY) {
        return false;                 /* already there */
    }
    portENTER_CRITICAL(&s_lock);
    s_diag.disarm_requests++;
    portEXIT_CRITICAL(&s_lock);
    return commit(MODE_STANDBY, src, "disarmed");
}

bool flight_mode_note_liftoff(mode_source_t src)
{
    flight_mode_t m = s_mode;
    /* STANDBY jumps straight to FLIGHT: if the accelerometer says the
     * vehicle is moving, it is flying whether or not anyone armed it,
     * and disarm should lock in the same instant. */
    if (m != MODE_STANDBY && m != MODE_ARMED) {
        return false;
    }
    return commit(MODE_FLIGHT, src, "liftoff detected");
}

bool flight_mode_note_landed(mode_source_t src)
{
    if (s_mode != MODE_FLIGHT) {
        return false;
    }
    bool changed = commit(MODE_LANDED, src, "landing detected");
    if (changed) {
        /* Separate from EV_MODE_CHANGED on purpose: this is the record
         * the ground station watches for to un-grey its toggle, and it
         * should be findable without decoding a mode enum. Rates are
         * deliberately NOT dropped here. */
        event_log_event(EV_LANDED, (uint8_t)MODE_LANDED, (uint32_t)src);
        ESP_LOGW(TAG, "LANDED -- disarm is now unlocked; rates stay at flight "
                      "config until the ground confirms");
    }
    return changed;
}

bool flight_mode_test_force(flight_mode_t to)
{
#ifdef CONFIG_FC_MODE_TEST
    return commit(to, MODE_SRC_TEST, "TEST sequence");
#else
    (void)to;
    ESP_LOGE(TAG, "flight_mode_test_force() in a FLIGHT build -- refused");
    return false;
#endif
}

void flight_mode_get_diagnostics(flight_mode_diag_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_diag;
    portEXIT_CRITICAL(&s_lock);
}
