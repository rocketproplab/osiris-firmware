#ifndef EVENT_LOG_H
#define EVENT_LOG_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include <stddef.h>

#include "sensor_records.h"

/* ===================================================================
 *  Error and event recording.
 *
 *  A file-scope singleton with no handle, deliberately -- the same
 *  shape as data_router, and for the same reason. Every component in
 *  the project needs to be able to report a fault, and threading a
 *  handle through bmp388_read_once() so it can say "that SPI read
 *  failed" would put plumbing in every signature in the codebase to
 *  serve one call. There is exactly one flight, so exactly one log.
 *
 *  Records reach the card and the radio through a SINK the caller
 *  installs at init -- in practice data_router_publish, so they land in
 *  err.bin and event.bin AND go out over telemetry once the XBee TX
 *  stub is implemented. Events are the highest-value thing on that
 *  link: a dozen records that describe the whole flight.
 *
 *  The sink is a function pointer rather than a direct call to
 *  data_router because sd_logger reports its own faults through here,
 *  and sd_logger -> event_log -> data_router -> sd_logger is a
 *  dependency cycle the build system will not accept. Injecting it
 *  also means event_log has nothing to fail at init and no hardware
 *  of its own, so it can be the first thing brought up in main().
 *
 *  NOT ISR-SAFE. Both functions end in ring_buffer_push(), which
 *  sd_logger explicitly does not support from interrupt context. An
 *  interrupt that needs to record something captures the timestamp in
 *  the ISR and hands it to event_log_event_at() from its task -- see
 *  br_signal.c, which does exactly this so the recorded time is the
 *  edge, not the scheduling delay after it.
 * =================================================================== */

/* Minimum spacing between recorded errors from the same source. A
 * failing SPI sensor can produce an error every sample at 104 Hz, and
 * without this it would fill the error queue with 156 copies of one
 * fact and evict everything else. Suppressed errors are not lost --
 * they are counted, and the next record from that source carries the
 * count in err_record_t.suppressed. */
#define EVENT_LOG_MIN_INTERVAL_MS   100

/* The first few from any source always get through regardless of the
 * rate limit, so a burst at startup is never invisible. */
#define EVENT_LOG_BURST_ALLOWANCE   3

typedef struct {
    uint32_t errors_recorded;
    uint32_t errors_suppressed;
    uint32_t events_recorded;
    uint32_t dropped_no_sink;   // reported before the sink was installed
} event_log_diag_t;

/* Where records go. Matches data_router_publish's signature so it can
 * be passed straight in. */
typedef esp_err_t (*event_log_sink_t)(sensor_stream_t stream,
                                      const void *record, size_t len);

/* Installs the sink and zeroes the rate-limiter state.
 *
 * A NULL sink is legal and useful: calls still count in the
 * diagnostics but go nowhere. That is the state during early boot,
 * before the router exists -- reporting a fault then is not an error,
 * it just cannot be recorded yet. */
void event_log_init(event_log_sink_t sink);

/* Record a fault. `esp_err` is the esp_err_t that caused it, or ESP_OK
 * when there isn't one; `detail` is a free field -- a pin number, a
 * byte count, whatever makes the record diagnosable. */
void event_log_error(err_source_t source, err_code_t code,
                     esp_err_t esp_err, uint32_t detail);

/* Record a flight event. Never rate-limited: events are rare by
 * construction, and silently dropping one would corrupt the timeline
 * that is the whole point of the stream. */
void event_log_event(event_type_t type, uint8_t channel, uint32_t detail);

/* Same, with an externally captured timestamp -- for events whose real
 * time was recorded in an ISR before the task ran. */
void event_log_event_at(event_type_t type, uint8_t channel, uint32_t detail,
                        uint64_t timestamp_us);

void event_log_get_diagnostics(event_log_diag_t *out);

#endif // EVENT_LOG_H
