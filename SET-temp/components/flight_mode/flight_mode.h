#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "sensor_records.h"

/* ===================================================================
 *  The vehicle's operating mode, and the rules for changing it.
 *
 *  A file-scope singleton with no handle -- same shape as data_router
 *  and event_log, and for the same reason: there is one vehicle, so
 *  there is one mode, and every task needs to read it without being
 *  handed a pointer.
 *
 *  WHY THIS IS A STATE MACHINE AND NOT A BOOLEAN
 *
 *  "Standby or flight" is how you describe it out loud, but the rule
 *  that actually matters -- may this request change the mode? -- has a
 *  different answer before liftoff, after liftoff, and after landing.
 *  A boolean would push that rule out to every caller, where one caller
 *  getting it wrong drops the vehicle out of flight mode mid-ascent.
 *  Here the rule lives in exactly one function.
 *
 *      power on
 *          |
 *          v
 *      STANDBY ---- arm (uplink or IMU) ----> ARMED
 *          ^                                    |
 *          |                              liftoff seen
 *          |                                    v
 *          |                                 FLIGHT   <-- disarm REFUSED
 *          |                                    |
 *          |                             landing detected
 *          |                                    v
 *          +---- disarm (uplink, now allowed) LANDED
 *
 *  Two details worth stating because they are easy to get backwards:
 *
 *  1. LANDED KEEPS FLIGHT RATES. Landing detection does not slow the
 *     vehicle down; it only unlocks the disarm command and announces
 *     that it has. Dropping to standby automatically would start
 *     thinning the record at the exact moment somebody is still
 *     deciding whether the vehicle actually landed or just went quiet.
 *
 *  2. The IMU wake-up can arm from STANDBY *and* skip straight past
 *     ARMED. If the accelerometer says liftoff, the vehicle is flying
 *     whether or not anyone armed it, so the mode goes STANDBY ->
 *     FLIGHT in one step and disarm locks immediately.
 * =================================================================== */

typedef struct {
    uint32_t arm_requests;        // accepted arms
    uint32_t disarm_requests;     // accepted disarms
    uint32_t disarm_refused;      // refused because the vehicle was flying
    uint32_t transitions;         // total mode changes
    uint64_t armed_at_us;         // first entry into ARMED or FLIGHT
    uint64_t liftoff_at_us;       // entry into FLIGHT
    uint64_t landed_at_us;        // entry into LANDED
} flight_mode_diag_t;

/* Sets the mode to STANDBY and clears the counters. Call once, early --
 * before any task that reads the mode is started. */
void flight_mode_init(void);

flight_mode_t flight_mode_get(void);

/* True whenever the vehicle should be running at full flight rates:
 * ARMED, FLIGHT and LANDED. Every driver's reconfiguration decision
 * keys on this one predicate rather than testing modes individually,
 * so adding a state cannot silently leave a sensor slow. */
bool flight_mode_is_fast(void);

/* True when a disarm from the ground would currently be accepted. This
 * is what STATUS_FLAG_DISARM_OK reports, and it is what greys out the
 * toggle in the ground station. */
bool flight_mode_disarm_allowed(void);

/* STANDBY -> ARMED. Ignored (and counted) in any other state. */
bool flight_mode_arm(mode_source_t src);

/* -> STANDBY. Accepted from STANDBY, ARMED and LANDED; REFUSED from
 * FLIGHT. Returns false when refused, and records EV_UPLINK_REJECTED so
 * the refusal is in the flight log rather than only in someone's
 * memory of what the ground station showed. */
bool flight_mode_disarm(mode_source_t src);

/* Liftoff observed. STANDBY or ARMED -> FLIGHT. Idempotent: calling it
 * every control-loop tick is the intended usage, since both liftoff
 * detectors are level-triggered latches rather than edges. */
bool flight_mode_note_liftoff(mode_source_t src);

/* Landing observed. FLIGHT -> LANDED. Idempotent, and deliberately does
 * NOT change any sensor rate -- see the note above. */
bool flight_mode_note_landed(mode_source_t src);

/* TEST BUILDS ONLY (menuconfig: Flight Computer Operating Mode -> Test).
 * Moves to `to` unconditionally -- including out of FLIGHT, which no
 * other path may do -- and records it as MODE_SRC_TEST. In a flight
 * build the body is compiled out: it refuses, logs, and returns false,
 * so the rule above cannot be bypassed by a stray call. */
bool flight_mode_test_force(flight_mode_t to);

void flight_mode_get_diagnostics(flight_mode_diag_t *out);

/* "STANDBY" / "ARMED" / "FLIGHT" / "LANDED", for log lines. */
const char *flight_mode_name(flight_mode_t m);

#endif // FLIGHT_MODE_H
