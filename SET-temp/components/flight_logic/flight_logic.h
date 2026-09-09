#ifndef DEPLOY_LOGIC_H
#define DEPLOY_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bmp388_sensor.h"
#include "lsm6dsox_sensor.h"

/* ===================================================================
 *  THIS COMPONENT CANNOT ACTUATE ANYTHING.
 *
 *  The Blue Raven fires every charge on this vehicle. PCB v1 has no
 *  deployment outputs at all -- its only two-pin connectors, J7 and J8,
 *  are INPUTS carrying the Blue Raven's recovery and staging lines in
 *  through dividers. Each of those Blue Raven outputs is split two
 *  ways, one branch to the actuation hardware and one to this board, so
 *  the ESP is a listener on a line somebody else drives.
 *
 *  So what is this for? It is a SECOND OPINION, computed from this
 *  board's own sensors and written to event.bin with its own
 *  timestamps. After a flight you lay this timeline next to the Blue
 *  Raven's and see whether the two agree. When they disagree, you have
 *  learned something before the disagreement mattered.
 *
 *  There is deliberately no GPIO in this component's interface, no pin
 *  number in deploy_channel_t, and no call anywhere in the .c file that
 *  can drive a level. That is not an oversight to be fixed later by
 *  adding a pin back -- if a board revision ever gains real outputs,
 *  the actuation belongs in its own component with its own arming
 *  interlocks, not bolted onto the evaluator.
 * =================================================================== */

// Must match the event keys used in the JSON config / host config editor.
typedef enum {
    EVT_AGL1 = 0, EVT_AGL2, EVT_FANG1, EVT_FANG2, EVT_FUTANG,
    EVT_TVAL1, EVT_TVAL2, EVT_VEL1, EVT_VEL2, EVT_BVEL1, EVT_BURNCOUNT,
    EVT_LIFTOFF, EVT_APOGEE, EVT_PINC,
    EVT_APOFIRED, EVT_MAINFIRED, EVT_R3FIRED, EVT_R4FIRED,
    EVT_COUNT   // must stay last
} deploy_event_t;

/* ===================================================================
 *  What each event means, and in what units.
 *
 *  The "1" / "2" suffix is a comparison direction, not two different
 *  quantities: 1 is BELOW the threshold, 2 is ABOVE it. That pattern
 *  was already set by AGL1/AGL2 and VEL1/VEL2 and the newly implemented
 *  events follow it, so a config authored against one pair reads the
 *  same way against the others.
 *
 *   AGL1   altitude above the pad  <  threshold   [feet]
 *   AGL2   altitude above the pad  >  threshold   [feet]
 *   FANG1  tilt from vertical      <  threshold   [degrees]
 *   FANG2  tilt from vertical      >  threshold   [degrees]
 *   FUTANG projected tilt at the moment this channel would actually
 *          reach its condition     <  threshold   [degrees]
 *   TVAL1  time since liftoff      <  threshold   [seconds]
 *   TVAL2  time since liftoff      >  threshold   [seconds]
 *   VEL1   |vertical speed|        <  threshold   [ft/s]
 *   VEL2   |vertical speed|        >  threshold   [ft/s]
 *   BVEL1  signed vertical speed   <  threshold   [ft/s]  (descending)
 *   BURNCOUNT  motor burns started >= threshold   [count]
 *   LIFTOFF    liftoff detected
 *   APOGEE     apogee detected
 *   PINC       pressure increasing (descending)
 *   APOFIRED / MAINFIRED / R3FIRED / R4FIRED
 *          that channel reached its condition at least 1.5 s ago
 *
 *  FUTANG IS THE ONE WORTH READING TWICE. It is not the current tilt;
 *  it is the tilt this vehicle is projected to have delay_ms from now,
 *  extrapolated from the current tilt rate -- "where will the nose be
 *  pointing when this charge would actually go off". It is written as
 *  a BELOW test because that is the direction you would AND into a fire
 *  condition ("still close enough to vertical"). Flip it in
 *  event_is_true() if your config means the opposite.
 *
 *  APOFIRED and friends refer to THIS board's channels reaching their
 *  conditions, not to the Blue Raven's real charges. The board does see
 *  the Blue Raven fire -- that arrives on J7/J8 through discrete_in and
 *  lands in event.bin as EV_STAGING_SIGNAL / EV_RECOVERY_SIGNAL -- but
 *  wiring those into these events would make this timeline a mirror of
 *  the Blue Raven's rather than an independent check of it, which is
 *  the entire point of computing it.
 * =================================================================== */

typedef struct {
    bool  primary_enabled[EVT_COUNT];    // which events are checked in the primary set
    bool  secondary_enabled[EVT_COUNT];  // which events are checked in the secondary set
    float thresholds[EVT_COUNT];
    bool  latch;
    int   delay_ms;
    bool  manual_arm;

    // Runtime state (managed internally, don't set these from JSON)
    bool     armed;
    bool     timing;          // conditions became true, delay countdown running
    uint32_t timing_start_ms;
    bool     triggered;       // conditions held for delay_ms; recorded, never actuated
    uint32_t triggered_time_ms;
} deploy_channel_t;

typedef struct {
    deploy_channel_t apo, main_chute, third, fourth;
} deploy_config_t;

/* ===================================================================
 *  Flight state, derived once per tick and shared by every channel.
 *
 *  Exposed because main.c's status_task needs the same numbers for the
 *  1 Hz status record, and recomputing altitude-above-pad there would
 *  mean two different definitions of "AGL" in one firmware.
 * =================================================================== */
typedef struct {
    bool     ground_calibrated;
    float    ground_altitude_m;   // pad reference, MSL
    float    altitude_agl_m;
    float    velocity_fps;        // signed; positive is climbing
    bool     liftoff;
    bool     apogee;
    bool     pressure_increasing;

    bool     tilt_valid;          // gyro bias settled and integration running
    float    tilt_deg;            // angle between the vehicle axis and the
                                  // direction that was "up" on the pad
    float    tilt_rate_dps;       // rate of change of tilt_deg
    uint8_t  burn_count;          // motor burns started since liftoff
    bool     burning;             // a burn is in progress right now
    uint32_t liftoff_ms;
} deploy_state_t;

// Loads and parses the JSON config (matching the host app's format) out of
// NVS into the given deploy_config_t. Expects the config to already be
// stored under namespace "flight_cfg", key "config_json" -- e.g. by the
// config_receiver code that saves incoming SET_CONFIG uploads from the host.
// Requires nvs_flash_init() to have already been called.
//
// If NVS has no saved config yet -- or the saved config can't be read,
// parsed, or is missing its "channels" object -- this does NOT fail.
// cfg is populated with safe defaults instead: every event disabled on
// every channel, which per set_all_true()'s "an empty set is never
// considered satisfied" rule means NO channel can ever reach its
// condition. Every return path leaves cfg fully and consistently
// initialized; the caller never needs to zero it separately or guard
// against garbage. Check the logs (WARN/ERROR) to tell "using defaults"
// apart from "loaded real config", since both return ESP_OK.
esp_err_t deploy_logic_load_config(deploy_config_t *cfg);

// Resets the flight-state estimator (ground calibration, liftoff/apogee
// latches, tilt integration, burn counter) and arms the channels that
// are not manual-arm. Call once at startup, after
// deploy_logic_load_config(). Configures no hardware -- this component
// owns no pins.
esp_err_t deploy_logic_init(deploy_config_t *cfg);

// Call once per flight-loop iteration with the latest sensor readings.
// Updates flight state and evaluates each channel. Channels that reach
// their conditions are recorded to event.bin; nothing is actuated.
void deploy_logic_update(deploy_config_t *cfg, const bmp388_data_t *baro,
                          const lsm6_data_t *imu, uint32_t now_ms);

// Snapshot of the derived flight state. Safe to call from another task:
// the fields are written by one task and read by others, and a torn
// read costs a stale status record, not a wrong decision.
void deploy_logic_get_state(deploy_state_t *out);

// Manually arm a channel that has manual_arm = true (e.g. an airstart).
// Channels with manual_arm = false are armed automatically at boot.
void deploy_logic_manual_arm(deploy_channel_t *channel);

#endif // DEPLOY_LOGIC_H
