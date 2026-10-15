#include "flight_logic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_check.h"
#include "event_log.h"
#include "flight_mode.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "nvs.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"      /* CONFIG_FC_* failover settings; absent in host builds */
#endif
#endif

/* Defaults for builds without the menuconfig options (the host tests and
 * the simulator). Keep equal to the Kconfig defaults in
 * main/Kconfig.projbuild -> "Flight Computer Sensor Failover". */
#ifndef CONFIG_FC_FAILOVER_MS
#define CONFIG_FC_FAILOVER_MS         2000
#endif
#ifndef CONFIG_FC_ALT_LIFTOFF_AGL_M
#define CONFIG_FC_ALT_LIFTOFF_AGL_M   30
#endif
#ifndef CONFIG_FC_GPS_STALE_MS
#define CONFIG_FC_GPS_STALE_MS        3000
#endif

static const char *TAG = "DEPLOY_LOGIC";

// Matches the namespace/key used by the config_receiver code that saves
// incoming SET_CONFIG uploads from the host app.
#define CONFIG_NVS_NAMESPACE   "flight_cfg"
#define CONFIG_NVS_KEY         "config_json"

/* ===================================================================
 *  NO OUTPUTS. NO PINS. NOTHING TO DRIVE.
 *
 *  This file used to carry four output-pin defines and a gpio_set_level()
 *  in the firing path. They are gone, along with driver/gpio.h. The Blue
 *  Raven fires every charge on this vehicle; PCB v1 has no deployment
 *  pads at all, and both of its two-pin connectors are inputs listening
 *  to lines the Blue Raven drives.
 *
 *  What remains is the evaluation, which is the part that was ever worth
 *  having here: an independent timeline computed from this board's own
 *  sensors, written to event.bin, to be compared against the Blue
 *  Raven's after the flight.
 *
 *  If you are here to add an output back: don't add it to this file. A
 *  board revision that gains real pyro channels wants a separate
 *  component with its own arming interlocks and its own continuity
 *  checks, so that the thing which decides and the thing which fires
 *  can be reviewed, and failed, independently.
 * =================================================================== */

// Event key strings, must exactly match the JSON field names used by the
// host config editor / test.json.
static const char *EVENT_KEYS[EVT_COUNT] = {
    "AGL1", "AGL2", "FANG1", "FANG2", "FUTANG",
    "TVAL1", "TVAL2", "VEL1", "VEL2", "BVEL1", "BURNCOUNT",
    "LIFTOFF", "APOGEE", "PINC",
    "APOFIRED", "MAINFIRED", "R3FIRED", "R4FIRED",
};

static const char *CHANNEL_NAMES[4] = { "Apo", "Main", "3rd", "4th" };

// ── Flight-level state, shared across all channels ──
static bool     s_liftoff_detected   = false;
static bool     s_apogee_detected    = false;
static bool     s_pressure_increasing = false;
static uint32_t s_liftoff_ms         = 0;

static float    s_ground_altitude    = 0.0f;
static bool     s_ground_calibrated  = false;
static float    s_altitude_history[10];
static int      s_altitude_hist_idx  = 0;
static int      s_altitude_hist_count = 0;

static float    s_baro_velocity      = 0.0f;   // ft/s, from altitude derivative
static float    s_altitude_agl_m     = 0.0f;
static float    s_prev_altitude      = 0.0f;
static uint32_t s_prev_time_ms       = 0;       // IMU timeline
static uint32_t s_prev_baro_ms       = 0;       // barometer timeline
static uint32_t s_last_baro_seq      = 0;
static bool     s_have_baro          = false;
static bool     s_have_prev_alt      = false;   // velocity has a previous sample to diff against

/* ---- Sensor failover (see flight_logic.h) ---- */
static uint32_t s_last_baro_ms       = 0;       // when the barometer last produced a NEW sample
static uint32_t s_last_imu_seq       = 0;
static bool     s_have_imu           = false;
static uint32_t s_last_imu_ms        = 0;
static bool     s_imu_failover       = false;   // IMU silent for the failover time, now
static bool     s_imu_degraded       = false;   // IMU was lost after liftoff: sticky
static uint32_t s_imu_lost_at_ms     = 0;
static int      s_alt_liftoff_n      = 0;       // consecutive altitude samples above the liftoff AGL
static deploy_alt_source_t s_alt_source = DEPLOY_ALT_BARO;
static deploy_vel_source_t s_vel_source = DEPLOY_VEL_BARO;

/* Vertical velocity from the accelerometer -- the barometer's backup for
 * velocity, apogee and PINC. Integrated from rest: it is held at zero on
 * the pad and starts at the first sample of the liftoff dwell, so the
 * boost captured while liftoff is still being confirmed is not lost. */
static float    s_imu_vel_ms         = 0.0f;    // m/s, + up
static bool     s_imu_vel_rose       = false;   // passed IMU_APOGEE_ARM_MS: a later <= 0 is apogee
static int      s_imu_apogee_n       = 0;
static float    s_g_pad              = 9.80665f; // |accel| measured on the pad
static int      s_cal_imu_n          = 0;

#define G_STD                     9.80665f
#define IMU_APOGEE_ARM_MS         10.0f   // must have climbed faster than this first
#define IMU_APOGEE_SUSTAIN        5       // samples at or below zero (~50 ms at 104 Hz)

/* GPS altitude, and the two ways of turning it into "above the pad".
 * PAD: an average of the fixes seen before liftoff -- the GPS's own idea
 *      of the pad. Preferred: it is independent of the barometer, which
 *      is the point when the barometer is the thing that failed.
 * FLIGHT: learned in flight as (GPS MSL - baro AGL) while both work, for
 *      a launch the GPS was not on for (straight out of STANDBY, where
 *      the GPS is powered down). Only used when there is no PAD ref. */
static uint32_t s_last_gps_seq       = 0;
static bool     s_have_gps           = false;
static uint32_t s_last_gps_ms        = 0;
static float    s_gps_pad_msl        = 0.0f;
static int      s_gps_pad_n          = 0;
static float    s_gps_flt_offset     = 0.0f;
static int      s_gps_flt_n          = 0;

#define GPS_REF_MIN_FIXES         5       // fixes before a reference is trusted
#define GPS_REF_ALPHA             0.2f    // EMA weight: ~the last 5 fixes dominate

/* Apogee and liftoff need a few consecutive agreeing samples, and "a
 * few" depends on the sample rate. The barometer values are the ones
 * this file always used (50 Hz). The GPS gets far fewer because each of
 * its samples is a second long: a 10-sample window would put apogee
 * ~15 s late. */
#define GPS_APOGEE_WINDOW         3       // newest vs 2 fixes earlier (2 s span)
#define GPS_APOGEE_SUSTAIN        2
#define ALT_LIFTOFF_SAMPLES_BARO  3
#define ALT_LIFTOFF_SAMPLES_GPS   2

/* A gap longer than this between IMU samples is an outage, not a sample
 * interval. Tilt already refuses to integrate across it; the dwell
 * timers must not count it either, or one thrust sample after an outage
 * would be credited with the whole outage's worth of burn. */
#define IMU_GAP_MS                500

/* In STANDBY the barometer runs at 1.5 Hz and, with a quiet INT line,
 * polls every 2 s -- a 2 s failover there would flap on every sample.
 * Failover is a flight feature, so standby gets a floor. */
#define STANDBY_FAILOVER_FLOOR_MS 5000

/* EV_LIFTOFF_SW channel: which detector called it. */
#define LIFTOFF_SRC_ACCEL         0
#define LIFTOFF_SRC_ALTITUDE      1

/* ===================================================================
 *  Dwell times, not sample counts.
 *
 *  These were sample counts until the loop moved onto the IMU clock.
 *  A count is only a duration if the rate is fixed, and this vehicle's
 *  IMU runs at 12.5 Hz in standby and 104 Hz in flight -- an 8x swing,
 *  which would have turned "three samples" into anything from 29 ms to
 *  240 ms depending on a mode this file cannot see.
 *
 *  Accumulating elapsed milliseconds instead makes every detector
 *  behave the same at any rate. The minimum sample count is kept
 *  alongside it so that a single outlier can never satisfy a dwell on
 *  its own, however long the interval it arrived on.
 * =================================================================== */
#define LIFTOFF_ACCEL_THRESHOLD_MS2   20.0f   // net accel above this = liftoff
#define LIFTOFF_SUSTAIN_MS            60      // was 3 samples at 50 Hz
#define DWELL_MIN_SAMPLES             2
static uint32_t s_liftoff_accum_ms   = 0;
static int      s_liftoff_samples    = 0;

#define APOGEE_SUSTAIN_SAMPLES        5       // consecutive descending BARO samples
static int s_apogee_sustain_count = 0;

/* ===================================================================
 *  Tilt tracking -- what FANG1, FANG2 and FUTANG are built on.
 *
 *  THE IDEA. A direction that is fixed in the world -- "up", as it was
 *  on the pad -- does not stay fixed in the vehicle's own frame once the
 *  vehicle rotates. Track that one vector in body coordinates and the
 *  angle between it and the airframe is the tilt, with no magnetometer
 *  and no GPS.
 *
 *  A vector u that is constant in the inertial frame evolves in body
 *  coordinates as du/dt = -(omega x u), where omega is the gyro reading.
 *  So each tick we rotate u by the small angle the vehicle just turned
 *  through, and renormalise to stop numerical drift from changing its
 *  length.
 *
 *  WHY THE REFERENCE AXIS IS MEASURED, NOT ASSUMED. s_axis is whatever
 *  direction the accelerometer called "up" while the vehicle sat on the
 *  pad. It is therefore the airframe's long axis as actually mounted --
 *  the code never needs to know whether the board's +Z, -X or something
 *  diagonal points at the sky, and a board mounted sideways in the next
 *  revision needs no change here. The cost is that the pad must be
 *  roughly vertical at calibration, which it is.
 *
 *  WHAT WILL EVENTUALLY BREAK IT. Integrated gyro drifts, and nothing
 *  here corrects it -- there is no second reference during flight. Bias
 *  is estimated before liftoff and frozen, which removes the constant
 *  part; what is left is random walk. Over a 30 s flight that is small.
 *  Over a long coast it is not, and this is NOT a substitute for a real
 *  attitude filter. It is honest for the first minute of flight, which
 *  is the part these events are asked about.
 *
 *  THE STANDBY INTERACTION. In STANDBY the LSM6DSOX's gyro is powered
 *  down and the driver reports explicit zeros. The bias estimator below
 *  runs continuously until liftoff, so when ARM powers the gyro back up
 *  it re-converges on the real bias within its window instead of
 *  carrying the zeros forward. That is why bias is a running estimate
 *  rather than a one-shot at ground calibration -- a one-shot taken in
 *  standby would be an estimate of nothing.
 * =================================================================== */
#define GYRO_BIAS_ALPHA           0.02f   // ~2 s of settling at 25 Hz
#define GYRO_BIAS_MIN_SAMPLES     50
#define GYRO_AT_REST_DPS          20.0f   // above this, don't feed the estimator
#define TILT_RATE_ALPHA           0.2f    // light smoothing on the tilt derivative
#define FUTANG_DEFAULT_HORIZON_MS 1000

static float s_axis[3]        = {0, 0, 1};  // airframe axis, body frame, unit
static float s_up[3]          = {0, 0, 1};  // pad "up", body frame, unit
static float s_gyro_bias[3]   = {0, 0, 0};  // dps
static int   s_gyro_bias_n    = 0;
static bool  s_bias_frozen    = false;
static bool  s_tilt_valid     = false;
static float s_tilt_deg       = 0.0f;
static float s_tilt_rate_dps  = 0.0f;
static float s_prev_tilt_deg  = 0.0f;
static bool  s_gyro_live      = false;  // last observed flight_mode_is_fast()

/* ===================================================================
 *  Burn counting -- what BURNCOUNT is built on.
 *
 *  A burn is a sustained period of high acceleration ALONG THE AIRFRAME
 *  AXIS, measured against s_axis rather than against the raw vector
 *  magnitude. The axis matters: magnitude cannot tell thrust from a
 *  hard lateral jolt, and it reads a healthy ~1 g while the vehicle
 *  sits on the pad, whereas the axial projection is what a motor
 *  actually pushes on.
 *
 *  The two thresholds are deliberately different. Entering a burn takes
 *  more acceleration than leaving one, so the chatter around a single
 *  threshold cannot count one motor twice -- the same hysteresis a
 *  Schmitt trigger uses, for the same reason.
 *
 *  THE COUNT INCREMENTS AT IGNITION, not at burnout, because that is
 *  the plain reading of "how many burns have there been" and it is what
 *  makes BURNCOUNT >= 2 mean "the sustainer has lit". Burnout is still
 *  recorded in event.bin as its own entry, so the burn's duration is
 *  recoverable from the log.
 *
 *  This is the vehicle's own opinion, from the accelerometer alone. It
 *  deliberately does not consult the Blue Raven's staging line on J8 --
 *  that arrives separately as EV_STAGING_SIGNAL, and keeping the two
 *  independent is what lets a post-flight review notice they disagreed.
 * =================================================================== */
#define BURN_ENTER_ACCEL_MS2      25.0f   // ~2.5 g axial: thrusting
#define BURN_EXIT_ACCEL_MS2       12.0f   // ~1.2 g axial: no longer thrusting
#define BURN_ENTER_MS             60      // was 3 samples at 50 Hz
#define BURN_EXIT_MS              100     // was 5 samples at 50 Hz
#define BURN_MAX_COUNT            255

/* Ground calibration accumulators. Kept at file scope rather than as
 * function statics so deploy_logic_init() can reset them -- function
 * statics would survive a re-init and quietly poison the second
 * calibration with the first one's samples. */
static float s_cal_alt_sum = 0.0f;
static float s_cal_acc_sum[3] = {0, 0, 0};
static int   s_cal_count = 0;
#define CAL_SAMPLES 20

static uint8_t  s_burn_count     = 0;
static bool     s_burning        = false;
static uint32_t s_burn_enter_ms  = 0;
static uint32_t s_burn_exit_ms   = 0;
static int      s_burn_enter_n   = 0;
static int      s_burn_exit_n    = 0;

/* ===================================================================
 *  Publishing the state to other tasks.
 *
 *  The estimator's statics above are written by exactly one task
 *  (flight_logic_task) and are read only by this file, so they need no
 *  lock. status_task, on the other core, needs the same numbers -- and
 *  it must not see half of one tick and half of the next.
 *
 *  So the tick ENDS by copying its results into s_snapshot under a
 *  spinlock. That critical section is a struct assignment and nothing
 *  else: no logging, no ring-buffer push, no call that could take
 *  another lock. Holding a portMUX across event_log_event() -- which
 *  ends in ring_buffer_push() taking a different spinlock -- would be a
 *  lock-ordering bug waiting for the one interleaving that deadlocks
 *  the flight computer, and it would do it with interrupts disabled at
 *  104 Hz.
 * =================================================================== */
/* Velocity and "descending" from whichever source is live -- see
 * deploy_vel_source_t. Defined up here because publish_state() reports
 * the same numbers the channels are evaluated on. */
static float current_vel_fps(void)
{
    switch (s_vel_source) {
        case DEPLOY_VEL_IMU:  return s_imu_vel_ms * 3.28084f;
        case DEPLOY_VEL_BARO:
        case DEPLOY_VEL_GPS:  return s_baro_velocity;   /* altitude derivative */
        default:              return 0.0f;
    }
}

static bool currently_descending(void)
{
    switch (s_vel_source) {
        case DEPLOY_VEL_IMU:  return s_imu_vel_ms < 0.0f;
        case DEPLOY_VEL_BARO:
        case DEPLOY_VEL_GPS:  return s_pressure_increasing;
        default:              return false;
    }
}

static portMUX_TYPE  s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static deploy_state_t s_snapshot;

static void publish_state(void)
{
    deploy_state_t s = {
        .ground_calibrated   = s_ground_calibrated,
        .ground_altitude_m   = s_ground_altitude,
        .altitude_agl_m      = s_altitude_agl_m,
        .velocity_fps        = current_vel_fps(),
        .liftoff             = s_liftoff_detected,
        .apogee              = s_apogee_detected,
        .pressure_increasing = currently_descending(),
        .tilt_valid          = s_tilt_valid,
        .tilt_deg            = s_tilt_deg,
        .tilt_rate_dps       = s_tilt_rate_dps,
        .burn_count          = s_burn_count,
        .burning             = s_burning,
        .liftoff_ms          = s_liftoff_ms,
        .alt_source          = (uint8_t)s_alt_source,
        .vel_source          = (uint8_t)s_vel_source,
        .imu_velocity_mps    = s_imu_vel_ms,
        .imu_failover        = s_imu_failover,
        .imu_degraded        = s_imu_degraded,
        .gps_ref_valid       = (s_gps_pad_n >= GPS_REF_MIN_FIXES ||
                                s_gps_flt_n >= GPS_REF_MIN_FIXES),
    };
    portENTER_CRITICAL(&s_state_lock);
    s_snapshot = s;
    portEXIT_CRITICAL(&s_state_lock);
}

// ── Small vector helpers ──

static inline float v_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void v_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/* Returns false for a vector too short to have a meaningful direction --
 * which is what free fall looks like to an accelerometer, and is exactly
 * when normalising would amplify noise into a confident wrong answer. */
static bool v_normalize(float v[3])
{
    float n = sqrtf(v_dot(v, v));
    if (n < 1e-4f) { return false; }
    v[0] /= n; v[1] /= n; v[2] /= n;
    return true;
}

// ── JSON parsing helpers ──

// NULL-safe: cJSON_GetObjectItem(NULL, key) returns NULL by design, and
// every lookup below is already guarded with `if (primary)` etc., so
// passing channel_json == NULL produces a fully-defaulted channel
// (every event disabled, latch=false, delay_ms=1500, manual_arm=false
// -> armed=true) instead of a special case. This is what makes the
// "no config in NVS" and "this channel's key was missing from otherwise
// valid JSON" fallbacks below identical and safe.
static void parse_channel_from_json(cJSON *channel_json, deploy_channel_t *ch)
{
    memset(ch, 0, sizeof(*ch));

    cJSON *primary = cJSON_GetObjectItem(channel_json, "primary");
    cJSON *secondary = cJSON_GetObjectItem(channel_json, "secondary");
    cJSON *thresholds = cJSON_GetObjectItem(channel_json, "thresholds");

    for (int i = 0; i < EVT_COUNT; i++) {
        if (primary) {
            cJSON *item = cJSON_GetObjectItem(primary, EVENT_KEYS[i]);
            if (item && cJSON_IsTrue(item)) ch->primary_enabled[i] = true;
        }
        if (secondary) {
            cJSON *item = cJSON_GetObjectItem(secondary, EVENT_KEYS[i]);
            if (item && cJSON_IsTrue(item)) ch->secondary_enabled[i] = true;
        }
        if (thresholds) {
            cJSON *item = cJSON_GetObjectItem(thresholds, EVENT_KEYS[i]);
            if (item && cJSON_IsNumber(item)) ch->thresholds[i] = (float)item->valuedouble;
        }
    }

    cJSON *latch = cJSON_GetObjectItem(channel_json, "latch");
    ch->latch = latch ? cJSON_IsTrue(latch) : false;

    cJSON *delay = cJSON_GetObjectItem(channel_json, "delay_ms");
    ch->delay_ms = delay ? delay->valueint : 1500;

    cJSON *arm = cJSON_GetObjectItem(channel_json, "manual_arm");
    ch->manual_arm = arm ? cJSON_IsTrue(arm) : false;

    cJSON *skip = cJSON_GetObjectItem(channel_json, "skip_imu_if_lost");
    ch->skip_imu_if_lost = skip ? cJSON_IsTrue(skip) : false;

    ch->armed = !ch->manual_arm;   // deployment channels auto-arm at boot
    ch->timing = false;
    ch->triggered = false;
}

// Fills every channel with the same safe defaults parse_channel_from_json
// produces for an empty/missing JSON object. Used by every "config isn't
// usable" fallback below so they all leave cfg in exactly the same,
// fully-defined, inert state.
static void apply_default_config(deploy_config_t *cfg)
{
    parse_channel_from_json(NULL, &cfg->apo);
    parse_channel_from_json(NULL, &cfg->main_chute);
    parse_channel_from_json(NULL, &cfg->third);
    parse_channel_from_json(NULL, &cfg->fourth);
}

/* Logs which events a loaded channel actually depends on. Worth the few
 * lines: the commonest way to get a surprising flight is a config that
 * enables an event nobody remembered enabling, and this puts the answer
 * in the boot log instead of requiring the JSON to be dug back out of
 * NVS afterwards. */
static void log_channel_summary(const char *name, const deploy_channel_t *ch)
{
    char primary[192] = {0};
    char secondary[192] = {0};
    for (int i = 0; i < EVT_COUNT; i++) {
        char frag[32];
        if (ch->primary_enabled[i]) {
            snprintf(frag, sizeof frag, "%s%s(%.1f)",
                     primary[0] ? " " : "", EVENT_KEYS[i], ch->thresholds[i]);
            strncat(primary, frag, sizeof(primary) - strlen(primary) - 1);
        }
        if (ch->secondary_enabled[i]) {
            snprintf(frag, sizeof frag, "%s%s(%.1f)",
                     secondary[0] ? " " : "", EVENT_KEYS[i], ch->thresholds[i]);
            strncat(secondary, frag, sizeof(secondary) - strlen(secondary) - 1);
        }
    }
    ESP_LOGI(TAG, "  %-4s delay=%dms latch=%d arm=%s imu-lost=%s", name, ch->delay_ms,
             (int)ch->latch, ch->manual_arm ? "manual" : "auto",
             ch->skip_imu_if_lost ? "skip tilt/burn conditions" : "block");
    ESP_LOGI(TAG, "       primary  : %s", primary[0] ? primary : "(none -- can never trigger)");
    ESP_LOGI(TAG, "       secondary: %s", secondary[0] ? secondary : "(none -- can never trigger)");
}

esp_err_t deploy_logic_load_config(deploy_config_t *cfg)
{
    // Known baseline no matter what happens below -- every early return
    // in this function goes through apply_default_config() instead of
    // leaving cfg as whatever garbage was on the caller's stack.
    memset(cfg, 0, sizeof(*cfg));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No NVS namespace '%s' yet (0x%x) -- no config uploaded? "
                      "Using safe defaults: every channel disabled.",
                 CONFIG_NVS_NAMESPACE, err);
        apply_default_config(cfg);
        return ESP_OK;
    }

    size_t required_size = 0;
    err = nvs_get_str(handle, CONFIG_NVS_KEY, NULL, &required_size);
    if (err != ESP_OK || required_size == 0) {
        ESP_LOGW(TAG, "No config found under key '%s' (0x%x) -- using safe defaults",
                 CONFIG_NVS_KEY, err);
        nvs_close(handle);
        apply_default_config(cfg);
        return ESP_OK;
    }

    char *buf = malloc(required_size);
    if (!buf) {
        ESP_LOGE(TAG, "malloc(%u) failed loading config -- using safe defaults "
                      "rather than failing boot over it",
                 (unsigned)required_size);
        nvs_close(handle);
        apply_default_config(cfg);
        return ESP_OK;
    }

    err = nvs_get_str(handle, CONFIG_NVS_KEY, buf, &required_size);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read config string from NVS (0x%x) -- using safe defaults", err);
        free(buf);
        apply_default_config(cfg);
        return ESP_OK;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse config JSON from NVS -- using safe defaults");
        apply_default_config(cfg);
        return ESP_OK;
    }

    cJSON *channels = cJSON_GetObjectItem(root, "channels");
    if (!channels) {
        ESP_LOGE(TAG, "Config JSON missing 'channels' object -- using safe defaults");
        cJSON_Delete(root);
        apply_default_config(cfg);
        return ESP_OK;
    }

    // Unconditional: cJSON_GetObjectItem(channels, "Apo") returns NULL if
    // the key is missing, and parse_channel_from_json already handles a
    // NULL channel_json correctly -- so one code path covers both "key
    // present" and "key missing from otherwise-valid JSON", and a missing
    // key lands on exactly the same defaults as no config at all.
    parse_channel_from_json(cJSON_GetObjectItem(channels, "Apo"), &cfg->apo);
    parse_channel_from_json(cJSON_GetObjectItem(channels, "Main"), &cfg->main_chute);
    parse_channel_from_json(cJSON_GetObjectItem(channels, "3rd"), &cfg->third);
    parse_channel_from_json(cJSON_GetObjectItem(channels, "4th"), &cfg->fourth);

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Config loaded from NVS ('%s'/'%s'):", CONFIG_NVS_NAMESPACE, CONFIG_NVS_KEY);
    log_channel_summary(CHANNEL_NAMES[0], &cfg->apo);
    log_channel_summary(CHANNEL_NAMES[1], &cfg->main_chute);
    log_channel_summary(CHANNEL_NAMES[2], &cfg->third);
    log_channel_summary(CHANNEL_NAMES[3], &cfg->fourth);
    return ESP_OK;
}

esp_err_t deploy_logic_init(deploy_config_t *cfg)
{
    (void)cfg;   // channels arm themselves in parse_channel_from_json()

    /* Statics, reset explicitly rather than trusted to be zero. This
     * function is cheap to call twice and a half-reset estimator is a
     * much worse failure than a redundant assignment. */
    s_liftoff_detected = false;
    s_apogee_detected = false;
    s_pressure_increasing = false;
    s_liftoff_ms = 0;
    s_ground_altitude = 0.0f;
    s_ground_calibrated = false;
    s_altitude_hist_idx = 0;
    s_altitude_hist_count = 0;
    s_baro_velocity = 0.0f;
    s_altitude_agl_m = 0.0f;
    s_prev_altitude = 0.0f;
    s_prev_time_ms = 0;
    s_prev_baro_ms = 0;
    s_last_baro_seq = 0;
    s_have_baro = false;
    s_have_prev_alt = false;
    s_last_baro_ms = 0;
    s_last_imu_seq = 0;
    s_have_imu = false;
    s_last_imu_ms = 0;
    s_imu_failover = false;
    s_imu_degraded = false;
    s_imu_lost_at_ms = 0;
    s_alt_liftoff_n = 0;
    s_alt_source = DEPLOY_ALT_BARO;
    s_vel_source = DEPLOY_VEL_BARO;
    s_imu_vel_ms = 0.0f;
    s_imu_vel_rose = false;
    s_imu_apogee_n = 0;
    s_g_pad = G_STD;
    s_cal_imu_n = 0;
    s_last_gps_seq = 0;
    s_have_gps = false;
    s_last_gps_ms = 0;
    s_gps_pad_msl = 0.0f;
    s_gps_pad_n = 0;
    s_gps_flt_offset = 0.0f;
    s_gps_flt_n = 0;
    s_liftoff_accum_ms = 0;
    s_liftoff_samples = 0;
    s_apogee_sustain_count = 0;

    s_axis[0] = s_axis[1] = 0.0f; s_axis[2] = 1.0f;
    s_up[0] = s_up[1] = 0.0f; s_up[2] = 1.0f;
    s_gyro_bias[0] = s_gyro_bias[1] = s_gyro_bias[2] = 0.0f;
    s_gyro_bias_n = 0;
    s_bias_frozen = false;
    s_gyro_live = false;
    s_tilt_valid = false;
    s_tilt_deg = 0.0f;
    s_tilt_rate_dps = 0.0f;
    s_prev_tilt_deg = 0.0f;

    s_burn_count = 0;
    s_burning = false;
    s_burn_enter_ms = 0;
    s_burn_exit_ms = 0;
    s_burn_enter_n = 0;
    s_burn_exit_n = 0;

    s_cal_alt_sum = 0.0f;
    s_cal_acc_sum[0] = s_cal_acc_sum[1] = s_cal_acc_sum[2] = 0.0f;
    s_cal_count = 0;

    publish_state();

    ESP_LOGI(TAG, "Flight logic initialized. NO OUTPUTS -- this board listens to "
                  "the Blue Raven and records its own opinion; it cannot actuate.");
    return ESP_OK;
}

void deploy_logic_manual_arm(deploy_channel_t *channel)
{
    channel->armed = true;
    ESP_LOGI(TAG, "Channel manually armed");
}

void deploy_logic_get_state(deploy_state_t *out)
{
    if (!out) { return; }
    portENTER_CRITICAL(&s_state_lock);
    *out = s_snapshot;
    portEXIT_CRITICAL(&s_state_lock);
}

// ── Flight-state estimation ──

/* ===================================================================
 *  Is the gyro actually running?
 *
 *  In STANDBY the LSM6DSOX's gyro is powered down and the driver
 *  publishes explicit zeros rather than the stale contents of the
 *  output registers. Those zeros are indistinguishable from a
 *  perfectly still, perfectly unbiased gyro -- they sail through the
 *  at-rest gate below and the estimator happily converges on them.
 *
 *  That was a latent defect while the loop ran on the barometer, and
 *  moving to the IMU clock made it live: the tilt integrator's dt is
 *  now 80 ms in standby rather than 667 ms, so it no longer trips the
 *  implausible-gap guard, and s_tilt_valid could go true on the
 *  strength of samples taken from a switched-off sensor.
 *
 *  So the estimator asks the one predicate the whole firmware uses for
 *  this, and throws away everything it learned when the answer changes
 *  from false to true. Everything learned in standby was learned from
 *  a sensor that was not running.
 * =================================================================== */
static void update_gyro_liveness(void)
{
    bool live = flight_mode_is_fast();
    if (live == s_gyro_live) { return; }
    s_gyro_live = live;
    if (!live) { return; }

    s_gyro_bias[0] = s_gyro_bias[1] = s_gyro_bias[2] = 0.0f;
    s_gyro_bias_n  = 0;
    s_tilt_valid   = false;
    ESP_LOGI(TAG, "gyro live -- bias estimator restarted, tilt unavailable "
                  "for %d samples", GYRO_BIAS_MIN_SAMPLES);
}

static void update_gyro_bias(const lsm6_data_t *imu)
{
    if (s_bias_frozen || !s_gyro_live) { return; }

    /* Only feed the estimator when the vehicle looks still. A hand-carry
     * to the pad is not bias, and averaging it in would tilt the whole
     * flight's attitude estimate by however far the walk rotated it. */
    if (fabsf(imu->gx) > GYRO_AT_REST_DPS ||
        fabsf(imu->gy) > GYRO_AT_REST_DPS ||
        fabsf(imu->gz) > GYRO_AT_REST_DPS) {
        return;
    }

    s_gyro_bias[0] += GYRO_BIAS_ALPHA * (imu->gx - s_gyro_bias[0]);
    s_gyro_bias[1] += GYRO_BIAS_ALPHA * (imu->gy - s_gyro_bias[1]);
    s_gyro_bias[2] += GYRO_BIAS_ALPHA * (imu->gz - s_gyro_bias[2]);
    if (s_gyro_bias_n < GYRO_BIAS_MIN_SAMPLES * 4) { s_gyro_bias_n++; }
}

static void update_tilt(const lsm6_data_t *imu, float dt_s)
{
    /* The IMU dropped out after liftoff. Whatever the vehicle rotated
     * through during the outage is simply missing from s_up, and there
     * is no in-flight reference to recover it from -- so tilt stays
     * invalid for the rest of the flight, even once samples return. */
    if (s_imu_degraded) { s_tilt_valid = false; return; }

    /* No gyro, nothing to integrate. Integrating the driver's standby
     * zeros would not move the estimate, but it WOULD let the validity
     * latch below fire on samples the sensor never took. */
    if (!s_gyro_live) { return; }

    if (dt_s <= 0.0f || dt_s > 0.5f) {
        /* A gap this long means the loop stalled or the clock jumped.
         * Integrating across it would rotate the reference by a
         * fabricated amount, so skip the step and keep the last good
         * attitude rather than inventing a new one. */
        return;
    }

    const float DEG2RAD = 0.01745329252f;
    float w[3] = {
        (imu->gx - s_gyro_bias[0]) * DEG2RAD,
        (imu->gy - s_gyro_bias[1]) * DEG2RAD,
        (imu->gz - s_gyro_bias[2]) * DEG2RAD,
    };

    /* A vector fixed in the world, seen from a rotating body, turns the
     * other way: du/dt = -(w x u). */
    float wxu[3];
    v_cross(w, s_up, wxu);
    s_up[0] -= wxu[0] * dt_s;
    s_up[1] -= wxu[1] * dt_s;
    s_up[2] -= wxu[2] * dt_s;
    if (!v_normalize(s_up)) {
        /* Should be unreachable -- the update is a small perturbation of
         * a unit vector -- but if it ever happens, a zero-length
         * reference would make every later acosf() garbage. Re-seed and
         * say so rather than reporting confident nonsense. */
        s_up[0] = s_axis[0]; s_up[1] = s_axis[1]; s_up[2] = s_axis[2];
        s_tilt_valid = false;
        event_log_error(ERR_SRC_FLIGHT_LOGIC, ERR_CODE_CALIBRATION, ESP_OK, 0);
        return;
    }

    float c = v_dot(s_up, s_axis);
    if (c > 1.0f) { c = 1.0f; } else if (c < -1.0f) { c = -1.0f; }
    float tilt = acosf(c) * 57.2957795f;

    float rate = (tilt - s_prev_tilt_deg) / dt_s;
    s_tilt_rate_dps += TILT_RATE_ALPHA * (rate - s_tilt_rate_dps);
    s_prev_tilt_deg = tilt;
    s_tilt_deg = tilt;

    if (!s_tilt_valid && s_gyro_bias_n >= GYRO_BIAS_MIN_SAMPLES) {
        s_tilt_valid = true;
    }
}

static void update_burn_count(const lsm6_data_t *imu, uint32_t now_ms, uint32_t dt_ms)
{
    float a[3] = { imu->ax, imu->ay, imu->az };
    float axial = v_dot(a, s_axis);   // + is along the airframe, nose-up on the pad

    if (!s_burning) {
        if (axial > BURN_ENTER_ACCEL_MS2) {
            s_burn_enter_ms += dt_ms;
            s_burn_enter_n++;
            if (s_burn_enter_ms >= BURN_ENTER_MS && s_burn_enter_n >= DWELL_MIN_SAMPLES) {
                s_burning = true;
                s_burn_enter_ms = 0; s_burn_enter_n = 0;
                s_burn_exit_ms  = 0; s_burn_exit_n  = 0;
                if (s_burn_count < BURN_MAX_COUNT) { s_burn_count++; }
                event_log_event(EV_BURN_START, s_burn_count, now_ms);
                ESP_LOGI(TAG, "BURN %u START (axial %.1f m/s^2)",
                         (unsigned)s_burn_count, axial);
            }
        } else {
            s_burn_enter_ms = 0; s_burn_enter_n = 0;
        }
    } else {
        if (axial < BURN_EXIT_ACCEL_MS2) {
            s_burn_exit_ms += dt_ms;
            s_burn_exit_n++;
            if (s_burn_exit_ms >= BURN_EXIT_MS && s_burn_exit_n >= DWELL_MIN_SAMPLES) {
                s_burning = false;
                s_burn_exit_ms  = 0; s_burn_exit_n  = 0;
                s_burn_enter_ms = 0; s_burn_enter_n = 0;
                event_log_event(EV_BURN_END, s_burn_count, now_ms);
                ESP_LOGI(TAG, "BURN %u END", (unsigned)s_burn_count);
            }
        } else {
            s_burn_exit_ms = 0; s_burn_exit_n = 0;
        }
    }
}

/* ===================================================================
 *  TWO TIMELINES IN ONE FUNCTION.
 *
 *  The loop is paced by the IMU (104 Hz in flight, 12.5 Hz in
 *  standby). The barometer runs at its own rate on its own clock
 *  (50 Hz / 1.5 Hz), so on roughly half the ticks `baro` is the same
 *  reading the previous tick already saw.
 *
 *  Anything derived from the accelerometer or gyro therefore advances
 *  EVERY tick, on the IMU's interval. Anything derived from the
 *  barometer advances ONLY when baro->seq changes, on the barometer's
 *  own interval.
 *
 *  This is not tidiness. A single shared dt would divide a zero
 *  altitude change by the IMU interval on duplicate ticks and the full
 *  change by that same interval on fresh ones, producing a velocity
 *  estimate that alternates between 0 and twice the truth at half the
 *  sample rate -- a signal that looks like violent oscillation and
 *  averages to the right answer, which is the worst of both. Gating on
 *  the sequence number is what keeps VEL1, VEL2 and BVEL1 meaningful.
 *
 *  Freshness is decided by sequence number and never by comparing
 *  altitudes: two consecutive readings of the same altitude are
 *  ordinary, especially on the pad.
 * =================================================================== */
/* Elapsed ms, never negative. now_ms is an IMU timestamp on normal ticks
 * and the wall clock on ticks where the IMU is silent, so consecutive
 * values can step BACK by the few ms between an ISR timestamp and "now".
 * Plain unsigned subtraction would turn that into ~49 days and declare
 * every sensor stale at once. */
static uint32_t since_ms(uint32_t now_ms, uint32_t then_ms)
{
    int32_t d = (int32_t)(now_ms - then_ms);
    return d < 0 ? 0u : (uint32_t)d;
}

static uint32_t failover_ms(void)
{
    uint32_t ms = CONFIG_FC_FAILOVER_MS;
    if (!s_gyro_live && ms < STANDBY_FAILOVER_FLOOR_MS) {
        ms = STANDBY_FAILOVER_FLOOR_MS;
    }
    return ms;
}

static const char *alt_source_name(deploy_alt_source_t s)
{
    switch (s) {
        case DEPLOY_ALT_BARO: return "BAROMETER";
        case DEPLOY_ALT_GPS:  return "GPS";
        default:              return "NONE";
    }
}

static bool gps_ref_valid(void)
{
    return s_gps_pad_n >= GPS_REF_MIN_FIXES || s_gps_flt_n >= GPS_REF_MIN_FIXES;
}

static float gps_agl_m(float msl)
{
    return (s_gps_pad_n >= GPS_REF_MIN_FIXES) ? msl - s_gps_pad_msl
                                              : msl - s_gps_flt_offset;
}

static void ema_learn(float *acc, int *n, float sample)
{
    if (*n == 0) { *acc = sample; }
    else         { *acc += GPS_REF_ALPHA * (sample - *acc); }
    if (*n < GPS_REF_MIN_FIXES * 4) { (*n)++; }
}

/* ===================================================================
 *  IMU failover.
 *
 *  "Silent" means no new sample for failover_ms() -- judged from the
 *  sequence number, so it covers a missing chip, a reset one that has
 *  not been reconfigured yet, and a dead task alike.
 *
 *  BEFORE liftoff there is nothing irreplaceable to lose: the gyro bias
 *  estimate restarts when the IMU returns, and until then an
 *  altitude-based detector stands in for the accelerometer's liftoff.
 *
 *  AFTER liftoff the attitude integrator has missed every rotation of
 *  the outage and has no reference to recover from, and the burn
 *  counter has missed any burn. Both are marked unreliable for the rest
 *  of the flight (s_imu_degraded), even if the IMU comes back.
 * =================================================================== */
static void update_imu_failover(uint32_t now_ms)
{
    if (!s_have_imu) { return; }    /* never had one: nothing to fail over from */

    bool silent = since_ms(now_ms, s_last_imu_ms) >= failover_ms();
    if (silent == s_imu_failover) { return; }
    s_imu_failover = silent;

    if (silent) {
        s_imu_lost_at_ms = s_last_imu_ms;
        s_alt_liftoff_n  = 0;
        if (s_liftoff_detected) {
            s_imu_degraded = true;
            s_tilt_valid   = false;
            ESP_LOGE(TAG, "IMU SILENT %u ms in flight -- tilt invalid and burn count "
                          "frozen for the rest of the flight; apogee and recovery "
                          "continue on %s", (unsigned)since_ms(now_ms, s_last_imu_ms),
                     alt_source_name(s_alt_source));
        } else {
            s_gyro_bias[0] = s_gyro_bias[1] = s_gyro_bias[2] = 0.0f;
            s_gyro_bias_n  = 0;
            s_tilt_valid   = false;
            ESP_LOGE(TAG, "IMU SILENT %u ms before liftoff -- liftoff will be detected "
                          "from altitude (> %d m AGL) until it returns",
                     (unsigned)since_ms(now_ms, s_last_imu_ms), CONFIG_FC_ALT_LIFTOFF_AGL_M);
        }
        event_log_event(EV_IMU_FAILOVER, 1, since_ms(now_ms, s_last_imu_ms));
    } else {
        uint32_t outage = since_ms(now_ms, s_imu_lost_at_ms);
        /* The first interval after an outage is not a sample interval. */
        s_prev_time_ms = now_ms;
        ESP_LOGW(TAG, "IMU back after %u ms%s", (unsigned)outage,
                 s_imu_degraded ? " -- tilt stays invalid for this flight" : "");
        event_log_event(EV_IMU_FAILOVER, 0, outage);
    }
}

/* ===================================================================
 *  Which sensor altitude comes from, decided every tick.
 *
 *  Barometer whenever it has produced a sample within failover_ms();
 *  otherwise GPS if it has a fresh fix and a pad reference; otherwise
 *  nothing. The barometer is preferred the moment it returns.
 *
 *  A change of source restarts velocity and the apogee window: a
 *  derivative or a descending test taken across two different sensors
 *  would be measuring the difference between the sensors.
 * =================================================================== */
static const char *vel_source_name(deploy_vel_source_t s)
{
    switch (s) {
        case DEPLOY_VEL_BARO: return "BAROMETER";
        case DEPLOY_VEL_GPS:  return "GPS";
        case DEPLOY_VEL_IMU:  return "IMU";
        default:              return "NONE";
    }
}

static bool baro_alive(uint32_t now_ms)
{
    return s_have_baro && since_ms(now_ms, s_last_baro_ms) < failover_ms();
}

/* Velocity, apogee and PINC: the barometer while it lives; then the IMU,
 * if it has been alive the whole flight (its velocity is an integral, and
 * a gap would leave it wrong for good); then GPS; then nothing. Called
 * after select_alt_source(), since the GPS choice follows the altitude. */
static void select_vel_source(uint32_t now_ms)
{
    deploy_vel_source_t want;
    if (baro_alive(now_ms)) {
        want = DEPLOY_VEL_BARO;
    } else if (s_have_imu && !s_imu_failover && !s_imu_degraded) {
        want = DEPLOY_VEL_IMU;
    } else if (s_alt_source == DEPLOY_ALT_GPS) {
        want = DEPLOY_VEL_GPS;
    } else {
        want = DEPLOY_VEL_NONE;
    }
    if (want == s_vel_source) { return; }

    ESP_LOGW(TAG, "VELOCITY/APOGEE SOURCE %s -> %s (IMU velocity now %.1f m/s)",
             vel_source_name(s_vel_source), vel_source_name(want), s_imu_vel_ms);
    s_vel_source = want;
    s_apogee_sustain_count = 0;
    s_imu_apogee_n = 0;
    event_log_event(EV_VEL_SOURCE, (uint8_t)want, now_ms);
}

static void select_alt_source(uint32_t now_ms)
{
    bool baro_ok = baro_alive(now_ms);
    bool gps_ok  = s_have_gps &&
                   since_ms(now_ms, s_last_gps_ms) < (uint32_t)CONFIG_FC_GPS_STALE_MS &&
                   gps_ref_valid();
    deploy_alt_source_t want = baro_ok ? DEPLOY_ALT_BARO
                             : gps_ok  ? DEPLOY_ALT_GPS
                             :           DEPLOY_ALT_NONE;
    if (want == s_alt_source) { return; }

    ESP_LOGW(TAG, "ALTITUDE SOURCE %s -> %s", alt_source_name(s_alt_source),
             alt_source_name(want));
    s_alt_source = want;
    s_have_prev_alt = false;
    s_altitude_hist_idx = 0;
    s_altitude_hist_count = 0;
    s_apogee_sustain_count = 0;
    s_alt_liftoff_n = 0;
    event_log_event(EV_ALT_SOURCE, (uint8_t)want, now_ms);
}

static void update_flight_state(const deploy_inputs_t *in, uint32_t now_ms)
{
    const bmp388_data_t *baro = in->baro;
    const lsm6_data_t   *imu  = in->imu;
    const deploy_gps_t  *gps  = in->gps;

    update_gyro_liveness();

    /* A sample is fresh if its sequence number moved; seq 0 means the
     * driver has not published anything yet. The loop calls this on
     * every IMU sample and, when the IMU is silent, on a timer -- so
     * no input can be assumed new. */
    bool imu_fresh  = imu && imu->seq != 0 && (!s_have_imu || imu->seq != s_last_imu_seq);
    bool baro_fresh = baro && baro->seq != 0 && (!s_have_baro || baro->seq != s_last_baro_seq);
    bool gps_fresh  = gps && gps->seq != 0 && gps->fix_quality > 0 &&
                      (!s_have_gps || gps->seq != s_last_gps_seq);

    if (imu_fresh)  { s_last_imu_seq  = imu->seq;  s_have_imu  = true; s_last_imu_ms  = now_ms; }
    if (baro_fresh) { s_last_baro_seq = baro->seq; s_have_baro = true; s_last_baro_ms = now_ms; }
    if (gps_fresh)  { s_last_gps_seq  = gps->seq;  s_have_gps  = true; s_last_gps_ms  = now_ms; }

    /* ---------------- ground calibration ----------------
     * Altitude is averaged over CAL_SAMPLES *barometric* samples, so the
     * window is the same 20 readings it always was rather than 20 ticks
     * containing 10 distinct pressures. The accelerometer average takes
     * every new IMU sample, because more samples is strictly better for
     * the axis and they are all genuinely independent. */
    if (!s_ground_calibrated) {
        if (imu_fresh) {
            s_cal_acc_sum[0] += imu->ax;
            s_cal_acc_sum[1] += imu->ay;
            s_cal_acc_sum[2] += imu->az;
            s_cal_imu_n++;
            update_gyro_bias(imu);
        }
        if (baro_fresh) {
            s_cal_alt_sum += baro->altitude_m;
            s_cal_count++;
        }
        if (gps_fresh) {
            ema_learn(&s_gps_pad_msl, &s_gps_pad_n, gps->altitude_m);
        }

        if (s_cal_count >= CAL_SAMPLES) {
            s_ground_altitude = s_cal_alt_sum / s_cal_count;

            /* The averaged accelerometer vector on a stationary pad IS
             * the airframe's up-axis in body coordinates. Normalising it
             * is the whole of the "which way is the rocket mounted"
             * question. */
            float axis[3] = { s_cal_acc_sum[0], s_cal_acc_sum[1], s_cal_acc_sum[2] };

            /* Gravity as THIS accelerometer measures it, used to correct its
             * scale (see the IMU velocity integration). Assuming the
             * textbook 9.80665 instead would turn a 1% scale error into a
             * constant 0.1 m/s^2 -- 2 m/s of velocity error by apogee. A
             * value far from 1 g means the board was moving or the sensor
             * is broken; keep the standard. */
            if (s_cal_imu_n > 0) {
                float n = (float)s_cal_imu_n;
                float g = sqrtf((axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])) / n;
                s_g_pad = (g > 0.8f * G_STD && g < 1.2f * G_STD) ? g : G_STD;
            }
            if (v_normalize(axis)) {
                memcpy(s_axis, axis, sizeof(s_axis));
                memcpy(s_up, axis, sizeof(s_up));
                s_tilt_deg = 0.0f;
                s_prev_tilt_deg = 0.0f;
            } else {
                /* No usable gravity vector -- the board is in free fall
                 * or the accelerometer is dead. Either way tilt is not
                 * computable, and saying so is better than defaulting to
                 * an axis that happens to be wrong. */
                ESP_LOGE(TAG, "Ground calibration: no usable gravity vector; "
                              "FANG/FUTANG will stay unavailable");
                event_log_error(ERR_SRC_FLIGHT_LOGIC, ERR_CODE_CALIBRATION, ESP_OK, 1);
            }

            s_ground_calibrated = true;
            s_prev_altitude = 0;
            s_have_prev_alt = true;
            s_prev_time_ms = now_ms;
            s_prev_baro_ms = now_ms;
            ESP_LOGI(TAG, "Ground altitude calibrated: %.2f m; airframe axis "
                          "[%.2f %.2f %.2f]", s_ground_altitude,
                     s_axis[0], s_axis[1], s_axis[2]);
        }
        return;
    }

    update_imu_failover(now_ms);
    select_alt_source(now_ms);
    select_vel_source(now_ms);

    /* ---------------- IMU timeline: every NEW IMU sample ---------------- */
    if (imu_fresh) {
        uint32_t dt_ms = since_ms(now_ms, s_prev_time_ms);
        float    dt_s  = dt_ms / 1000.0f;
        s_prev_time_ms = now_ms;
        uint32_t dwell_ms = dt_ms > IMU_GAP_MS ? 0 : dt_ms;

        update_gyro_bias(imu);
        update_tilt(imu, dt_s);
        update_burn_count(imu, now_ms, dwell_ms);

        // Liftoff detection: sustained high net acceleration
        if (!s_liftoff_detected) {
            float accel_mag = sqrtf(imu->ax * imu->ax + imu->ay * imu->ay + imu->az * imu->az);
            if (accel_mag > LIFTOFF_ACCEL_THRESHOLD_MS2) {
                s_liftoff_accum_ms += dwell_ms;
                s_liftoff_samples++;
                if (s_liftoff_accum_ms >= LIFTOFF_SUSTAIN_MS &&
                    s_liftoff_samples >= DWELL_MIN_SAMPLES) {
                    s_liftoff_detected = true;
                    s_liftoff_ms = now_ms;
                    /* Freeze the gyro bias here. After this point the vehicle
                     * is never at rest again, so every further sample would
                     * corrupt the estimate rather than refine it. */
                    s_bias_frozen = true;

                    /* ...but a frozen estimate is only worth having if it
                     * converged. Launching seconds after arming -- or going
                     * STANDBY straight to FLIGHT on the IMU wake-up, which is
                     * the whole point of that fallback -- can freeze a bias
                     * that has seen almost no live samples. Reporting tilt
                     * from it would be reporting a ramp. */
                    if (s_gyro_bias_n < GYRO_BIAS_MIN_SAMPLES) {
                        s_tilt_valid = false;
                        ESP_LOGE(TAG, "LIFTOFF before the gyro bias converged (%d/%d "
                                      "live samples) -- tilt events DISABLED for this flight",
                                 s_gyro_bias_n, GYRO_BIAS_MIN_SAMPLES);
                        event_log_error(ERR_SRC_FLIGHT_LOGIC, ERR_CODE_CALIBRATION,
                                        ESP_OK, 2);
                    }

                    event_log_event(EV_LIFTOFF_SW, LIFTOFF_SRC_ACCEL, now_ms);
                    ESP_LOGI(TAG, "LIFTOFF DETECTED (gyro bias frozen at "
                                  "[%.2f %.2f %.2f] dps from %d samples)",
                             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2], s_gyro_bias_n);
                }
            } else {
                s_liftoff_accum_ms = 0;
                s_liftoff_samples = 0;
            }
        }

        /* ---- vertical velocity from the accelerometer ----
         * The accelerometer reads specific force: true acceleration minus
         * gravity. Projected onto "up" and with the pad's measured gravity
         * removed, what is left is vertical acceleration. "Up" is the
         * tracked attitude while tilt is valid; otherwise the airframe
         * axis -- exact on the pad, and within cos(tilt) of right in a
         * near-vertical boost.
         *
         * Held at zero on the pad, integrated from the first sample of
         * the liftoff dwell, reset if that dwell turns out to be a bump. */
        if (!s_imu_degraded) {
            bool moving = s_liftoff_detected || s_liftoff_samples > 0;
            if (moving && dwell_ms > 0) {
                float f[3] = { imu->ax, imu->ay, imu->az };
                const float *up = s_tilt_valid ? s_up : s_axis;
                /* The pad reading is treated as a scale calibration: this
                 * accelerometer says s_g_pad where the truth is 1 g, so
                 * every reading is divided by that ratio before g is
                 * removed. Subtracting s_g_pad alone would cancel the
                 * offset on the pad but still inflate the boost by the
                 * same percentage. */
                float a_vert = v_dot(f, up) * (G_STD / s_g_pad) - G_STD;
                s_imu_vel_ms += a_vert * (dwell_ms / 1000.0f);
            } else if (!moving) {
                s_imu_vel_ms = 0.0f;
            }

            /* Apogee from the IMU: velocity has been clearly upward, and
             * has now been at or below zero for a few samples. Only when
             * the IMU is the velocity source -- with the barometer alive
             * its altitude window is the authority, as it always was. */
            if (s_liftoff_detected && s_imu_vel_ms > IMU_APOGEE_ARM_MS) {
                s_imu_vel_rose = true;
            }
            if (s_vel_source == DEPLOY_VEL_IMU && s_liftoff_detected &&
                !s_apogee_detected && s_imu_vel_rose) {
                if (s_imu_vel_ms <= 0.0f) {
                    if (++s_imu_apogee_n >= IMU_APOGEE_SUSTAIN) {
                        s_apogee_detected = true;
                        event_log_event(EV_APOGEE, (uint8_t)DEPLOY_VEL_IMU, now_ms);
                        ESP_LOGI(TAG, "APOGEE DETECTED from IMU velocity (alt AGL ~%.1f m "
                                      "from %s)", s_altitude_agl_m,
                                 alt_source_name(s_alt_source));
                    }
                } else {
                    s_imu_apogee_n = 0;
                }
            }
        }
    }

    /* ---------------- GPS pad reference ---------------- */
    if (gps_fresh) {
        if (!s_liftoff_detected) {
            ema_learn(&s_gps_pad_msl, &s_gps_pad_n, gps->altitude_m);
        } else if (s_gps_pad_n < GPS_REF_MIN_FIXES && s_alt_source == DEPLOY_ALT_BARO) {
            /* No pad reference (the GPS was off before liftoff): learn
             * the GPS-to-AGL offset against the barometer while it still
             * works, so there is something to fall back on if it stops. */
            ema_learn(&s_gps_flt_offset, &s_gps_flt_n, gps->altitude_m - s_altitude_agl_m);
        }
    }

    /* ---------------- altitude timeline: one NEW sample of the source ---- */
    float altitude_agl;
    if (s_alt_source == DEPLOY_ALT_BARO && baro_fresh) {
        altitude_agl = baro->altitude_m - s_ground_altitude;
    } else if (s_alt_source == DEPLOY_ALT_GPS && gps_fresh) {
        altitude_agl = gps_agl_m(gps->altitude_m);
    } else {
        return;
    }
    s_altitude_agl_m = altitude_agl;

    // Vertical velocity (ft/s, matching JSON threshold units)
    if (s_have_prev_alt) {
        float alt_dt_s = since_ms(now_ms, s_prev_baro_ms) / 1000.0f;
        if (alt_dt_s > 0.001f) {
            float vel_ms = (altitude_agl - s_prev_altitude) / alt_dt_s;
            s_baro_velocity = vel_ms * 3.28084f;  // m/s -> ft/s
        }
    }
    s_prev_altitude = altitude_agl;
    s_prev_baro_ms = now_ms;
    s_have_prev_alt = true;

    /* Liftoff from altitude -- only while the IMU is silent, which is the
     * only time the accelerometer detector cannot run. Late by however
     * long the climb to CONFIG_FC_ALT_LIFTOFF_AGL_M takes, so TVAL
     * timings shift by that much in this case. Also moves the flight
     * mode: with no IMU there is no wake-up interrupt to do it, and a
     * vehicle left in STANDBY would keep its barometer at 1.5 Hz. */
    if (!s_liftoff_detected && s_imu_failover) {
        if (altitude_agl > (float)CONFIG_FC_ALT_LIFTOFF_AGL_M) {
            int need = (s_alt_source == DEPLOY_ALT_GPS) ? ALT_LIFTOFF_SAMPLES_GPS
                                                        : ALT_LIFTOFF_SAMPLES_BARO;
            if (++s_alt_liftoff_n >= need) {
                s_liftoff_detected = true;
                s_liftoff_ms   = now_ms;
                s_bias_frozen  = true;
                s_imu_degraded = true;      /* the IMU missed liftoff itself */
                s_tilt_valid   = false;
                event_log_event(EV_LIFTOFF_SW, LIFTOFF_SRC_ALTITUDE, now_ms);
                ESP_LOGW(TAG, "LIFTOFF DETECTED FROM %s ALTITUDE (%.1f m AGL) -- IMU "
                              "silent, tilt unavailable this flight",
                         alt_source_name(s_alt_source), altitude_agl);
                flight_mode_note_liftoff(MODE_SRC_FLIGHT_LOGIC);
            }
        } else {
            s_alt_liftoff_n = 0;
        }
    }

    /* Descending / apogee. The history push and the descending test are
     * deliberately OUTSIDE the !s_apogee_detected guard: freezing
     * s_pressure_increasing at apogee would make EVT_PINC mean "apogee
     * has happened" for the rest of the flight rather than "descending
     * now". Only the latch belongs behind the guard.
     *
     * The window compares the newest sample with the one (win - 1)
     * samples earlier: 10 barometer samples is ~180 ms at 50 Hz, which
     * is what this file always used; 3 GPS fixes is 2 s. */
    if (s_liftoff_detected) {
        int win     = (s_alt_source == DEPLOY_ALT_GPS) ? GPS_APOGEE_WINDOW  : 10;
        int sustain = (s_alt_source == DEPLOY_ALT_GPS) ? GPS_APOGEE_SUSTAIN : APOGEE_SUSTAIN_SAMPLES;

        s_altitude_history[s_altitude_hist_idx] = altitude_agl;
        s_altitude_hist_idx = (s_altitude_hist_idx + 1) % 10;
        if (s_altitude_hist_count < 10) s_altitude_hist_count++;

        if (s_altitude_hist_count >= win) {
            float oldest = s_altitude_history[(s_altitude_hist_idx - win + 10) % 10];
            s_pressure_increasing = (altitude_agl < oldest);   // net descending over the window

            if (!s_apogee_detected) {
                if (s_pressure_increasing) {
                    s_apogee_sustain_count++;
                    if (s_apogee_sustain_count >= sustain &&
                        s_vel_source != DEPLOY_VEL_IMU) {
                        s_apogee_detected = true;
                        event_log_event(EV_APOGEE, (uint8_t)s_alt_source, now_ms);
                        ESP_LOGI(TAG, "APOGEE DETECTED from %s (alt AGL ~%.1f m, tilt %.1f deg)",
                                 alt_source_name(s_alt_source), altitude_agl, s_tilt_deg);
                    }
                } else {
                    s_apogee_sustain_count = 0;
                }
            }
        }
    }
}

// ── Per-event truth evaluation ──

static bool event_is_true(deploy_event_t evt, float threshold,
                           const bmp388_data_t *baro, uint32_t now_ms,
                           deploy_config_t *cfg, const deploy_channel_t *ch)
{
    (void)baro;   /* altitude now comes from whichever source is live */
    float altitude_agl_ft = s_altitude_agl_m * 3.28084f;
    float t_since_liftoff_s = s_liftoff_detected
                            ? (float)(now_ms - s_liftoff_ms) / 1000.0f
                            : 0.0f;

    /* No altitude source at all: every altitude-derived event is
     * unknown, and unknown reads FALSE -- the same rule tilt follows.
     * A stale altitude would otherwise keep satisfying "below 500 ft"
     * or "descending" from whatever it last said. */
    bool alt_known = (s_alt_source != DEPLOY_ALT_NONE);
    /* Velocity, BVEL and PINC follow the velocity source instead: with the
     * barometer gone they come from the IMU even when there is no GPS
     * altitude at all. */
    bool vel_known = (s_vel_source != DEPLOY_VEL_NONE);

    switch (evt) {
        case EVT_AGL1:     return alt_known && altitude_agl_ft < threshold;
        case EVT_AGL2:     return alt_known && altitude_agl_ft > threshold;
        case EVT_VEL1:     return vel_known && fabsf(current_vel_fps()) < threshold;
        case EVT_VEL2:     return vel_known && fabsf(current_vel_fps()) > threshold;
        case EVT_BVEL1:    return vel_known && current_vel_fps() < threshold;
        case EVT_LIFTOFF:  return s_liftoff_detected;
        case EVT_APOGEE:   return s_apogee_detected;
        case EVT_PINC:     return vel_known && currently_descending();

        /* Tilt events. An unavailable attitude estimate reads FALSE, not
         * "assume it's fine": these sit in an AND-set, so false blocks
         * the channel. A tilt condition that silently passes because the
         * gyro never came up is the one failure mode worth designing
         * out, since it turns a safety check into a no-op exactly when
         * the safety check is the only thing that would have caught it. */
        case EVT_FANG1:    return s_tilt_valid && s_tilt_deg < threshold;
        case EVT_FANG2:    return s_tilt_valid && s_tilt_deg > threshold;
        case EVT_FUTANG: {
            if (!s_tilt_valid) { return false; }
            /* Horizon = this channel's own delay, because the question
             * FUTANG answers is "where will the nose be pointing when
             * this channel would actually reach its condition", and that
             * is delay_ms from now by construction. */
            float horizon_s = (ch && ch->delay_ms > 0)
                            ? (float)ch->delay_ms / 1000.0f
                            : (float)FUTANG_DEFAULT_HORIZON_MS / 1000.0f;
            float projected = s_tilt_deg + s_tilt_rate_dps * horizon_s;
            if (projected < 0.0f) { projected = 0.0f; }
            return projected < threshold;
        }

        case EVT_TVAL1:    return s_liftoff_detected && t_since_liftoff_s < threshold;
        case EVT_TVAL2:    return s_liftoff_detected && t_since_liftoff_s > threshold;

        /* Threshold is a count, so this is the one event where the
         * comparison is >= rather than < or >. BURNCOUNT(2) means "the
         * second motor has lit". */
        case EVT_BURNCOUNT: return (float)s_burn_count >= threshold;

        case EVT_APOFIRED:   return cfg->apo.triggered        && (now_ms - cfg->apo.triggered_time_ms) >= 1500;
        case EVT_MAINFIRED:  return cfg->main_chute.triggered && (now_ms - cfg->main_chute.triggered_time_ms) >= 1500;
        case EVT_R3FIRED:    return cfg->third.triggered      && (now_ms - cfg->third.triggered_time_ms) >= 1500;
        case EVT_R4FIRED:    return cfg->fourth.triggered     && (now_ms - cfg->fourth.triggered_time_ms) >= 1500;

        default:
            return false;
    }
}

/* Events that only the IMU can make true. */
static bool is_imu_event(deploy_event_t e)
{
    return e == EVT_FANG1 || e == EVT_FANG2 || e == EVT_FUTANG || e == EVT_BURNCOUNT;
}

static bool set_all_true(bool *enabled, float *thresholds, const bmp388_data_t *baro,
                          uint32_t now_ms, deploy_config_t *cfg, const deploy_channel_t *ch)
{
    /* skip_imu_if_lost: with the IMU unreliable, its events are dropped
     * from the set rather than read as FALSE. Dropped, not passed --
     * they do not count towards any_enabled, so a set made ONLY of IMU
     * events becomes empty and stays unsatisfiable, exactly as before. */
    bool skip_imu = ch && ch->skip_imu_if_lost && (s_imu_failover || s_imu_degraded);

    bool any_enabled = false;
    for (int i = 0; i < EVT_COUNT; i++) {
        if (!enabled[i]) continue;
        if (skip_imu && is_imu_event((deploy_event_t)i)) continue;
        any_enabled = true;
        if (!event_is_true((deploy_event_t)i, thresholds[i], baro, now_ms, cfg, ch)) {
            return false;
        }
    }
    return any_enabled;   // an empty set is never considered "satisfied"
}

static void evaluate_channel(deploy_channel_t *ch, const bmp388_data_t *baro,
                              uint32_t now_ms, deploy_config_t *cfg,
                              uint8_t channel_index)
{
    if (!ch->armed || ch->triggered) return;
    if (!s_liftoff_detected) return;   // nothing is evaluated before liftoff

    bool primary_ok = set_all_true(ch->primary_enabled, ch->thresholds, baro, now_ms, cfg, ch);
    bool secondary_ok = set_all_true(ch->secondary_enabled, ch->thresholds, baro, now_ms, cfg, ch);
    bool conditions_met = primary_ok || secondary_ok;

    if (conditions_met && !ch->timing) {
        ch->timing = true;
        ch->timing_start_ms = now_ms;
    }

    /* Non-latching channels release the countdown if the conditions stop
     * holding. Previously `timing` was one-way, so a condition that went
     * true for a single sample started a countdown that could never be
     * cancelled -- the channel would record a trigger delay_ms later on
     * the strength of one noisy reading. `latch` is what the config uses
     * to ask for the old behaviour deliberately. */
    if (ch->timing && !conditions_met && !ch->latch) {
        ch->timing = false;
    }

    if (ch->timing && (now_ms - ch->timing_start_ms) >= (uint32_t)ch->delay_ms) {
        ch->triggered = true;
        ch->triggered_time_ms = now_ms;
        /* THIS IS A RECORD, NOT AN ACTION. Nothing is energised; the
         * board has no output to energise. The value of the record is
         * that it carries this board's own timestamp for the moment its
         * logic would have acted, which is what you lay next to the Blue
         * Raven's timeline afterwards. */
        event_log_event(EV_DEPLOY_PREDICTED, channel_index, now_ms);
        ESP_LOGI(TAG, "CHANNEL %s reached its condition at t=%lu ms "
                      "(alt %.1f m AGL, tilt %.1f deg, burns %u) -- recorded only",
                 CHANNEL_NAMES[channel_index < 4 ? channel_index : 0],
                 (unsigned long)now_ms, s_altitude_agl_m, s_tilt_deg,
                 (unsigned)s_burn_count);
    }
}

void deploy_logic_update(deploy_config_t *cfg, const bmp388_data_t *baro,
                          const lsm6_data_t *imu, uint32_t now_ms)
{
    deploy_inputs_t in = { .baro = baro, .imu = imu, .gps = NULL };
    deploy_logic_update_ex(cfg, &in, now_ms);
}

void deploy_logic_update_ex(deploy_config_t *cfg, const deploy_inputs_t *in,
                             uint32_t now_ms)
{
    const bmp388_data_t *baro = in->baro;
    update_flight_state(in, now_ms);
    publish_state();

    if (!s_ground_calibrated) return;   // still calibrating, don't evaluate channels yet

    /* Index matches the declaration order in deploy_config_t, which is
     * what event_record_t.channel refers to: 0=apo 1=main 2=third
     * 3=fourth. Keep these in step or the event log lies about which
     * channel triggered. */
    evaluate_channel(&cfg->apo,        baro, now_ms, cfg, 0);
    evaluate_channel(&cfg->main_chute, baro, now_ms, cfg, 1);
    evaluate_channel(&cfg->third,      baro, now_ms, cfg, 2);
    evaluate_channel(&cfg->fourth,     baro, now_ms, cfg, 3);
}
