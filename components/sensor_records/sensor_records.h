#ifndef SENSOR_RECORDS_H
#define SENSOR_RECORDS_H

#include <stdint.h>

/* ===================================================================
 *  Shared "which sensor" identifier and on-the-wire record shapes.
 *
 *  These used to live inside sd_logger.h, named sd_stream_t /
 *  sd_baro_record_t / etc. Moved here and renamed once a second and
 *  third consumer (telemetry, flight logic) needed the exact same
 *  shapes: SD was never supposed to be the naming authority for "what
 *  a barometer sample looks like on the wire" -- it's just one of
 *  three peers now, not a shared header everyone else has to import.
 *
 *  __attribute__((packed)) is kept even though flight_logic_t doesn't
 *  care about byte-for-byte layout -- SD and telemetry both do, and
 *  reusing one struct for all three is the whole point of this file.
 * =================================================================== */

typedef enum {
    STREAM_BARO = 0,
    STREAM_IMU  = 1,
    STREAM_GPS  = 2,
    STREAM_BATT = 3,
    STREAM_ERR  = 4,
    STREAM_EVENT = 5,
    STREAM_STATUS = 6,
    STREAM_COUNT
} sensor_stream_t;

/* ===================================================================
 *  Flight mode
 *
 *  Four states, not two. "Standby" and "flight" is the user-facing
 *  description, but the rules about what may change the mode differ
 *  before liftoff, after liftoff, and again after landing -- and
 *  encoding that as a state is far safer than scattering those
 *  conditions through whoever happens to be asking.
 *
 *    STANDBY  slow rates, gyro powered down. Boot state.
 *    ARMED    flight rates, liftoff not yet seen. Ground may disarm.
 *    FLIGHT   liftoff seen. DISARM IS REFUSED -- losing the flight
 *             record to a stray packet is the worst outcome available.
 *    LANDED   landing detected. Flight rates CONTINUE; disarm is
 *             unlocked and announced, but the vehicle never drops to
 *             standby on its own. It takes sensor evidence AND a human
 *             on the ground.
 * =================================================================== */
typedef enum {
    MODE_STANDBY = 0,
    MODE_ARMED   = 1,
    MODE_FLIGHT  = 2,
    MODE_LANDED  = 3,
    MODE_COUNT
} flight_mode_t;

/* Who asked for a mode change -- recorded in the event so a log says
 * whether the vehicle was armed by the ground or by its own
 * accelerometer. */
typedef enum {
    MODE_SRC_NONE = 0,
    MODE_SRC_UPLINK,       // command from the ground station
    MODE_SRC_IMU_WAKEUP,   // LSM6DSOX INT2 -- the backup trigger
    MODE_SRC_FLIGHT_LOGIC, // software liftoff / apogee / landing detector
    MODE_SRC_BOOT
} mode_source_t;

/* STREAM_BATT is house-keeping, not flight data, and it is deliberately
 * its own stream rather than a field bolted onto an existing record.
 * Two reasons: it samples at its own rate (10 Hz, unrelated to any
 * sensor's), and it must stay readable when a sensor stream is the
 * thing that failed -- a brownout that corrupts imu.bin should still
 * leave batt.bin intact and explaining why. */

/* STREAM_ERR and STREAM_EVENT are the two non-measurement streams.
 *
 * They are separate from each other because they answer different
 * questions and fail differently. err.bin is "what went wrong" -- it can
 * be empty on a perfect flight, and it can burst. event.bin is "what
 * happened, and when" -- a short, ordered timeline of a dozen or so
 * records that is the first thing anyone opens after a flight. Mixing
 * them would let an error burst bury the timeline in exactly the
 * situation where the timeline matters most. */

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;
    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;
} baro_record_t;           // 24 bytes

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;
    float    ax, ay, az;
    float    gx, gy, gz;
} imu_record_t;             // 36 bytes

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;
    float    battery_v;        // reconstructed pack voltage
    float    adc_v;            // raw voltage at the pin, before the
                                // divider is undone -- kept so a bad
                                // divider constant can be corrected
                                // offline instead of losing the flight
} batt_record_t;            // 20 bytes

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;
    double   latitude;
    double   longitude;
    float    altitude_m;
    uint8_t  fix_quality;
    uint8_t  satellites;
} gps_record_t;              // 34 bytes

/* ===================================================================
 *  Error stream
 *
 *  Recorded by event_log_error(). Both source and code are small enums
 *  rather than a string: a string would have to be either a pointer
 *  (meaningless once the log is off the vehicle) or an inline buffer
 *  (which would dwarf the rest of the record). tools/decode_logs.py
 *  maps them back to names.
 * =================================================================== */

typedef enum {
    ERR_SRC_NONE = 0,
    ERR_SRC_BMP388,
    ERR_SRC_LSM6DSOX,
    ERR_SRC_GPS,
    ERR_SRC_BATTERY,
    ERR_SRC_SD,
    ERR_SRC_TELEMETRY,
    ERR_SRC_FLIGHT_LOGIC,
    ERR_SRC_DISCRETE_IN,
    ERR_SRC_ROUTER,
    ERR_SRC_SYSTEM,          // boot, NVS, SPI bus, anything in main.c
    ERR_SRC_COUNT
} err_source_t;

typedef enum {
    ERR_CODE_NONE = 0,
    ERR_CODE_INIT_FAILED,
    ERR_CODE_READ_FAILED,
    ERR_CODE_WRITE_FAILED,
    ERR_CODE_IRQ_TIMEOUT,
    ERR_CODE_QUEUE_FULL,     // a record was dropped to make room
    ERR_CODE_PARSE_FAILED,
    ERR_CODE_MOUNT_FAILED,
    ERR_CODE_TASK_FAILED,
    ERR_CODE_CONFIG_INVALID,
    ERR_CODE_OVERFLOW,
    ERR_CODE_CALIBRATION,
    ERR_CODE_COUNT
} err_code_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;
    uint8_t  source;         // err_source_t
    uint8_t  code;           // err_code_t
    int16_t  esp_err;        // the esp_err_t that caused it, 0 if none
    uint32_t detail;         // free field: a pin, a count, an offset
    uint32_t suppressed;     // identical errors rate-limited away since the
                              // last record of this source. Non-zero here is
                              // the difference between "one glitch" and "this
                              // sensor has been failing 200 times a second".
} err_record_t;              // 24 bytes

/* ===================================================================
 *  Event stream -- the flight timeline
 * =================================================================== */

typedef enum {
    EV_NONE = 0,
    EV_BOOT,
    EV_SD_MOUNTED,
    EV_STAGING_SIGNAL,       // Blue Raven staging line asserted (J8, GPIO22)
    EV_RECOVERY_SIGNAL,      // Blue Raven recovery line asserted (J7, GPIO23)
    EV_LIFTOFF_HW,           // LSM6DSOX wake-up block on INT2
    EV_LIFTOFF_SW,           // flight_logic's own threshold detector
    EV_APOGEE,
    EV_DEPLOY_ARMED,
    /* Renamed from EV_DEPLOY_FIRED, same numeric value so existing logs
     * still decode. The old name was a lie on this hardware: PCB v1 has
     * no deployment outputs, so this record has never meant "a charge
     * was fired" -- it means "this board's own logic reached the
     * condition where it would have". That distinction is the whole
     * point of the record, and a name that hid it was actively
     * misleading in a flight review next to the Blue Raven's timeline. */
    EV_DEPLOY_PREDICTED,
    EV_MODE_CHANGED,         // channel = new flight_mode_t, detail = mode_source_t
    EV_LANDED,               // landing detected; disarm is now unlocked
    EV_UPLINK_COMMAND,       // a command was accepted from the ground
    EV_UPLINK_REJECTED,      // a command was understood but refused
    /* Appended, never inserted: an old decoder reading a new log gets an
     * unknown name for these, not a wrong name for everything after
     * them. channel = the burn's number (1 = first), detail = now_ms. */
    EV_BURN_START,           // flight_logic saw axial thrust begin
    EV_BURN_END,             // ...and end. BURNCOUNT counts the starts.
    EV_COUNT
} event_type_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;   // ISR-captured where the source is an interrupt,
                              // so this is when the edge happened, not when a
                              // task got around to noticing it
    uint8_t  type;           // event_type_t
    uint8_t  channel;        // deploy channel index, or BR line index; 0 if n/a
    uint16_t flags;          // reserved
    uint32_t detail;         // event-specific
} event_record_t;            // 20 bytes

/* ===================================================================
 *  Status stream -- the vehicle's own health snapshot.
 *
 *  Written periodically and on demand (a PING from the ground). It goes
 *  to the card as well as over the radio deliberately: the pad record is
 *  where you find out the pack was already sagging before launch.
 *
 *  STATUS_FLAG_DISARM_OK is the one the ground UI keys on. It is set
 *  only in MODE_STANDBY, MODE_ARMED and MODE_LANDED -- never during
 *  MODE_FLIGHT -- so the toggle greys itself out for exactly the window
 *  where a disarm would destroy the flight record.
 * =================================================================== */

#define STATUS_FLAG_SD_OK        0x01
#define STATUS_FLAG_TELEM_OK     0x02
#define STATUS_FLAG_GPS_FIX      0x04
#define STATUS_FLAG_LIFTOFF_HW   0x08
#define STATUS_FLAG_LIFTOFF_SW   0x10
#define STATUS_FLAG_APOGEE       0x20
#define STATUS_FLAG_LANDED       0x40
#define STATUS_FLAG_DISARM_OK    0x80

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t timestamp_us;
    uint8_t  mode;            // flight_mode_t
    uint8_t  flags;           // STATUS_FLAG_*
    uint16_t uptime_s;
    float    battery_v;
    float    altitude_agl_m;
    uint32_t sd_drops;
    uint32_t telem_drops;
} status_record_t;            // 32 bytes

/* Largest of the seven, used by anything sizing a fixed buffer to hold
 * "whichever record type this happens to be" -- e.g. the router and
 * each subsystem's internal queue item. Still 36 (imu_record_t);
 * status_record_t is 32. */
#define SENSOR_RECORD_MAX_SIZE   40

/* ===================================================================
 *  Shared queue envelope.
 *
 *  Every ring-buffer-backed consumer (SD, telemetry) needs a single
 *  fixed-size element type to store, since one record type can be
 *  24, 34, or 36 bytes and a ring buffer needs to know its element
 *  size once, up front. Both sd_logger.c and telemetry.c originally
 *  defined their own private, byte-for-byte identical copy of this
 *  (sd_queue_item_t / telem_queue_item_t) -- same fields, same size,
 *  just two names for one shape. Sharing it here means a future
 *  queue-based consumer reuses this instead of adding a third copy.
 * =================================================================== */

typedef struct {
    uint8_t stream;
    uint8_t len;
    uint8_t payload[SENSOR_RECORD_MAX_SIZE];
} queue_item_t;

/* ===================================================================
 *  Stream -> record size, in one place.
 *
 *  sd_logger needed this to write file headers and telemetry needs it
 *  to pack records without spending a length byte on each one. Two
 *  private tables would be two things to forget to update; an inline
 *  function keeps it single-source without giving this otherwise
 *  header-only component a .c file. Returns 0 for an unknown stream,
 *  which every caller treats as "refuse the record".
 * =================================================================== */
static inline uint8_t sensor_record_size(sensor_stream_t stream)
{
    switch (stream) {
        case STREAM_BARO:  return (uint8_t)sizeof(baro_record_t);
        case STREAM_IMU:   return (uint8_t)sizeof(imu_record_t);
        case STREAM_GPS:   return (uint8_t)sizeof(gps_record_t);
        case STREAM_BATT:  return (uint8_t)sizeof(batt_record_t);
        case STREAM_ERR:   return (uint8_t)sizeof(err_record_t);
        case STREAM_EVENT: return (uint8_t)sizeof(event_record_t);
        case STREAM_STATUS: return (uint8_t)sizeof(status_record_t);
        default:           return 0;
    }
}

#endif // SENSOR_RECORDS_H
