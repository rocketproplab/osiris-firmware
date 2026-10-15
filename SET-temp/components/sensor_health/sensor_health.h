#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_records.h"

/* ===================================================================
 *  Is this sensor still there, and if not, when do we try again?
 *
 *  Shared by bmp388_sensor and lsm6dsox_sensor. It decides nothing about
 *  the chips themselves -- each driver owns its own probe and its own
 *  re-init -- it only keeps the bookkeeping identical between them:
 *  one LOST event when contact goes, one RECOVERED event when it comes
 *  back, a retry schedule in between, and counters for the console.
 *
 *  WHY A DRIVER CANNOT JUST WAIT FOR A BUS ERROR. On SPI a chip that is
 *  not there produces no error at all: the master clocks out its bytes
 *  and reads back whatever MISO floats to, usually 0x00 or 0xFF, and
 *  calls it a success. So "lost" is decided by asking the chip a
 *  question only it can answer -- its ID register -- not by the bus.
 *
 *  THREE CASES the drivers tell apart, because they want different
 *  responses:
 *
 *    ID wrong                 -> NO_RESPONSE. Unplugged, or a bus wire
 *                                (SCK/MOSI/MISO/CS) open. Retry.
 *    ID right, config gone    -> RESET. The chip lost power and came
 *                                back with factory defaults: asleep, no
 *                                data, no interrupt. Re-init at once.
 *    ID right, config intact  -> not lost. Only the INT line is quiet,
 *                                so the driver keeps polling on its
 *                                timeout exactly as it always has.
 *
 *  Not ISR-safe: it ends in event_log_event(). Called from each
 *  sensor's own task only, which is also why it needs no lock.
 * =================================================================== */

/* How often a healthy sensor is asked for its ID even when its
 * interrupt is arriving normally. Catches the case where the chip is
 * powered and pulsing DRDY but its data line is open, so every read is
 * garbage that looks like a sample. One short bus transaction a second. */
#define SENSOR_HEALTH_CHECK_MS   1000

/* Retry period while a sensor is lost. A probe is one ID read, so this
 * costs almost nothing, and in flight a quarter second is the most data
 * you can lose after a connector reseats itself. */
#define SENSOR_HEALTH_RETRY_MS   250

/* While a sensor stays lost, EV_SENSOR_LOST is recorded again this often.
 * The downlink has no acknowledgement, so a single event lost in one
 * dropped packet would otherwise hide the fault from the ground for the
 * rest of the flight -- and a sensor found missing at boot is declared
 * lost before the radio exists at all. The ground station collapses the
 * repeats into one line with a count. */
#define SENSOR_HEALTH_REANNOUNCE_MS  5000

/* Carried in EV_SENSOR_LOST's detail field. */
typedef enum {
    SENSOR_LOST_NO_RESPONSE = 1,   /* ID register wrong: unplugged or a bus wire */
    SENSOR_LOST_RESET       = 2,   /* ID right, configuration gone: power-cycled */
    SENSOR_LOST_BUS_ERROR   = 3,   /* the bus driver itself failed (I2C NACK etc.) */
} sensor_lost_reason_t;

typedef struct {
    err_source_t source;           /* ERR_SRC_BMP388, ERR_SRC_LSM6DSOX */
    bool         lost;
    uint8_t      last_reason;      /* sensor_lost_reason_t, 0 = never lost */
    uint64_t     lost_at_us;
    uint64_t     next_retry_us;
    uint64_t     next_check_us;
    uint64_t     next_announce_us;
    uint32_t     lost_count;       /* times contact was lost */
    uint32_t     recoveries;       /* times it came back */
    uint32_t     retry_attempts;   /* re-init attempts, successful or not */
    uint32_t     last_outage_ms;   /* length of the most recent outage */
} sensor_health_t;

void sensor_health_init(sensor_health_t *h, err_source_t source, uint64_t now_us);

/* True once every SENSOR_HEALTH_CHECK_MS for a healthy sensor, and
 * reschedules itself when it says so. Always false while lost: a lost
 * sensor is probed by the retry, not by this. */
bool sensor_health_check_due(sensor_health_t *h, uint64_t now_us);

/* Contact lost. Records EV_SENSOR_LOST and logs at error level the
 * FIRST time only; calling it again while already lost does nothing,
 * so a driver can report every failed probe without flooding the log.
 * The first retry is due immediately -- a RESET in particular wants
 * re-initialising now, not a quarter second from now. */
void sensor_health_lost(sensor_health_t *h, sensor_lost_reason_t why, uint64_t now_us);

bool sensor_health_retry_due(const sensor_health_t *h, uint64_t now_us);

/* Re-record EV_SENSOR_LOST if the sensor is still lost and
 * SENSOR_HEALTH_REANNOUNCE_MS has passed since the last time. Call it as
 * often as convenient; sensor_health_retry_result() calls it on every
 * failed attempt. */
void sensor_health_poll(sensor_health_t *h, uint64_t now_us);

/* Back to healthy WITHOUT an event -- for a sensor that went quiet
 * because it was deliberately switched off (the GPS in STANDBY), which
 * is not a recovery and was never really a loss. */
void sensor_health_clear(sensor_health_t *h, uint64_t now_us);

/* How long the driver's task may block before the next retry is due,
 * in ms. 0 means "now". Only meaningful while lost. */
uint32_t sensor_health_retry_wait_ms(const sensor_health_t *h, uint64_t now_us);

/* Report the result of one re-init attempt. Success records
 * EV_SENSOR_RECOVERED with the outage length and returns the sensor to
 * healthy; failure schedules the next attempt. Individual failures are
 * counted, never logged -- the LOST event already says it is gone. */
void sensor_health_retry_result(sensor_health_t *h, bool ok, uint64_t now_us);

const char *sensor_health_reason_name(uint8_t reason);

#endif /* SENSOR_HEALTH_H */
