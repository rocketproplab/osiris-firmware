#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPS_MAX_INSTANCES    1
#define GPS_STACK_WORDS      4096   /* same size as the original xTaskCreate(...) call */

/* ===================================================================
 *  Public data -- plain value types, nothing to hide.
 * =================================================================== */

typedef struct {
    double   latitude;      // decimal degrees, + = North, - = South
    double   longitude;     // decimal degrees, + = East, - = West
    float    altitude_m;
    uint8_t  fix_quality;   // 0 = no fix, 1+ = valid fix
    uint8_t  satellites;

    uint64_t timestamp_us;  // esp_timer time of the most recent PPS edge
                             // at the moment this fix was parsed -- see
                             // the PPS notes in gps_sensor.c for the
                             // correlation caveat
    uint32_t seq;           // increments once per successful fix update;
                             // 0 means no fix has ever been received
} gps_data_t;

typedef struct {
    int        uart_port;   // e.g. UART_NUM_1 -- kept as a plain int,
                             // matching the original gps_init() signature,
                             // since this driver has its own dedicated
                             // UART, separate from the XBee's
    int        tx_pin;
    int        rx_pin;
    gpio_num_t pps_pin;      // PA1616D PPS output -- rising edge once per
                             // second, aligned to the UTC second boundary
    gpio_num_t en_pin;       // PA1616D EN input. The module has an internal
                             // pull-up, so it runs with this floating -- but
                             // "runs because nobody drove it" is not a state
                             // worth relying on next to pyro currents.
                             // GPIO_NUM_NC leaves it floating as before.
} gps_config_t;

typedef struct {
    uint32_t fixes;             // GGA sentences that produced a valid fix
    uint32_t sentences_parsed;  // GGA-family sentences seen, fix or not
    uint32_t parse_errors;      // malformed/too-short sentences
    uint32_t line_overflows;    // line buffer overflow -> resync resets
    uint32_t pps_pulses;        // PPS rising edges seen since start
} gps_diag_t;

/* ===================================================================
 *  Opaque handle -- struct gps_dev is only ever defined inside
 *  gps_sensor.c. Same idiom as bmp388_handle_t / lsm6dsox_handle_t.
 * =================================================================== */

typedef struct gps_dev *gps_handle_t;

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

/* Claims a slot from the static pool and brings up the UART peripheral
 * (driver install, baud/parity/stop-bit config, pin assignment). Does
 * not start the parsing task yet. On success, *out is a handle for
 * every other call to use. */
esp_err_t gps_init(const gps_config_t *cfg, gps_handle_t *out);

/* Spawns the FreeRTOS task that reads UART bytes, accumulates NMEA
 * sentences, and parses GGA fixes. */
esp_err_t gps_start(gps_handle_t dev, UBaseType_t priority, BaseType_t core_id);

/* Tears down the task and UART driver, and releases the pool slot. */
esp_err_t gps_stop(gps_handle_t dev);

/* ===================================================================
 *  Data access
 * =================================================================== */

/* Non-blocking copy of the most recent fix. Returns ESP_ERR_NOT_FOUND
 * if no valid fix has been received yet (out->seq will read 0 either
 * way, so out is always safe to inspect even on that error). */
esp_err_t gps_get_latest(gps_handle_t dev, gps_data_t *out);

/* Blocks until a fix newer than last_seq is published, or timeout.
 * Pass 0 for last_seq to just take whatever fix is current right now. */
esp_err_t gps_wait_for_fix(gps_handle_t dev, uint32_t last_seq,
                           gps_data_t *out, TickType_t timeout);

void gps_get_diagnostics(gps_handle_t dev, gps_diag_t *out);

/* ===================================================================
 *  One-time provisioning -- NOT part of normal startup
 *
 *  Reconfigures the physical GPS module's baud rate and fix update
 *  rate via PMTK commands, then updates THIS driver's own UART config
 *  to match. Meant to be run ONCE (e.g. from a standalone debug build,
 *  or a single manual call), not every boot -- most MTK-chipset
 *  modules, including the PA1616D, persist these settings across power
 *  cycles as long as the module has its battery backup. Once
 *  reconfigured, just update GPS_BAUD_RATE in gps_sensor.c to match
 *  and never call this again.
 *
 *  DANGER: the moment the baud-rate command is sent, the module starts
 *  talking at the NEW rate immediately. This function reconfigures the
 *  local UART to match in the same call, but if it's interrupted
 *  between those two steps (crash, power loss), you're left unable to
 *  talk to the module until you know to reconnect at the new rate.
 * =================================================================== */

esp_err_t gps_reconfigure(gps_handle_t dev, uint32_t new_baud, uint32_t update_rate_ms);

/* ===================================================================
 *  Power control (EN pin)
 * =================================================================== */

/* Drives the PA1616D's EN line. false puts the module into low-power
 * shutdown; true releases it.
 *
 * Be aware of what a power cycle costs: the module loses its acquired
 * almanac and any baud rate set by gps_reconfigure(), coming back at the
 * 9600 default and needing a cold start -- typically 30+ seconds to a
 * fix. That makes this a pad-side tool (save current before arming, or
 * force a known state), not something to reach for in flight.
 *
 * Returns ESP_ERR_NOT_SUPPORTED if en_pin was GPIO_NUM_NC. */
esp_err_t gps_set_enabled(gps_handle_t dev, bool enabled);

/* Whether the EN line is currently driven high. Reports the level this
 * driver last set, not a read-back of the pin. */
bool gps_is_enabled(gps_handle_t dev);

#endif // GPS_SENSOR_H
