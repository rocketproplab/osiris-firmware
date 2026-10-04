#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_records.h"

#define TELEMETRY_MAX_INSTANCES   1
#define TELEMETRY_STACK_WORDS     4096

/* Depth is intentionally small relative to SD's per-stream buffers.
 * Telemetry is a live downlink, not an archive -- if the radio can't
 * keep up you want to drop old, stale-by-now samples and stay close to
 * real time, not build a deep backlog nobody will care about by the
 * time it arrives. */
#define TELEMETRY_QUEUE_DEPTH     48

/* ===================================================================
 *  Downlink over the XBee-PRO S3B, in API mode.
 *
 *  Shape of the thing:
 *
 *    data_router -> telemetry_write() -> ring buffer
 *                                          |
 *                                    telemetry_task
 *                                     packs a block
 *                                          |
 *                              [packet: header + records + CRC]
 *                                          |
 *                                  XBee 0x10 TX Request
 *
 *  Records accumulate until the block reaches TELEMETRY_BLOCK_BYTES or
 *  TELEMETRY_BLOCK_MAX_MS elapses, then the whole block goes out as one
 *  packet -- the same "buffer, then write once" trade sd_logger makes,
 *  for the same reason. One 200-byte packet costs one set of framing
 *  and one over-air transaction; forty 5-byte packets cost forty.
 *
 *  The timeout is what stops a slow stream from being held hostage. A
 *  1 Hz GPS fix behind a size-only threshold would wait for the IMU to
 *  fill the block; on a quiet link it might wait forever.
 *
 *  IMU DOWNSAMPLING. Every record is logged to SD, but only every Nth
 *  IMU sample is transmitted (TELEMETRY_IMU_DECIMATION). At 104 Hz an
 *  imu_record_t is ~3.7 kB/s of payload before overhead, which does not
 *  leave room for baro, GPS, battery, events and retries on the same
 *  link. The card keeps the full-rate truth; the radio carries a usable
 *  picture. Decimation happens in telemetry_write(), so nothing else in
 *  the project is affected by it.
 * =================================================================== */

/* Bytes of record payload per packet. The S3B's maximum RF payload is
 * 256; 200 leaves room for the 10-byte packet header, the per-record
 * stream tags, and a margin against a firmware-dependent lower cap. */
#define TELEMETRY_BLOCK_BYTES     200
#define TELEMETRY_BLOCK_MAX_MS    500
#define TELEMETRY_IMU_DECIMATION  4

/* Standby downlink. The XBee's transmit current dominates this board's
 * power budget, so on the pad the block timeout stretches to 5 s and the
 * IMU is thinned much harder. The link stays alive -- you can still see
 * the vehicle is healthy and still send it a command -- it just stops
 * paying full airtime for a rocket that is not moving. */
#define TELEMETRY_STANDBY_BLOCK_MS      5000
#define TELEMETRY_STANDBY_IMU_DECIM     32

/* Retry. Bounded on purpose: an unbounded queue of un-acked packets on
 * a dead link would stall everything behind packets nobody will hear,
 * which on a rocket is exactly the wrong trade -- the next fresh packet
 * is worth more than a perfect record of an old one. */
#define TELEMETRY_RETRY_SLOTS     4
#define TELEMETRY_MAX_RETRIES     1

/* How often the vehicle asks the module for its last-packet RSSI. */
#define TELEMETRY_RSSI_POLL_MS    1000

/* ===================================================================
 *  Module bring-up timing and the bootloader escape.
 *
 *  Digi documents the bootloader as running at 115200 8N1 with flow
 *  control off, which is what this link already uses -- so the escape
 *  can go out on the same UART with no reconfiguration.
 *
 *  THE ESCAPE SEQUENCE IS A GUESS THAT YOU CAN CORRECT. Digi's public
 *  docs describe entering the bootloader and say a carriage return
 *  elicits its menu, but do not publish the menu's keys for this
 *  module. XBEE_ESCAPE_BYTES below is CR, then the key observed on this
 *  vehicle's module, then CR. If your module's banner wants something
 *  else, this is the one line to change -- and nothing else depends on
 *  it, because success is judged by whether ATVR answers afterwards,
 *  never by parsing the banner. */
#define XBEE_RESET_PULSE_MS       5     /* /RESET held low; datasheet minimum is far less */
#define XBEE_BOOT_SETTLE_MS       350   /* module boot before it will answer anything */
#define XBEE_PROBE_TIMEOUT_MS     400   /* how long to wait for the ATVR response */
#define XBEE_PROBE_ATTEMPTS       3     /* ATVR tries per phase */
#define XBEE_ESCAPE_BYTES         "\rb\r"
#define XBEE_ESCAPE_SETTLE_MS     300   /* after the escape, before probing again */

#define TELEMETRY_MAX_PACKET      256
#define TELEMETRY_MAX_FRAME       (TELEMETRY_MAX_PACKET + 32)

/* ===================================================================
 *  Wire format -- little-endian, matching the records themselves.
 *  (The XBee's own frame fields around this are big-endian; that is
 *  Digi's convention and stops at the API layer.)
 *
 *    0  uint8   magic      0xA5
 *    1  uint8   ver_flags  high nibble version, low nibble flags
 *    2  uint16  seq        packet sequence, wraps at 65535
 *    4  uint8   rssi       -dBm magnitude at the VEHICLE, 0 = unknown
 *    5  uint8   count      records in this packet
 *    6  uint16  len        payload bytes that follow this header
 *    8  records            [uint8 stream][record bytes] x count
 *    8+len uint16 crc      CRC16-CCITT over bytes 0 .. 8+len
 *
 *  Records carry a 1-byte stream tag and no length: the length is
 *  implied by the stream via sensor_record_size(). That is a byte per
 *  record saved, and the version nibble is the guard -- bump it if any
 *  record's size ever changes, or an old decoder will read new data as
 *  plausible garbage.
 *
 *  WHAT THE RSSI FIELD ACTUALLY MEANS. It is read from the module's
 *  ATDB register, which reports the strength of the last packet the
 *  VEHICLE received. It therefore describes the uplink, and it is only
 *  meaningful if the ground station transmits periodically. It is NOT a
 *  measure of how well the ground station is hearing the rocket -- that
 *  can only be measured at the ground station. Do not use this field to
 *  judge downlink margin.
 * =================================================================== */

/* ===================================================================
 *  UPLINK command format -- ground station -> vehicle
 *
 *  Ten bytes, little-endian, carried as the RF payload of an XBee 0x10
 *  Transmit Request from the ground, arriving here inside a 0x90
 *  Receive Packet frame.
 *
 *    0  uint8   magic     0x5A
 *    1  uint8   ver_cmd   high nibble = version (1), low nibble = command
 *    2  uint16  seq       strictly increasing; a command whose seq is not
 *                         newer than the last ACCEPTED one is discarded
 *    4  uint32  arg       command-specific, 0 where unused
 *    8  uint16  crc       CRC16-CCITT over bytes 0..7
 *
 *  Three deliberate choices:
 *
 *  MAGIC IS 0x5A, the downlink's 0xA5 reversed. A downlink packet looped
 *  back by a repeater, a test harness, or the ground station's own echo
 *  can therefore never parse as a command.
 *
 *  SEQ MUST INCREASE. The XBee retransmits on its own, so the same
 *  command can legitimately arrive twice; without this a single ARM
 *  press would be logged as two. It also discards a replayed capture.
 *  Wrap-safe: newer means (seq - last) is in the lower half of the
 *  16-bit space, so the counter can roll over without locking up.
 *
 *  THE CRC IS NOT SECURITY. It rejects corruption, not an attacker. Two
 *  commands can cost you the record -- DISARM and SD_EJECT -- and in
 *  both cases it is the state machine that refuses them while the
 *  vehicle is flying, not the packet format. SD_EJECT is the stricter
 *  of the two: DISARM is allowed from ARMED, an eject is not, because
 *  a vehicle on the pad with its card unmounted is a vehicle that will
 *  fly with no flight recorder and report STATUS_FLAG_SD_OK clear the
 *  whole way up.
 * =================================================================== */

#define TELEMETRY_CMD_MAGIC       0x5A
#define TELEMETRY_CMD_VERSION     1
#define TELEMETRY_CMD_SIZE        10

typedef enum {
    CMD_NOP    = 0x0,
    CMD_ARM    = 0x1,   /* STANDBY -> ARMED */
    CMD_DISARM = 0x2,   /* -> STANDBY; refused while in FLIGHT */
    CMD_PING   = 0x3,   /* emit a status record immediately */
    CMD_MARK   = 0x4,   /* write an operator marker into event.bin */
    /* Flush the buffers, close the files and unmount the card, so it
     * can be pulled without corrupting the filesystem. Accepted only
     * in STANDBY and LANDED; refused in ARMED and FLIGHT, which is
     * recorded as EV_UPLINK_REJECTED. ONE WAY: there is deliberately
     * no remount opcode, so the only route back to logging is a power
     * cycle. A half-working remount that silently logged to nothing
     * would be worse than an obvious one, and the confirmation the
     * ground watches for -- STATUS_FLAG_SD_OK going clear -- means
     * exactly one thing if the bit can never come back on its own. */
    CMD_SD_EJECT = 0x5,
    /* Make the config that was uploaded into NVS the LIVE one, without
     * a power cycle. Accepted only in STANDBY and LANDED -- the same
     * gate as the eject, and for a related reason: swapping deployment
     * thresholds is not something to do to a vehicle that is armed or
     * flying. Storing a config is always allowed; only applying it is
     * gated, so an upload can be staged on the pad and committed when
     * the operator is ready. */
    CMD_CONFIG_APPLY = 0x6,
    CMD_COUNT
} telemetry_cmd_t;

#define TELEMETRY_PKT_MAGIC       0xA5
#define TELEMETRY_PKT_VERSION     1
#define TELEMETRY_PKT_HDR_SIZE    8
#define TELEMETRY_FLAG_RETRANSMIT 0x01

typedef struct {
    int      uart_port;      // UART_NUM_2 on this board
    int      tx_pin;         // ESP -> XBee DIN
    int      rx_pin;         // XBee DOUT -> ESP
    int      rst_pin;        // ESP -> XBee /RESET, active low. -1 = not wired.
    int      baud_rate;      // 0 => 115200
    uint64_t dest_addr;      // 0 => XBEE_ADDR_BROADCAST
    uint8_t  imu_decimation; // 0 => TELEMETRY_IMU_DECIMATION
} telemetry_config_t;

typedef struct {
    uint32_t records_queued;
    uint32_t records_dropped_decimated; // IMU samples skipped on purpose
    uint32_t records_sent;              // records that went into a packet
    uint32_t queue_full_drops;          // evicted before they could be packed
    uint32_t packets_built;
    uint32_t packets_sent;              // handed to the UART
    uint32_t packets_acked;             // TX Status said delivered
    uint32_t packets_failed;            // TX Status said it did not arrive
    uint32_t packets_retried;
    uint32_t packets_abandoned;         // out of retries, or no free slot
    uint32_t uart_write_errors;
    uint32_t bad_checksums;             // malformed frames from the module
    uint32_t cmds_accepted;             // uplink commands acted on
    uint32_t cmds_bad_crc;              // failed the packet CRC
    uint32_t cmds_stale;                // seq not newer -- retransmit or replay
    uint32_t cmds_malformed;            // wrong magic, version, size or opcode
    uint32_t cmds_refused;              // understood, but the state machine said no
    uint32_t bringup_resets;            // hardware resets issued to the module
    uint32_t bootloader_escapes;        // escape sequences sent
    uint8_t  last_rssi;                 // -dBm magnitude, 0 = unknown
    bool     radio_verified;            // an AT command round-tripped at boot
} telemetry_diag_t;

typedef struct telemetry_dev *telemetry_handle_t;

/* ===================================================================
 *  Bringing the radio up.
 *
 *  telemetry_init() does more than open a UART. In order:
 *
 *    1. configure the UART, so TX (the module's DIN) idles HIGH
 *    2. pulse the module's /RESET line, if one is wired
 *    3. prove the radio answers, with a real ATVR round-trip
 *    4. only if it does not answer, send the bootloader escape
 *       sequence and probe once more
 *
 *  WHY THE ORDER MATTERS. Digi's bootloader is entered by a serial
 *  BREAK on DIN across a reset -- that is, DIN held low longer than one
 *  character time while the module comes up. An ESP32 pad is an input
 *  until firmware configures it, so between board power-on and the
 *  uart_set_pin() below, DIN is held only by R10's 10k pull-up and is
 *  vulnerable to exactly that condition. Resetting the module AFTER the
 *  UART is driving DIN high gives it a clean boot, which is why step 2
 *  comes after step 1 and not before.
 *
 *  Step 3 is the part worth having regardless. "The UART opened" says
 *  nothing about whether a radio is on the other end of it; an ATVR
 *  that comes back does. diag.radio_verified carries the answer, and
 *  status_task puts it in STATUS_FLAG_TELEM_OK.
 *
 *  Does NOT put the module into API mode -- see the note in telemetry.c
 *  about why that is a bench configuration step, not a runtime one. A
 *  module left in transparent mode will fail step 3, log loudly, and
 *  still return ESP_OK: a radio that cannot be verified is a reason to
 *  fly with a warning, not a reason to refuse to boot. */
esp_err_t telemetry_init(const telemetry_config_t *cfg, telemetry_handle_t *out);

esp_err_t telemetry_start(telemetry_handle_t dev, UBaseType_t priority, BaseType_t core_id);
esp_err_t telemetry_stop(telemetry_handle_t dev);

/* Enqueue one record for downlink. Non-blocking, overwrite-oldest, same
 * as sd_logger_write(). IMU records are decimated here. */
esp_err_t telemetry_write(telemetry_handle_t dev, sensor_stream_t stream,
                          const void *record, size_t len);

void telemetry_get_diagnostics(telemetry_handle_t dev, telemetry_diag_t *out);

/* Returns true once per CMD_PING received, clearing the request. The
 * status producer polls this. Done as a flag rather than a callback so
 * telemetry needs no knowledge of who assembles a status record. */
bool telemetry_take_ping_request(telemetry_handle_t dev);

/* Returns true once per ACCEPTED CMD_SD_EJECT, clearing the request.
 * Same flag-not-callback shape as the ping above, and for a stronger
 * reason: unmounting means calling sd_logger_stop(), and telemetry has
 * no business knowing sd_logger exists -- the host test compiles
 * telemetry.c without it. Whoever owns the SD handle polls this and
 * does the work. That is main.c's status_task, which already polls for
 * pings, already holds every handle, and is the one task that can
 * afford to block for the ~0.5 s the unmount takes. */
bool telemetry_take_sd_eject_request(telemetry_handle_t dev);

/* Returns true once per ACCEPTED CMD_CONFIG_APPLY. Polled by whoever
 * owns the live deploy_config_t -- flight_logic's task -- so that the
 * config is swapped by the task that reads it, between ticks, instead
 * of underneath it from somewhere else. */
bool telemetry_take_config_apply_request(telemetry_handle_t dev);

/* ===================================================================
 *  Uplink hook
 *
 *  Not every uplink packet is a command. The config upload carries its
 *  own magic (0x5B) and is reassembled by a component telemetry knows
 *  nothing about -- deliberately: telemetry.c is compiled on its own by
 *  the host test, and making it depend on NVS and a reassembly buffer
 *  to parse a radio frame would be backwards.
 *
 *  So an RF payload is offered to the hook first. A hook that returns
 *  true has consumed the packet; false falls through to the ordinary
 *  command parser. No hook installed means the old behaviour exactly.
 * =================================================================== */
typedef bool (*telemetry_uplink_hook_t)(const uint8_t *payload, uint16_t len);

void telemetry_set_uplink_hook(telemetry_handle_t dev, telemetry_uplink_hook_t hook);

#endif // TELEMETRY_H
