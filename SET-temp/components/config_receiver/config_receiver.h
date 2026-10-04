#ifndef CONFIG_RECEIVER_H
#define CONFIG_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ===================================================================
 *  Deployment config upload, ground -> vehicle.
 *
 *  This is the component flight_logic.c's comments have referred to for
 *  a while -- "the config_receiver code that saves incoming SET_CONFIG
 *  uploads from the host". It did not exist. deploy_logic_load_config()
 *  reads NVS at boot and nothing had ever written it, so every flight
 *  so far has run on the all-disabled safe defaults.
 *
 *  WHY A SECOND PACKET FORMAT
 *
 *  A command is ten bytes with a uint32 argument. A deployment config
 *  is four channels x eighteen event keys x (primary, secondary,
 *  threshold) plus latch, delay and manual_arm -- hundreds of bytes to
 *  a couple of kilobytes of JSON. It does not fit in an argument, and
 *  widening the command would make every command carry the cost of the
 *  one that needed it.
 *
 *  So config chunks have their OWN magic, 0x5B, one away from the
 *  command's 0x5A and nowhere near the downlink's 0xA5. A packet of one
 *  kind can never parse as the other, and telemetry's command parser is
 *  untouched by any of this.
 *
 *  WIRE FORMAT (little-endian, carried as an XBee 0x10 RF payload)
 *
 *      0   u8   magic        0x5B
 *      1   u8   ver_sub      ver<<4 | subtype (0 = CHUNK, 1 = COMMIT)
 *      2   u16  seq          strictly increasing, SHARED with the
 *                            command sequence space
 *      4   u16  cfg_crc      CRC16-CCITT over the whole config text.
 *                            Doubles as the transfer id.
 *      6   u16  chunk_idx    0-based (COMMIT: 0)
 *      8   u16  chunk_count
 *      10  u16  total_len    bytes in the complete config
 *      12  u8   payload_len  (COMMIT: 0)
 *      13  ...  payload
 *          u16  crc16        over bytes 0 .. 13+payload_len-1
 *
 *  cfg_crc IS THE TRANSFER ID, and that is what makes "resend the whole
 *  thing" safe. A chunk whose cfg_crc differs from the transfer in
 *  progress starts a new transfer and discards the old partial one, so
 *  a retry after an edit cannot silently interleave its chunks with the
 *  bytes of the previous attempt and commit a config that was never
 *  sent. Retrying the SAME config resumes the same transfer, so chunks
 *  that already arrived are simply re-stored over themselves.
 *
 *  seq is the command counter, not a second one. The vehicle's
 *  strictly-increasing rule therefore drops the XBee's own retransmits
 *  for free, exactly as it does for commands.
 *
 *  A COMMIT is what triggers evaluation, rather than the last chunk
 *  doing it. If the last chunk is the one that gets lost, a receiver
 *  that only evaluated on "final chunk arrived" would sit silent with a
 *  half-written buffer and the ground would have nothing to time out
 *  on. The commit always answers -- stored, or rejected with the reason
 *  and the first missing chunk.
 *
 *  Storing is always allowed. APPLYING -- making the stored config the
 *  live one -- is a separate uplink command and is gated to STANDBY and
 *  LANDED, so a config can be staged on the pad and committed when the
 *  operator is ready.
 * =================================================================== */

#define CONFIG_RX_MAGIC        0x5B
#define CONFIG_RX_VERSION      1
#define CONFIG_RX_HDR_SIZE     13
#define CONFIG_RX_SUB_CHUNK    0
#define CONFIG_RX_SUB_COMMIT   1

/* Biggest config that will be accepted. A fully populated four-channel
 * JSON with every event listed runs around 2.5 KB; this leaves room
 * without reserving a buffer nobody needs. It is static RAM, not heap:
 * a multi-kilobyte malloc on an uplink packet is an easy way to make
 * the radio path a memory-pressure problem. */
#define CONFIG_RX_MAX_BYTES    4096

/* Payload bytes per chunk. Kept well under the XBee's 256-byte RF
 * payload so the framing and the header always fit with room to spare. */
#define CONFIG_RX_CHUNK_BYTES  96

#define CONFIG_RX_MAX_CHUNKS   ((CONFIG_RX_MAX_BYTES + CONFIG_RX_CHUNK_BYTES - 1) \
                                / CONFIG_RX_CHUNK_BYTES)

typedef struct {
    bool     in_progress;
    uint16_t cfg_crc;          // transfer id of whatever is in the buffer
    uint16_t chunks_received;
    uint16_t chunks_expected;
    uint16_t total_len;
    uint32_t transfers_stored;
    uint32_t transfers_rejected;
    uint32_t chunks_dropped;   // malformed, bad CRC, or out of range
} config_receiver_status_t;

/* Wires up the sequence check. Safe to call before NVS is ready; the
 * write only happens on a commit. */
void config_receiver_init(void);

/* Offer one uplink RF payload. Returns true if this was a config packet
 * and has been consumed -- whether or not it was any good -- so that
 * the caller does not also try to parse it as a command. Matches
 * telemetry_uplink_hook_t. */
bool config_receiver_handle_packet(const uint8_t *payload, uint16_t len);

void config_receiver_get_status(config_receiver_status_t *out);

#endif // CONFIG_RECEIVER_H
