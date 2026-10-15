#ifndef UPLINK_STREAM_H
#define UPLINK_STREAM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ===================================================================
 *  Uplink packets out of a plain byte stream -- TRANSPARENT mode only.
 *
 *  In API mode the XBee hands over each received RF packet in its own
 *  0x90 frame, so a packet's start and end come for free. In transparent
 *  mode (AP=0) the module just prints the RF payload bytes on DOUT, and
 *  this is what finds the packets again.
 *
 *  It can, because every uplink packet is self-delimiting and carries a
 *  CRC:
 *
 *    0x5A  command        always 10 bytes, CRC16 over bytes 0..7
 *                         (telemetry.h, TELEMETRY_CMD_*)
 *    0x5B  config chunk   13-byte header, byte 12 = payload length,
 *                         then the payload, then CRC16 over all of it
 *                         (config_receiver.h, CONFIG_RX_*)
 *
 *  The layouts are restated here rather than included, so telemetry
 *  keeps compiling without config_receiver; test_uplink_stream.c checks
 *  that the two copies agree.
 *
 *  Resynchronisation: a candidate whose CRC fails costs ONE byte, and
 *  the search restarts at the next byte -- so a stray 0x5A in line
 *  noise cannot swallow the real command that follows it. A candidate
 *  that never completes (a stray 0x5B whose "length" byte points 200
 *  bytes into the future) is given up after UPLINK_STREAM_STALE_MS of
 *  silence, and anything valid behind it is still delivered.
 *
 *  Pure C, no ESP-IDF: host-tested directly.
 * =================================================================== */

#define UPLINK_STREAM_CMD_MAGIC     0x5A
#define UPLINK_STREAM_CMD_SIZE      10
#define UPLINK_STREAM_CFG_MAGIC     0x5B
#define UPLINK_STREAM_CFG_HDR       13
#define UPLINK_STREAM_MAX_PACKET    (UPLINK_STREAM_CFG_HDR + 255 + 2)

/* A partial packet older than this, with no new bytes, is abandoned.
 * A real packet arrives from the module in one burst (270 bytes at
 * 115200 baud is ~23 ms), so this is generous. */
#define UPLINK_STREAM_STALE_MS      250

typedef void (*uplink_stream_cb_t)(void *ctx, const uint8_t *pkt, uint16_t len);

typedef struct {
    uint8_t  buf[UPLINK_STREAM_MAX_PACKET + 16];
    uint16_t len;
    uint32_t last_byte_ms;
    uint32_t packets;           // CRC-valid packets delivered
    uint32_t discarded_bytes;   // junk, failed candidates, stale partials
    uint32_t crc_failures;      // candidates that completed but failed CRC
    uint32_t stale_flushes;     // partials abandoned after the timeout
} uplink_stream_t;

void uplink_stream_reset(uplink_stream_t *s);

/* Feed bytes (n may be 0: then only the stale timeout is applied). Calls
 * cb once for every complete, CRC-valid packet found, in order. */
void uplink_stream_push(uplink_stream_t *s, const uint8_t *data, size_t n,
                        uint32_t now_ms, uplink_stream_cb_t cb, void *ctx);

#endif // UPLINK_STREAM_H
