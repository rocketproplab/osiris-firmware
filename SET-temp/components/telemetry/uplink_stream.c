#include "uplink_stream.h"

#include <string.h>

/* Same CRC16-CCITT (poly 0x1021, init 0xFFFF) as every other packet in
 * the project. */
static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void uplink_stream_reset(uplink_stream_t *s)
{
    memset(s, 0, sizeof *s);
}

static void drop(uplink_stream_t *s, uint16_t k)
{
    if (k >= s->len) { s->len = 0; return; }
    memmove(s->buf, s->buf + k, (size_t)(s->len - k));
    s->len = (uint16_t)(s->len - k);
}

/* Total length of the packet starting at buf[0], or 0 if not yet known. */
static uint16_t needed(const uplink_stream_t *s)
{
    if (s->buf[0] == UPLINK_STREAM_CMD_MAGIC) {
        return UPLINK_STREAM_CMD_SIZE;
    }
    if (s->len < UPLINK_STREAM_CFG_HDR) {
        return 0;
    }
    return (uint16_t)(UPLINK_STREAM_CFG_HDR + s->buf[UPLINK_STREAM_CFG_HDR - 1] + 2);
}

/* One step at the head of the buffer. Returns false when it needs more
 * bytes to decide anything. */
static bool step(uplink_stream_t *s, uplink_stream_cb_t cb, void *ctx)
{
    if (s->len == 0) {
        return false;
    }
    const uint8_t b = s->buf[0];
    if (b != UPLINK_STREAM_CMD_MAGIC && b != UPLINK_STREAM_CFG_MAGIC) {
        drop(s, 1);
        s->discarded_bytes++;
        return true;
    }
    const uint16_t need = needed(s);
    if (need == 0 || s->len < need) {
        return false;
    }
    const uint16_t body = (uint16_t)(need - 2);
    const uint16_t got  = (uint16_t)(s->buf[body] | ((uint16_t)s->buf[body + 1] << 8));
    if (crc16(s->buf, body) != got) {
        s->crc_failures++;
        s->discarded_bytes++;
        drop(s, 1);
        return true;
    }
    s->packets++;
    if (cb) { cb(ctx, s->buf, need); }
    drop(s, need);
    return true;
}

void uplink_stream_push(uplink_stream_t *s, const uint8_t *data, size_t n,
                        uint32_t now_ms, uplink_stream_cb_t cb, void *ctx)
{
    /* A partial that has sat with no new bytes is not going to complete.
     * Peel it one byte at a time, so a valid packet that happens to sit
     * behind a bogus header is still found; whatever is left over is
     * the stale tail and goes too. */
    if (s->len > 0 && (uint32_t)(now_ms - s->last_byte_ms) >= UPLINK_STREAM_STALE_MS) {
        s->stale_flushes++;
        while (s->len > 0) {
            drop(s, 1);
            s->discarded_bytes++;
            while (step(s, cb, ctx)) { }
        }
    }

    for (size_t i = 0; i < n; i++) {
        /* Cannot overflow: after every byte the loop below runs until the
         * head is an incomplete packet, which is shorter than
         * UPLINK_STREAM_MAX_PACKET, and the buffer is larger than that.
         * The check stays as a guard against a future layout change. */
        if (s->len >= sizeof s->buf) {
            drop(s, 1);
            s->discarded_bytes++;
        }
        s->buf[s->len++] = data[i];
        while (step(s, cb, ctx)) { }
    }
    if (n > 0) {
        s->last_byte_ms = now_ms;
    }
}
