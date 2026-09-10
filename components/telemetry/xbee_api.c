#include "xbee_api.h"

#include <string.h>

static uint8_t xbee_checksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(0xFF - (sum & 0xFF));
}

size_t xbee_build_tx_request(uint8_t frame_id, uint64_t dest64,
                             const uint8_t *payload, size_t payload_len,
                             uint8_t *out, size_t out_cap)
{
    /* 14 bytes of frame body before the payload: type, frame id,
     * 8-byte address, 2-byte 16-bit address, radius, options. Plus the
     * 3-byte header and 1-byte checksum around it. */
    const size_t body = 14 + payload_len;
    const size_t total = 3 + body + 1;
    if (!out || !payload || total > out_cap) {
        return 0;
    }

    size_t i = 0;
    out[i++] = XBEE_START_DELIMITER;
    out[i++] = (uint8_t)(body >> 8);
    out[i++] = (uint8_t)(body & 0xFF);

    size_t body_start = i;
    out[i++] = XBEE_FRAME_TX_REQUEST;
    out[i++] = frame_id;
    for (int s = 56; s >= 0; s -= 8) {
        out[i++] = (uint8_t)((dest64 >> s) & 0xFF);
    }
    out[i++] = 0xFF;   /* 16-bit address: 0xFFFE means "use the 64-bit one" */
    out[i++] = 0xFE;
    out[i++] = 0x00;   /* broadcast radius: 0 = maximum hops */
    out[i++] = 0x00;   /* options */
    memcpy(&out[i], payload, payload_len);
    i += payload_len;

    out[i++] = xbee_checksum(&out[body_start], body);
    return i;
}

size_t xbee_build_at_query(uint8_t frame_id, char c0, char c1,
                           uint8_t *out, size_t out_cap)
{
    const size_t body = 4;
    const size_t total = 3 + body + 1;
    if (!out || total > out_cap) {
        return 0;
    }
    size_t i = 0;
    out[i++] = XBEE_START_DELIMITER;
    out[i++] = 0x00;
    out[i++] = (uint8_t)body;
    size_t body_start = i;
    out[i++] = XBEE_FRAME_AT_COMMAND;
    out[i++] = frame_id;
    out[i++] = (uint8_t)c0;
    out[i++] = (uint8_t)c1;
    out[i++] = xbee_checksum(&out[body_start], body);
    return i;
}

void xbee_parser_reset(xbee_parser_t *p)
{
    if (!p) return;
    p->state = XBEE_PS_IDLE;
    p->expected_len = 0;
    p->got = 0;
    p->sum = 0;
    p->frame_len = 0;
}

bool xbee_parser_feed(xbee_parser_t *p, uint8_t b)
{
    if (!p) return false;

    switch (p->state) {
    case XBEE_PS_IDLE:
        /* Resync on the delimiter and nothing else. Any byte that is
         * not 0x7E while idle is discarded silently -- that is normal
         * during bring-up, when the module may still be emitting
         * transparent-mode text before AP=1 takes effect. */
        if (b == XBEE_START_DELIMITER) {
            p->state = XBEE_PS_LEN_HI;
        }
        break;

    case XBEE_PS_LEN_HI:
        p->expected_len = (uint16_t)(b << 8);
        p->state = XBEE_PS_LEN_LO;
        break;

    case XBEE_PS_LEN_LO:
        p->expected_len |= b;
        p->got = 0;
        p->sum = 0;
        if (p->expected_len == 0 || p->expected_len > XBEE_MAX_FRAME_DATA) {
            p->overruns++;
            p->state = XBEE_PS_IDLE;   /* nonsense length -- resync */
        } else {
            p->state = XBEE_PS_DATA;
        }
        break;

    case XBEE_PS_DATA:
        p->frame[p->got++] = b;
        p->sum = (uint8_t)(p->sum + b);
        if (p->got >= p->expected_len) {
            p->state = XBEE_PS_CHECKSUM;
        }
        break;

    case XBEE_PS_CHECKSUM: {
        uint8_t expect = (uint8_t)(0xFF - p->sum);
        p->state = XBEE_PS_IDLE;
        if (b == expect) {
            p->frame_len = p->got;
            return true;
        }
        p->bad_checksums++;
        break;
    }
    }
    return false;
}

bool xbee_parse_tx_status(const uint8_t *frame, uint16_t len,
                          uint8_t *frame_id, uint8_t *retry_count,
                          uint8_t *delivery_status)
{
    /* 0x8B: type, frame_id, dest16[2], retries, delivery, discovery */
    if (!frame || len < 7 || frame[0] != XBEE_FRAME_TX_STATUS) {
        return false;
    }
    if (frame_id)        *frame_id = frame[1];
    if (retry_count)     *retry_count = frame[4];
    if (delivery_status) *delivery_status = frame[5];
    return true;
}

bool xbee_parse_at_response(const uint8_t *frame, uint16_t len,
                            uint8_t *frame_id, char *c0, char *c1,
                            uint8_t *status, const uint8_t **value,
                            uint16_t *value_len)
{
    /* 0x88: type, frame_id, cmd[2], status, value... */
    if (!frame || len < 5 || frame[0] != XBEE_FRAME_AT_RESPONSE) {
        return false;
    }
    if (frame_id) *frame_id = frame[1];
    if (c0)       *c0 = (char)frame[2];
    if (c1)       *c1 = (char)frame[3];
    if (status)   *status = frame[4];
    if (value)    *value = &frame[5];
    if (value_len) *value_len = (uint16_t)(len - 5);
    return true;
}
