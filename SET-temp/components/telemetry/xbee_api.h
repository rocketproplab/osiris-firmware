#ifndef XBEE_API_H
#define XBEE_API_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ===================================================================
 *  XBee API-mode framing (AP=1) for the XBee-PRO S3B / 900HP.
 *
 *  API mode rather than transparent mode for one reason: transparent
 *  mode is a pipe, and a pipe cannot tell you whether anything arrived.
 *  API mode returns a TX Status frame per transmission with a delivery
 *  result, which is what makes "retry on failure" mean something rather
 *  than being a guess. It also gives a clean place to read RSSI.
 *
 *  Frame layout, all multi-byte fields BIG-endian (Digi's convention,
 *  the opposite of the little-endian records this project logs):
 *
 *    0x7E | len_hi len_lo | frame_type ... data ... | checksum
 *
 *  len counts everything between the length field and the checksum.
 *  checksum = 0xFF - (sum of those same bytes & 0xFF).
 * =================================================================== */

#define XBEE_START_DELIMITER      0x7E
#define XBEE_MAX_FRAME_DATA       300

/* Frame types this project uses. */
#define XBEE_FRAME_AT_COMMAND     0x08
#define XBEE_FRAME_TX_REQUEST     0x10
#define XBEE_FRAME_AT_RESPONSE    0x88
#define XBEE_FRAME_TX_STATUS      0x8B
#define XBEE_FRAME_RX_PACKET      0x90

/* Delivery status values in a 0x8B TX Status frame. 0 is success;
 * everything else is a reason it did not get there. */
#define XBEE_DELIVERY_SUCCESS     0x00
#define XBEE_DELIVERY_NO_ACK      0x01
#define XBEE_DELIVERY_CCA_FAIL    0x02
#define XBEE_DELIVERY_PURGED      0x03

#define XBEE_ADDR_BROADCAST       0x000000000000FFFFULL

/* Builds a 0x10 Transmit Request into `out`. Returns the total frame
 * length, or 0 if it would not fit. Frame overhead is 18 bytes. */
size_t xbee_build_tx_request(uint8_t frame_id, uint64_t dest64,
                             const uint8_t *payload, size_t payload_len,
                             uint8_t *out, size_t out_cap);

/* Builds a 0x08 local AT Command (no parameter -- a query). */
size_t xbee_build_at_query(uint8_t frame_id, char c0, char c1,
                           uint8_t *out, size_t out_cap);

/* ===================================================================
 *  Incremental parser.
 *
 *  Fed one byte at a time from the UART, because the UART hands back
 *  arbitrary chunk boundaries and a frame will routinely straddle two
 *  reads. Returns true when a complete, checksum-valid frame is ready
 *  in `frame`/`frame_len` -- frame[0] is the frame type.
 * =================================================================== */

typedef enum {
    XBEE_PS_IDLE = 0,
    XBEE_PS_LEN_HI,
    XBEE_PS_LEN_LO,
    XBEE_PS_DATA,
    XBEE_PS_CHECKSUM
} xbee_parse_state_t;

typedef struct {
    xbee_parse_state_t state;
    uint16_t expected_len;
    uint16_t got;
    uint8_t  sum;
    uint8_t  frame[XBEE_MAX_FRAME_DATA];
    uint16_t frame_len;
    uint32_t bad_checksums;
    uint32_t overruns;      // frame longer than XBEE_MAX_FRAME_DATA
} xbee_parser_t;

void xbee_parser_reset(xbee_parser_t *p);
bool xbee_parser_feed(xbee_parser_t *p, uint8_t byte);

/* Convenience accessors over a completed frame. Each returns false if
 * the frame is the wrong type or too short to hold the field. */
bool xbee_parse_tx_status(const uint8_t *frame, uint16_t len,
                          uint8_t *frame_id, uint8_t *retry_count,
                          uint8_t *delivery_status);
bool xbee_parse_at_response(const uint8_t *frame, uint16_t len,
                            uint8_t *frame_id, char *c0, char *c1,
                            uint8_t *status, const uint8_t **value,
                            uint16_t *value_len);

#endif // XBEE_API_H
