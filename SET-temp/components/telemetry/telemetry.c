#include <stdio.h>
#include "telemetry.h"

#include <string.h>
#include <stdlib.h>

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"      /* CONFIG_TELEMETRY_XBEE_*; absent in host tests */
#endif
#endif

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "ring_buffer.h"
#include "event_log.h"
#include "xbee_api.h"
#include "flight_mode.h"
#include "uplink_stream.h"

static const char *TAG = "TELEMETRY";

/* menuconfig -> XBee telemetry link -> XBee serial mode. API is the
 * normal mode; transparent is the backup for a module that cannot be
 * (or has not been) put into AP=1. See components/telemetry/Kconfig. */
#if defined(CONFIG_TELEMETRY_XBEE_TRANSPARENT)
#define XBEE_TRANSPARENT 1
#else
#define XBEE_TRANSPARENT 0
#endif

#if XBEE_TRANSPARENT && defined(CONFIG_TELEMETRY_XBEE_TRANSPARENT_VERIFY)
#define XBEE_TRANSPARENT_VERIFY 1
#else
#define XBEE_TRANSPARENT_VERIFY 0
#endif

/* Silence either side of +++. Must exceed the module's GT (1000 ms by
 * default). Also used by an API build that suspects its module is
 * really in transparent mode. */
#if defined(CONFIG_TELEMETRY_XBEE_GUARD_MS)
#define XBEE_GUARD_MS CONFIG_TELEMETRY_XBEE_GUARD_MS
#else
#define XBEE_GUARD_MS 1100
#endif

#define TELEMETRY_UART_BUF_SIZE   1024
#define TELEMETRY_DEFAULT_BAUD    115200

/* One un-acked packet kept for possible re-send. */
typedef struct {
    bool     in_use;
    uint8_t  frame_id;
    uint8_t  retries_left;
    uint16_t seq;
    uint16_t packet_len;
    uint8_t  packet[TELEMETRY_MAX_PACKET];
} retry_slot_t;

struct telemetry_dev {
    bool                 in_use;
    bool                 initialized;
    telemetry_config_t   cfg;

    ring_buffer_t        rb;
    uint8_t              rb_storage[TELEMETRY_QUEUE_DEPTH * sizeof(queue_item_t)];

    /* Block being assembled: the packet's payload section, tagged
     * records, without the header or CRC. Oversized by one max record
     * so an append can never run off the end before the size check. */
    uint8_t              block[TELEMETRY_BLOCK_BYTES + SENSOR_RECORD_MAX_SIZE + 1];
    uint16_t             block_used;
    uint8_t              block_count;
    uint64_t             block_started_us;

    uint16_t             packet_seq;
    uint8_t              next_frame_id;
    uint8_t              imu_phase;

    retry_slot_t         retry[TELEMETRY_RETRY_SLOTS];

    xbee_parser_t        parser;
    uint64_t             last_rssi_poll_us;
#if XBEE_TRANSPARENT
    uplink_stream_t      ustream;     /* uplink packets out of raw DOUT bytes */
#endif

    /* Uplink command state. last_cmd_seq starts at 0 and no command may
     * use seq 0, so the very first real command is always newer. */
    uint16_t             last_cmd_seq;
    bool                 have_cmd_seq;
    volatile bool        ping_requested;
    volatile bool        sd_eject_requested;
    volatile bool        config_apply_requested;
    telemetry_uplink_hook_t uplink_hook;

    StaticTask_t         tcb;
    StackType_t          stack[TELEMETRY_STACK_WORDS];
    TaskHandle_t         task;
    volatile bool        task_should_run;

    portMUX_TYPE         diag_lock;
    telemetry_diag_t     diag;
};

static struct telemetry_dev s_pool[TELEMETRY_MAX_INSTANCES];
static portMUX_TYPE         s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* Same CRC the SD log entries use, deliberately: one checksum to reason
 * about, and a ground decoder that already knows it from
 * decode_logs.py. */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Frame IDs cycle 1..255. Zero is reserved by the XBee to mean "send me
 * no TX Status for this one", which would silently disable retry. */
#if !XBEE_TRANSPARENT || XBEE_TRANSPARENT_VERIFY   /* API frames are built */
static uint8_t next_frame_id(struct telemetry_dev *dev)
{
    dev->next_frame_id++;
    if (dev->next_frame_id == 0) {
        dev->next_frame_id = 1;
    }
    return dev->next_frame_id;
}
#endif

static uint16_t telemetry_build_packet(struct telemetry_dev *dev,
                                       uint8_t *out, uint8_t flags)
{
    uint16_t len = dev->block_used;
    uint16_t total = (uint16_t)(TELEMETRY_PKT_HDR_SIZE + len + 2);
    if (total > TELEMETRY_MAX_PACKET) {
        return 0;
    }

    uint8_t rssi;
    portENTER_CRITICAL(&dev->diag_lock);
    rssi = dev->diag.last_rssi;
    portEXIT_CRITICAL(&dev->diag_lock);

    out[0] = TELEMETRY_PKT_MAGIC;
    out[1] = (uint8_t)((TELEMETRY_PKT_VERSION << 4) | (flags & 0x0F));
    out[2] = (uint8_t)(dev->packet_seq & 0xFF);
    out[3] = (uint8_t)(dev->packet_seq >> 8);
    out[4] = rssi;
    out[5] = dev->block_count;
    out[6] = (uint8_t)(len & 0xFF);
    out[7] = (uint8_t)(len >> 8);
    memcpy(&out[TELEMETRY_PKT_HDR_SIZE], dev->block, len);

    uint16_t crc = crc16_ccitt(out, (size_t)(TELEMETRY_PKT_HDR_SIZE + len));
    out[TELEMETRY_PKT_HDR_SIZE + len]     = (uint8_t)(crc & 0xFF);
    out[TELEMETRY_PKT_HDR_SIZE + len + 1] = (uint8_t)(crc >> 8);
    return total;
}

static bool telemetry_uart_send(struct telemetry_dev *dev,
                                const uint8_t *packet, uint16_t packet_len,
                                uint8_t frame_id)
{
#if XBEE_TRANSPARENT
    /* The packet itself, nothing around it. The module sends whatever
     * arrives on DIN to its stored DH/DL, cutting an RF packet when DIN
     * goes quiet for RO character times or NP bytes have built up. One
     * uart_write_bytes() keeps the packet's bytes back to back, so it
     * normally leaves as one RF packet; the ground station copes when it
     * does not.
     *
     * No accidental command mode: +++ is only honoured with GT of
     * silence on BOTH sides, and every packet's bytes go out in one
     * contiguous burst. */
    (void)frame_id;
    const uint8_t *frame = packet;
    size_t n = packet_len;
#else
    uint8_t frame[TELEMETRY_MAX_FRAME];
    uint64_t dest = dev->cfg.dest_addr ? dev->cfg.dest_addr : XBEE_ADDR_BROADCAST;
    size_t n = xbee_build_tx_request(frame_id, dest, packet, packet_len,
                                     frame, sizeof(frame));
    if (n == 0) {
        return false;
    }
#endif

    int written = uart_write_bytes(dev->cfg.uart_port, (const char *)frame, n);
    if (written != (int)n) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.uart_write_errors++;
        portEXIT_CRITICAL(&dev->diag_lock);
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_WRITE_FAILED, ESP_FAIL, (uint32_t)n);
        return false;
    }
    return true;
}

static void telemetry_flush_block(struct telemetry_dev *dev)
{
    if (dev->block_count == 0) {
        return;
    }

    uint8_t packet[TELEMETRY_MAX_PACKET];
    uint16_t packet_len = telemetry_build_packet(dev, packet, 0);
    uint8_t  count      = dev->block_count;

    /* Cleared before the send, not after. A UART failure must not leave
     * the same records staged to be packed again -- the retry path owns
     * re-sending, and duplicating here would put identical samples out
     * under two different sequence numbers. */
    dev->block_used  = 0;
    dev->block_count = 0;

    if (packet_len == 0) {
        return;
    }

#if XBEE_TRANSPARENT
    /* No TX Status ever comes back in transparent mode, so a parked copy
     * would never be released: every slot would fill and each new packet
     * would "abandon" the oldest. Send and forget; the radio's own MAC
     * retries are all there is. */
    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.packets_built++;
    dev->diag.records_sent += count;
    portEXIT_CRITICAL(&dev->diag_lock);
    if (telemetry_uart_send(dev, packet, packet_len, 0)) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.packets_sent++;
        portEXIT_CRITICAL(&dev->diag_lock);
    }
    dev->packet_seq++;
#else
    uint8_t frame_id = next_frame_id(dev);

    /* Park a copy for retry BEFORE sending, so a TX Status arriving
     * while we are still in this function finds its slot. */
    retry_slot_t *slot = NULL;
    for (int i = 0; i < TELEMETRY_RETRY_SLOTS; i++) {
        if (!dev->retry[i].in_use) { slot = &dev->retry[i]; break; }
    }
    if (!slot) {
        /* Every slot waits on an ack that never came. The link is not
         * keeping up, so give up on the OLDEST un-acked packet: it is
         * the most stale, and holding it would block a fresher one.
         * Comparison is modular so a wrapped sequence still orders. */
        slot = &dev->retry[0];
        for (int i = 1; i < TELEMETRY_RETRY_SLOTS; i++) {
            if ((uint16_t)(slot->seq - dev->retry[i].seq) < 0x8000) {
                slot = &dev->retry[i];
            }
        }
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.packets_abandoned++;
        portEXIT_CRITICAL(&dev->diag_lock);
    }

    slot->in_use       = true;
    slot->frame_id     = frame_id;
    slot->retries_left = TELEMETRY_MAX_RETRIES;
    slot->seq          = dev->packet_seq;
    slot->packet_len   = packet_len;
    memcpy(slot->packet, packet, packet_len);

    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.packets_built++;
    dev->diag.records_sent += count;
    portEXIT_CRITICAL(&dev->diag_lock);

    if (telemetry_uart_send(dev, packet, packet_len, frame_id)) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.packets_sent++;
        portEXIT_CRITICAL(&dev->diag_lock);
    } else {
        slot->in_use = false;   /* never left the vehicle; nothing to retry */
    }

    dev->packet_seq++;
#endif /* XBEE_TRANSPARENT */
}

#if !XBEE_TRANSPARENT
static void telemetry_handle_tx_status(struct telemetry_dev *dev,
                                       const uint8_t *frame, uint16_t len)
{
    uint8_t frame_id = 0, retries = 0, status = 0xFF;
    if (!xbee_parse_tx_status(frame, len, &frame_id, &retries, &status)) {
        return;
    }

    retry_slot_t *slot = NULL;
    for (int i = 0; i < TELEMETRY_RETRY_SLOTS; i++) {
        if (dev->retry[i].in_use && dev->retry[i].frame_id == frame_id) {
            slot = &dev->retry[i];
            break;
        }
    }

    if (status == XBEE_DELIVERY_SUCCESS) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.packets_acked++;
        portEXIT_CRITICAL(&dev->diag_lock);
        if (slot) { slot->in_use = false; }
        return;
    }

    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.packets_failed++;
    portEXIT_CRITICAL(&dev->diag_lock);

    if (!slot) {
        return;   /* already abandoned, or a status for a frame we evicted */
    }

    if (slot->retries_left == 0) {
        slot->in_use = false;
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.packets_abandoned++;
        portEXIT_CRITICAL(&dev->diag_lock);
        return;
    }

    slot->retries_left--;
    slot->frame_id = next_frame_id(dev);

    /* Mark the copy as a retransmit and fix the CRC, so the ground
     * station can tell a deliberate re-send from a duplicate arriving
     * for some other reason. */
    slot->packet[1] = (uint8_t)(slot->packet[1] | TELEMETRY_FLAG_RETRANSMIT);
    uint16_t body = (uint16_t)(slot->packet_len - 2);
    uint16_t crc  = crc16_ccitt(slot->packet, body);
    slot->packet[body]     = (uint8_t)(crc & 0xFF);
    slot->packet[body + 1] = (uint8_t)(crc >> 8);

    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.packets_retried++;
    portEXIT_CRITICAL(&dev->diag_lock);

    if (!telemetry_uart_send(dev, slot->packet, slot->packet_len, slot->frame_id)) {
        slot->in_use = false;
    }
}

static void telemetry_handle_at_response(struct telemetry_dev *dev,
                                         const uint8_t *frame, uint16_t len)
{
    char c0 = 0, c1 = 0; uint8_t status = 0xFF, fid = 0;
    const uint8_t *value = NULL; uint16_t value_len = 0;
    if (!xbee_parse_at_response(frame, len, &fid, &c0, &c1, &status,
                                &value, &value_len)) {
        return;
    }
    if (c0 == 'D' && c1 == 'B' && status == 0 && value_len >= 1) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.last_rssi = value[0];
        portEXIT_CRITICAL(&dev->diag_lock);
    }
}
#endif /* !XBEE_TRANSPARENT */

/* ===================================================================
 *  Uplink commands
 *
 *  Every rejection path increments its own counter and, where the
 *  command was well-formed enough to be understood, records an event.
 *  A command that is silently dropped is indistinguishable afterwards
 *  from one that never arrived, and those want different
 *  investigations -- the same reasoning flight_mode applies to a
 *  refused disarm.
 * =================================================================== */

/* TEMPORARY -- uplink trace. Set XBEE_RX_TRACE to 0 to silence.
 * Logs every 0x90 that reaches the parser, and makes the two rejection
 * paths that are otherwise silent (wrong magic/length, stale seq) say
 * so. Answers "did the packet arrive?" separately from "was it
 * accepted?". */
#define XBEE_RX_TRACE  1

#if XBEE_RX_TRACE
static void xbee_trace_hex(const char *what, const uint8_t *p, uint16_t len)
{
    char hex[24 * 3 + 1];
    const uint16_t n = len < 24 ? len : 24;
    for (uint16_t k = 0; k < n; k++) { snprintf(&hex[k * 3], 4, "%02X ", p[k]); }
    hex[n * 3] = '\0';
    ESP_LOGW(TAG, "rx trace: %s (%u bytes): %s%s", what, (unsigned)len, hex,
             len > n ? "..." : "");
}
#endif

static bool seq_is_newer(uint16_t seq, uint16_t last)
{
    /* Wrap-safe: newer means the unsigned distance is in the lower half
     * of the space, so the counter can roll 65535 -> 0 without the link
     * locking up until reboot. */
    return (uint16_t)(seq - last) != 0 && (uint16_t)(seq - last) < 0x8000;
}

static void telemetry_handle_command(struct telemetry_dev *dev,
                                     const uint8_t *pkt, uint16_t len)
{
    if (len < TELEMETRY_CMD_SIZE || pkt[0] != TELEMETRY_CMD_MAGIC) {
        portENTER_CRITICAL(&dev->diag_lock); dev->diag.cmds_malformed++;
        portEXIT_CRITICAL(&dev->diag_lock);
#if XBEE_RX_TRACE
        ESP_LOGW(TAG, "rx trace: not a command -- len %u (need %d), first byte 0x%02X "
                      "(need 0x%02X)", (unsigned)len, TELEMETRY_CMD_SIZE,
                 len ? pkt[0] : 0, TELEMETRY_CMD_MAGIC);
#endif
        return;
    }
    if ((pkt[1] >> 4) != TELEMETRY_CMD_VERSION) {
        portENTER_CRITICAL(&dev->diag_lock); dev->diag.cmds_malformed++;
        portEXIT_CRITICAL(&dev->diag_lock);
        ESP_LOGW(TAG, "uplink: unsupported command version %u", (unsigned)(pkt[1] >> 4));
        return;
    }

    uint16_t crc_rx = (uint16_t)(pkt[8] | ((uint16_t)pkt[9] << 8));
    if (crc16_ccitt(pkt, 8) != crc_rx) {
        portENTER_CRITICAL(&dev->diag_lock); dev->diag.cmds_bad_crc++;
        portEXIT_CRITICAL(&dev->diag_lock);
        ESP_LOGW(TAG, "uplink: CRC failed -- discarded");
        return;
    }

    uint8_t  cmd = pkt[1] & 0x0F;
    uint16_t seq = (uint16_t)(pkt[2] | ((uint16_t)pkt[3] << 8));
    uint32_t arg = (uint32_t)pkt[4] | ((uint32_t)pkt[5] << 8) |
                   ((uint32_t)pkt[6] << 16) | ((uint32_t)pkt[7] << 24);

    if (dev->have_cmd_seq && !seq_is_newer(seq, dev->last_cmd_seq)) {
        portENTER_CRITICAL(&dev->diag_lock); dev->diag.cmds_stale++;
        portEXIT_CRITICAL(&dev->diag_lock);
#if XBEE_RX_TRACE
        ESP_LOGW(TAG, "rx trace: STALE -- seq %u is not newer than last accepted %u. "
                      "Reboot the vehicle or reset tools/.uplink_seq.",
                 (unsigned)seq, (unsigned)dev->last_cmd_seq);
#endif
        return;   /* the XBee's own retransmit, or a replay */
    }

    bool acted = false;
    switch (cmd) {
    case CMD_NOP:
        acted = true;
        break;
    case CMD_ARM:
        acted = flight_mode_arm(MODE_SRC_UPLINK);
        break;
    case CMD_DISARM:
        /* flight_mode records EV_UPLINK_REJECTED itself when it refuses,
         * because it is the thing that knows why. */
        acted = flight_mode_disarm(MODE_SRC_UPLINK);
        break;
    case CMD_PING:
        dev->ping_requested = true;
        acted = true;
        break;
    case CMD_MARK:
        event_log_event(EV_UPLINK_COMMAND, (uint8_t)CMD_MARK, arg);
        acted = true;
        break;
    case CMD_CONFIG_APPLY: {
        /* Same gate as the eject, same reasoning, and the mode is read
         * once so the decision and the logged detail cannot disagree. */
        flight_mode_t m = flight_mode_get();
        if (m == MODE_STANDBY || m == MODE_LANDED) {
            dev->config_apply_requested = true;
            acted = true;
        } else {
            event_log_event(EV_UPLINK_REJECTED, (uint8_t)CMD_CONFIG_APPLY, (uint32_t)m);
            ESP_LOGW(TAG, "uplink: config apply refused in %s", flight_mode_name(m));
            acted = false;
        }
        break;
    }
    case CMD_SD_EJECT: {
        /* The gate is here rather than in whoever performs the unmount
         * because this is where a refusal can still be told apart from
         * a command that never arrived -- the same reason flight_mode
         * records its own refused disarm. The mode is read once and
         * used for both the decision and the event detail, so the log
         * says what state the vehicle believed it was in.
         *
         * STANDBY and LANDED only. ARMED is excluded on purpose even
         * though disarm is allowed from it: disarm is reversible and an
         * eject is not, and the pad is exactly where somebody reaches
         * for the wrong button. */
        flight_mode_t m = flight_mode_get();
        if (m == MODE_STANDBY || m == MODE_LANDED) {
            dev->sd_eject_requested = true;
            /* Answer with a status record immediately instead of making
             * the operator wait out the 1 Hz tick: the unmount happens
             * in the same task that builds that record, so the first
             * status after this one is the one with SD_OK clear. That
             * is the whole confirmation path, and it should be fast. */
            dev->ping_requested = true;
            acted = true;
        } else {
            event_log_event(EV_UPLINK_REJECTED, (uint8_t)CMD_SD_EJECT, (uint32_t)m);
            ESP_LOGW(TAG, "uplink: SD eject refused in %s", flight_mode_name(m));
            acted = false;
        }
        break;
    }
    default:
        portENTER_CRITICAL(&dev->diag_lock); dev->diag.cmds_malformed++;
        portEXIT_CRITICAL(&dev->diag_lock);
        ESP_LOGW(TAG, "uplink: unknown opcode 0x%X", (unsigned)cmd);
        return;
    }

    /* Accepted for sequencing purposes even when the state machine
     * declined to act -- otherwise a refused DISARM would leave the
     * window open for its own retransmit to be reconsidered forever. */
    dev->last_cmd_seq = seq;
    dev->have_cmd_seq = true;

    portENTER_CRITICAL(&dev->diag_lock);
    if (acted) { dev->diag.cmds_accepted++; } else { dev->diag.cmds_refused++; }
    portEXIT_CRITICAL(&dev->diag_lock);

    if (acted && cmd != CMD_MARK) {
        event_log_event(EV_UPLINK_COMMAND, (uint8_t)cmd, (uint32_t)seq);
    }
    ESP_LOGW(TAG, "uplink cmd=0x%X seq=%u arg=%lu -> %s",
             (unsigned)cmd, (unsigned)seq, (unsigned long)arg,
             acted ? "ACCEPTED" : "refused");
}

#if XBEE_TRANSPARENT
/* One complete, CRC-valid uplink packet found in the byte stream. From
 * here on it is treated exactly like the payload of a 0x90 frame. */
static void telemetry_on_stream_packet(void *ctx, const uint8_t *pl, uint16_t len)
{
    struct telemetry_dev *dev = (struct telemetry_dev *)ctx;
#if XBEE_RX_TRACE
    xbee_trace_hex("uplink packet (transparent)", pl, len);
#endif
    if (!dev->uplink_hook || !dev->uplink_hook(pl, len)) {
        telemetry_handle_command(dev, pl, len);
    }
}

static void telemetry_pump_rx(struct telemetry_dev *dev)
{
    uint8_t buf[128];
    int n = uart_read_bytes(dev->cfg.uart_port, buf, sizeof(buf), 0);
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    /* Called with n == 0 too: that is what lets a stale partial time out
     * on a quiet link. */
    uplink_stream_push(&dev->ustream, buf, n > 0 ? (size_t)n : 0, now_ms,
                       telemetry_on_stream_packet, dev);
    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.uplink_discarded_bytes = dev->ustream.discarded_bytes;
    portEXIT_CRITICAL(&dev->diag_lock);
}
#else
static void telemetry_pump_rx(struct telemetry_dev *dev)
{
    uint8_t buf[128];
    int n = uart_read_bytes(dev->cfg.uart_port, buf, sizeof(buf), 0);
    for (int i = 0; i < n; i++) {
        if (!xbee_parser_feed(&dev->parser, buf[i])) {
            continue;
        }
        const uint8_t *f = dev->parser.frame;
        uint16_t flen = dev->parser.frame_len;
        switch (f[0]) {
        case XBEE_FRAME_TX_STATUS:   telemetry_handle_tx_status(dev, f, flen); break;
        case XBEE_FRAME_AT_RESPONSE: telemetry_handle_at_response(dev, f, flen); break;
        case XBEE_FRAME_RX_PACKET:
#if XBEE_RX_TRACE
            xbee_trace_hex("0x90 received, payload", flen > 12 ? &f[12] : f,
                           flen > 12 ? (uint16_t)(flen - 12) : flen);
#endif
            /* 0x90: type, src64[8], reserved[2], options, then payload.
             * The hook gets first refusal -- see the uplink hook notes
             * in telemetry.h. */
            if (flen > 12) {
                const uint8_t *pl = &f[12];
                const uint16_t pl_len = (uint16_t)(flen - 12);
                if (!dev->uplink_hook || !dev->uplink_hook(pl, pl_len)) {
                    telemetry_handle_command(dev, pl, pl_len);
                }
            }
            break;
        default:
#if XBEE_RX_TRACE
            xbee_trace_hex("unhandled frame", f, flen);
#endif
            break;
        }
    }
#if XBEE_RX_TRACE
    if (dev->parser.bad_checksums != dev->diag.bad_checksums) {
        ESP_LOGW(TAG, "rx trace: XBee frame checksum failures now %u",
                 (unsigned)dev->parser.bad_checksums);
    }
#endif
    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.bad_checksums = dev->parser.bad_checksums;
    portEXIT_CRITICAL(&dev->diag_lock);
}
#endif /* XBEE_TRANSPARENT */

static bool telemetry_stage_record(struct telemetry_dev *dev, const queue_item_t *item)
{
    uint8_t rec_size = sensor_record_size((sensor_stream_t)item->stream);
    if (rec_size == 0 || rec_size != item->len) {
        return false;   /* stream/size disagreement -- refuse it */
    }
    if (dev->block_used + 1 + rec_size > TELEMETRY_BLOCK_BYTES) {
        telemetry_flush_block(dev);
        dev->block_started_us = (uint64_t)esp_timer_get_time();
    }
    dev->block[dev->block_used++] = item->stream;
    memcpy(&dev->block[dev->block_used], item->payload, rec_size);
    dev->block_used = (uint16_t)(dev->block_used + rec_size);
    dev->block_count++;
    return true;
}

/* One pass of the telemetry task: uplink in, records staged, block
 * flushed on timeout, RSSI polled. Returns true if any record was
 * staged. Split out of the task so the host test can drive it. */
static bool telemetry_service(struct telemetry_dev *dev)
{
    queue_item_t item;
    telemetry_pump_rx(dev);

    bool got_one = false;
    while (ring_buffer_pop(&dev->rb, &item)) {
        got_one = true;
        telemetry_stage_record(dev, &item);
    }

    uint64_t now = (uint64_t)esp_timer_get_time();

    /* Size threshold is handled while staging; this is the timeout
     * that stops a slow stream being held hostage. A 1 Hz GPS fix
     * behind a size-only rule would wait for the IMU to fill the
     * block, and on a quiet link it might wait forever. */
    uint32_t block_ms = flight_mode_is_fast() ? TELEMETRY_BLOCK_MAX_MS
                                              : TELEMETRY_STANDBY_BLOCK_MS;
    if (dev->block_count > 0 &&
        (now - dev->block_started_us) >= (uint64_t)block_ms * 1000ULL) {
        telemetry_flush_block(dev);
        dev->block_started_us = now;
    }

#if !XBEE_TRANSPARENT
    /* Transparent mode has no way to ask: ATDB would need command mode,
     * which means a second of silence either side of +++ with the
     * downlink stopped. last_rssi stays 0, i.e. "unknown". */
    if ((now - dev->last_rssi_poll_us) >= (uint64_t)TELEMETRY_RSSI_POLL_MS * 1000ULL) {
        uint8_t f[16];
        size_t n = xbee_build_at_query(next_frame_id(dev), 'D', 'B', f, sizeof(f));
        if (n) {
            uart_write_bytes(dev->cfg.uart_port, (const char *)f, n);
        }
        dev->last_rssi_poll_us = now;
    }
#endif
    return got_one;
}

static void telemetry_task(void *arg)
{
    struct telemetry_dev *dev = (struct telemetry_dev *)arg;
    queue_item_t item;

    ESP_LOGI(TAG, "telemetry task running on core %d (XBee %s mode)",
             (int)xPortGetCoreID(), XBEE_TRANSPARENT ? "TRANSPARENT" : "API");

    dev->block_started_us = (uint64_t)esp_timer_get_time();

    while (dev->task_should_run) {
        if (!telemetry_service(dev)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    /* Shutdown: get whatever is queued into the air rather than
     * discarding it. Best effort -- no waiting for acks. */
    while (ring_buffer_pop(&dev->rb, &item)) {
        telemetry_stage_record(dev, &item);
    }
    telemetry_flush_block(dev);

    ESP_LOGI(TAG, "telemetry task exiting");
    dev->task = NULL;
    vTaskDelete(NULL);
}

/* ===================================================================
 *  Module bring-up: reset, prove it is there, escape if it is not.
 * =================================================================== */

/* Drives the module's /RESET low and releases it.
 *
 * Open-drain, not push-pull. The reset net may carry its own pull-up on
 * the module or the board, and an open-drain output can pull it low
 * without ever contending with whatever else is holding it high. A
 * push-pull output that disagrees with a pull-up is merely wasteful; one
 * that disagrees with a driver is a fight. */
static void xbee_hard_reset(struct telemetry_dev *dev)
{
    if (dev->cfg.rst_pin < 0) { return; }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << dev->cfg.rst_pin,
        .mode         = GPIO_MODE_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGW(TAG, "reset pin %d config failed -- skipping hardware reset",
                 dev->cfg.rst_pin);
        return;
    }

    gpio_set_level(dev->cfg.rst_pin, 1);          /* released */
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(dev->cfg.rst_pin, 0);          /* asserted */
    vTaskDelay(pdMS_TO_TICKS(XBEE_RESET_PULSE_MS));
    gpio_set_level(dev->cfg.rst_pin, 1);          /* released */

    dev->diag.bringup_resets++;
    vTaskDelay(pdMS_TO_TICKS(XBEE_BOOT_SETTLE_MS));
}

/* Sends ATVR and waits for the matching AT response.
 *
 * ATVR is chosen because it is read-only, always answers, and answers
 * with something worth logging. A command that changed state would be a
 * poor choice for a liveness probe -- if the probe is what is broken,
 * the last thing it should do is leave the radio different.
 *
 * This runs BEFORE the telemetry task exists, so it pumps the UART
 * itself rather than going through telemetry_pump_rx(). Frames that are
 * not the response we want (a stray 0x90 from a ground station already
 * transmitting, say) are parsed and discarded -- the parser must stay
 * fed or it would resynchronise mid-frame later. */
#if !XBEE_TRANSPARENT || XBEE_TRANSPARENT_VERIFY
static bool xbee_probe_once(struct telemetry_dev *dev, uint32_t *version_out)
{
    uint8_t frame[XBEE_MAX_FRAME_DATA];
    const uint8_t want_id = next_frame_id(dev);

    size_t n = xbee_build_at_query(want_id, 'V', 'R', frame, sizeof(frame));
    if (n == 0 || uart_write_bytes(dev->cfg.uart_port, (const char *)frame, n) != (int)n) {
        return false;
    }

    const int64_t deadline = esp_timer_get_time() + (int64_t)XBEE_PROBE_TIMEOUT_MS * 1000;
    while (esp_timer_get_time() < deadline) {
        uint8_t buf[64];
        int got = uart_read_bytes(dev->cfg.uart_port, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < got; i++) {
            if (!xbee_parser_feed(&dev->parser, buf[i])) { continue; }

            const uint8_t *f = dev->parser.frame;
            uint16_t flen = dev->parser.frame_len;
            uint8_t id = 0, status = 0;
            const uint8_t *val = NULL;
            uint16_t val_len = 0;
            char c0 = 0, c1 = 0;

            if (f[0] == XBEE_FRAME_AT_RESPONSE &&
                xbee_parse_at_response(f, flen, &id, &c0, &c1, &status, &val, &val_len) &&
                id == want_id && c0 == 'V' && c1 == 'R' && status == 0) {
                uint32_t v = 0;
                for (uint16_t k = 0; k < val_len && k < 4; k++) {
                    v = (v << 8) | val[k];
                }
                if (version_out) { *version_out = v; }
                return true;
            }
        }
    }
    return false;
}

static bool xbee_probe(struct telemetry_dev *dev, uint32_t *version_out)
{
    for (int i = 0; i < XBEE_PROBE_ATTEMPTS; i++) {
        if (xbee_probe_once(dev, version_out)) { return true; }
    }
    return false;
}
#endif

/* ===================================================================
 *  TEMPORARY -- raw UART dump.
 *
 *  Set XBEE_RAW_DUMP_MS to 0, or delete this block and its two calls in
 *  xbee_bringup(), once the radio is verified.
 *
 *  Without a scope this is the only way to see what the module actually
 *  puts on DOUT. Note that xbee_bringup() normally calls
 *  uart_flush_input() before it probes -- so anything the module said
 *  at power-up has always been discarded unread. These dumps run
 *  BEFORE those flushes.
 *
 *  Read the verdict line, not the hex:
 *
 *    API frames    bytes start 0x7E -- the module IS in API mode and
 *                  IS talking. If ATVR still fails after this, the
 *                  fault is in what we SEND on DIN, not the module.
 *    ASCII text    a bootloader banner or an AT-mode reply. The module
 *                  is not in API mode at this moment.
 *    no structure  neither -- a baud mismatch, or two drivers fighting
 *                  over the line.
 *    SILENT        nothing at all, so nothing is reaching GPIO4.
 * =================================================================== */
#define XBEE_RAW_DUMP_MS   1500
#define XBEE_RAW_DUMP_MAX  256

#if XBEE_RAW_DUMP_MS > 0
static void xbee_dump_raw(struct telemetry_dev *dev, const char *when)
{
    static uint8_t cap[XBEE_RAW_DUMP_MAX];
    size_t   used = 0;
    uint32_t total = 0, printable = 0, sof = 0;

    const int64_t deadline =
        esp_timer_get_time() + (int64_t)XBEE_RAW_DUMP_MS * 1000;

    while (esp_timer_get_time() < deadline) {
        uint8_t buf[64];
        int got = uart_read_bytes(dev->cfg.uart_port, buf, sizeof buf,
                                  pdMS_TO_TICKS(20));
        for (int i = 0; i < got; i++) {
            const uint8_t b = buf[i];
            total++;
            if (b == 0x7E) { sof++; }
            if ((b >= 0x20 && b < 0x7F) || b == '\r' || b == '\n') {
                printable++;
            }
            if (used < sizeof cap) { cap[used++] = b; }
        }
    }

    if (total == 0) {
        ESP_LOGW(TAG, "raw dump [%s]: SILENT -- nothing on DOUT in %d ms",
                 when, XBEE_RAW_DUMP_MS);
        return;
    }

    ESP_LOGW(TAG, "raw dump [%s]: %u bytes, %u start-delimiters (0x7E), "
                  "%u%% printable",
             when, (unsigned)total, (unsigned)sof,
             (unsigned)(printable * 100u / total));

    for (size_t off = 0; off < used; off += 16) {
        char   hex[16 * 3 + 1];
        char   asc[17];
        size_t n = (used - off < 16) ? (used - off) : 16;
        for (size_t k = 0; k < n; k++) {
            const uint8_t b = cap[off + k];
            snprintf(&hex[k * 3], 4, "%02X ", b);
            asc[k] = (b >= 0x20 && b < 0x7F) ? (char)b : '.';
        }
        hex[n * 3] = '\0';
        asc[n]     = '\0';
        ESP_LOGW(TAG, "  %04u  %-48s |%s|", (unsigned)off, hex, asc);
    }

    if (cap[0] == 0x7E) {
        ESP_LOGW(TAG, "  verdict: API frames -- module is in API mode and "
                      "talking. Suspect what we SEND on DIN, not the module.");
    } else if (printable * 100u / total > 80u) {
        ESP_LOGW(TAG, "  verdict: ASCII text -- bootloader banner or AT mode. "
                      "The module is NOT in API mode right now.");
    } else {
        ESP_LOGW(TAG, "  verdict: no recognisable structure -- baud mismatch, "
                      "or something else is driving this line.");
    }
}
#endif /* XBEE_RAW_DUMP_MS */

/* ===================================================================
 *  TEMPORARY -- RX activity test. Set XBEE_RX_ACTIVITY_MS to 0 to skip.
 *
 *  The DOUT path test proves SOMETHING drives GPIO4. This one asks the
 *  next question: when the module is known to be answering, does that
 *  wire move, and does the UART turn the movement into bytes?
 *
 *  Method: with the UART already attached, send one ATAP query (read-
 *  only, always answered with a 0x88), then watch GPIO4 directly with
 *  gpio_get_level() -- which reads the pad, not the UART -- counting
 *  falling edges and the shortest low pulse. Then drain the UART and
 *  compare what it caught with what we sent. The pin watch and the UART
 *  see the same wire over the same window, so where they disagree is
 *  the answer.
 *
 *    line never moved     GPIO4 idles high and nothing ever arrives on
 *                         it. The module answers on its own DOUT (XCTU
 *                         shows that), so GPIO4 is not on that net.
 *    our own frame back   GPIO4 is on the DIN net -- it is hearing the
 *                         ESP's own transmission on GPIO3, not the
 *                         module. Wrong explorer pin.
 *    edges but no bytes   the module IS talking and the UART is NOT
 *                         hearing it: pin routing or baud on our side.
 *    a 0x88 for AP        receive path works.
 *
 *  The shortest low pulse is one bit time, so it estimates the sender's
 *  baud: ~8.7 us at 115200, ~104 us at 9600.
 * =================================================================== */
#define XBEE_RX_ACTIVITY_MS  300   /* watch window; 0 disables */
#define XBEE_RX_ACTIVITY_MAX 64    /* bytes kept for the hex dump */

#if XBEE_RX_ACTIVITY_MS > 0 && !XBEE_TRANSPARENT
static void xbee_rx_activity_test(struct telemetry_dev *dev)
{
    const int pin = dev->cfg.rx_pin;
    uint8_t sent[XBEE_MAX_FRAME_DATA];

    uart_flush_input(dev->cfg.uart_port);
    xbee_parser_reset(&dev->parser);

    const uint8_t want_id = next_frame_id(dev);
    const size_t sent_len = xbee_build_at_query(want_id, 'A', 'P', sent, sizeof sent);
    if (sent_len == 0 ||
        uart_write_bytes(dev->cfg.uart_port, (const char *)sent, sent_len) != (int)sent_len) {
        ESP_LOGW(TAG, "RX activity test: could not send ATAP -- skipping");
        return;
    }

    /* Watch the pad while the UART driver clocks the query out of its
     * TX buffer and the module answers. Busy-polls on purpose: one bit
     * at 115200 is 8.7 us, far shorter than a tick. */
    uint32_t samples = 0, lows = 0, edges = 0;
    int64_t  min_low_us = -1;
    int64_t  low_start  = 0;
    int64_t  now        = esp_timer_get_time();
    const int64_t deadline = now + (int64_t)XBEE_RX_ACTIVITY_MS * 1000;
    int prev = gpio_get_level(pin);

    while ((now = esp_timer_get_time()) < deadline) {
        const int lvl = gpio_get_level(pin);
        samples++;
        if (lvl == 0) { lows++; }
        if (prev == 1 && lvl == 0) {
            edges++;
            low_start = now;
        } else if (prev == 0 && lvl == 1 && edges > 0) {
            const int64_t w = now - low_start;
            if (min_low_us < 0 || w < min_low_us) { min_low_us = w; }
        }
        prev = lvl;
    }

    /* Everything the UART caught over that same window. */
    static uint8_t got[XBEE_RX_ACTIVITY_MAX];
    size_t kept = 0;
    uint32_t total = 0;
    bool ap_ok = false, echo = false;
    uint8_t ap_val = 0;
    for (;;) {
        uint8_t buf[64];
        const int n = uart_read_bytes(dev->cfg.uart_port, buf, sizeof buf, pdMS_TO_TICKS(20));
        if (n <= 0) { break; }
        for (int i = 0; i < n; i++) {
            total++;
            if (kept < sizeof got) { got[kept++] = buf[i]; }
            if (!xbee_parser_feed(&dev->parser, buf[i])) { continue; }

            const uint8_t *f = dev->parser.frame;
            const uint16_t flen = dev->parser.frame_len;
            uint8_t id = 0, status = 0;
            const uint8_t *val = NULL;
            uint16_t val_len = 0;
            char c0 = 0, c1 = 0;

            if (f[0] == XBEE_FRAME_AT_COMMAND && flen >= 4 &&
                f[1] == want_id && f[2] == 'A' && f[3] == 'P') {
                echo = true;            /* our own query came back */
            } else if (f[0] == XBEE_FRAME_AT_RESPONSE &&
                       xbee_parse_at_response(f, flen, &id, &c0, &c1, &status, &val, &val_len) &&
                       id == want_id && c0 == 'A' && c1 == 'P' && status == 0 && val_len >= 1) {
                ap_ok  = true;
                ap_val = val[0];
            }
        }
    }
    xbee_parser_reset(&dev->parser);

    ESP_LOGW(TAG, "=== RX activity test on GPIO%d (sent ATAP, watched %d ms) ===",
             pin, XBEE_RX_ACTIVITY_MS);
    ESP_LOGW(TAG, "  pin: %u samples, %u low, %u falling edges, shortest low %d us%s",
             (unsigned)samples, (unsigned)lows, (unsigned)edges,
             (int)min_low_us, min_low_us > 0 ? "" : " (none)");
    if (min_low_us > 0) {
        ESP_LOGW(TAG, "  pin: shortest low implies roughly %u baud (115200 -> ~9 us)",
                 (unsigned)(1000000 / min_low_us));
    }
    ESP_LOGW(TAG, "  uart: %u bytes", (unsigned)total);
    for (size_t off = 0; off < kept; off += 16) {
        char hex[16 * 3 + 1];
        const size_t n = (kept - off < 16) ? (kept - off) : 16;
        for (size_t k = 0; k < n; k++) {
            snprintf(&hex[k * 3], 4, "%02X ", got[off + k]);
        }
        hex[n * 3] = '\0';
        ESP_LOGW(TAG, "  %04u  %s", (unsigned)off, hex);
    }

    if (samples > 0 && lows == samples) {
        ESP_LOGW(TAG, "  verdict: line held LOW the whole window -- short to "
                      "ground or a continuous break. Not an idle UART.");
    } else if (ap_ok) {
        ESP_LOGW(TAG, "  verdict: RECEIVE PATH WORKS -- module answered ATAP = %u. "
                      "If ATVR still fails, the fault is in the probe, not the wire.",
                 (unsigned)ap_val);
    } else if (echo) {
        ESP_LOGW(TAG, "  verdict: OWN FRAME ECHOED -- GPIO%d is hearing GPIO%d's "
                      "transmission, not the module. The wire is on the DIN net. "
                      "Move it to the XBee's DOUT (pin 2).",
                 pin, dev->cfg.tx_pin);
    } else if (edges == 0) {
        ESP_LOGW(TAG, "  verdict: LINE NEVER MOVED -- GPIO%d sat high while the "
                      "module was answering. Whatever is on GPIO%d idles high but "
                      "is not the module's DOUT. Trace the wire to XBee pin 2.",
                 pin, pin);
    } else if (total == 0) {
        ESP_LOGW(TAG, "  verdict: EDGES BUT NO BYTES -- the line toggles and the "
                      "UART hears none of it. Our side: UART RX routing or baud.");
    } else {
        ESP_LOGW(TAG, "  verdict: bytes arrived but no ATAP answer -- check the "
                      "baud estimate above against %d.", dev->cfg.baud_rate);
    }
}
#endif /* XBEE_RX_ACTIVITY_MS */

/* Reset, verify, and only escape if verification failed. */
#if !XBEE_TRANSPARENT || XBEE_TRANSPARENT_VERIFY
/* ===================================================================
 *  Command mode (+++), the transparent-mode way to talk to the module.
 *
 *  Used at boot only: by a TRANSPARENT build to verify the radio, and by
 *  an API build whose probe went unanswered, to find out whether the
 *  module is simply in the other mode. Never while telemetry is running
 *  -- entering command mode needs GT of silence on DIN either side of
 *  the +++, which would mean stopping the downlink.
 *
 *  Replies are CR-terminated ASCII lines. Lines that are not the answer
 *  we want (uplink bytes from a ground station that is already talking,
 *  say) are skipped rather than trusted.
 * =================================================================== */
static bool xbee_cmd_read_line(struct telemetry_dev *dev, char *line, size_t cap,
                               int timeout_ms)
{
    size_t n = 0;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        uint8_t c;
        if (uart_read_bytes(dev->cfg.uart_port, &c, 1, pdMS_TO_TICKS(10)) != 1) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (n == 0) { continue; }
            line[n] = '\0';
            return true;
        }
        if (n + 1 < cap) { line[n++] = (char)c; }
    }
    return false;
}

static bool xbee_cmd_is_hex(const char *t)
{
    size_t n = strlen(t);
    if (n == 0 || n > 16) { return false; }
    for (size_t i = 0; i < n; i++) {
        char c = t[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) {
            return false;
        }
    }
    return true;
}

/* Waits until a line equal to `want` (or, with want == NULL, a hex
 * number) arrives. "ERROR" ends the wait early. */
static bool xbee_cmd_expect(struct telemetry_dev *dev, const char *want,
                            char *out, size_t cap, int timeout_ms)
{
    char line[40];
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        int left = (int)((deadline - esp_timer_get_time()) / 1000);
        if (left <= 0 || !xbee_cmd_read_line(dev, line, sizeof line, left)) {
            return false;
        }
        if (strcmp(line, "ERROR") == 0) { return false; }
        if (want ? strcmp(line, want) == 0 : xbee_cmd_is_hex(line)) {
            if (out && cap) { strncpy(out, line, cap - 1); out[cap - 1] = '\0'; }
            return true;
        }
    }
    return false;
}

static bool xbee_cmd_mode_enter(struct telemetry_dev *dev)
{
    vTaskDelay(pdMS_TO_TICKS(XBEE_GUARD_MS));           /* silence before */
    uart_flush_input(dev->cfg.uart_port);
    if (uart_write_bytes(dev->cfg.uart_port, "+++", 3) != 3) { return false; }
    /* The module answers OK only after GT of silence AFTER the +++. */
    return xbee_cmd_expect(dev, "OK", NULL, 0, XBEE_GUARD_MS + 1000);
}

static bool xbee_cmd_query(struct telemetry_dev *dev, const char *at,
                           char *out, size_t cap)
{
    char cmd[8];
    int n = snprintf(cmd, sizeof cmd, "%s\r", at);
    if (n <= 0 || uart_write_bytes(dev->cfg.uart_port, cmd, (size_t)n) != n) {
        return false;
    }
    return xbee_cmd_expect(dev, NULL, out, cap, 500);
}

static void xbee_cmd_mode_exit(struct telemetry_dev *dev)
{
    uart_write_bytes(dev->cfg.uart_port, "ATCN\r", 5);
    if (!xbee_cmd_expect(dev, "OK", NULL, 0, 500)) {
        /* Not fatal: the module drops out of command mode on its own
         * after CT (10 s by default). Anything sent before then is
         * taken as AT commands, not data -- hence the warning. */
        ESP_LOGW(TAG, "ATCN not acknowledged -- module may stay in command "
                      "mode for up to CT (10 s by default)");
    }
}

/* +++, ATVR (and optionally ATDH/ATDL), ATCN. True if ATVR answered. */
static bool xbee_transparent_probe(struct telemetry_dev *dev, uint32_t *version,
                                   uint64_t *dest)
{
    if (!xbee_cmd_mode_enter(dev)) {
        return false;
    }
    char v[20];
    bool ok = xbee_cmd_query(dev, "ATVR", v, sizeof v);
    if (ok && version) {
        *version = (uint32_t)strtoul(v, NULL, 16);
    }
    char hi[20], lo[20];
    if (ok && dest &&
        xbee_cmd_query(dev, "ATDH", hi, sizeof hi) &&
        xbee_cmd_query(dev, "ATDL", lo, sizeof lo)) {
        *dest = ((uint64_t)strtoul(hi, NULL, 16) << 32) | (uint64_t)strtoul(lo, NULL, 16);
    }
    xbee_cmd_mode_exit(dev);
    return ok;
}
#endif /* command mode helpers */

#if XBEE_TRANSPARENT
static void xbee_bringup_transparent(struct telemetry_dev *dev)
{
#if XBEE_RAW_DUMP_MS > 0
    xbee_dump_raw(dev, "power-on, we are silent");
#endif
    uart_flush_input(dev->cfg.uart_port);
    xbee_hard_reset(dev);
#if XBEE_RAW_DUMP_MS > 0
    xbee_dump_raw(dev, "after hard reset");
#endif

    if (dev->cfg.dest_addr) {
        ESP_LOGW(TAG, "transparent mode: dest_addr %016llX is IGNORED -- the module's "
                      "own DH/DL decide where packets go",
                 (unsigned long long)dev->cfg.dest_addr);
    }

#if !XBEE_TRANSPARENT_VERIFY
    dev->diag.radio_verified = false;
    ESP_LOGW(TAG, "transparent mode, boot check disabled in menuconfig -- "
                  "sending blind (radio reported as unverified)");
#else
    uint32_t version = 0;
    uint64_t dest = 0;

    bool ok = xbee_transparent_probe(dev, &version, &dest);
    if (!ok) {
        /* Same reasoning as the API path: a module stuck in its
         * bootloader answers nothing until it is told to run. */
        static const char escape[] = XBEE_ESCAPE_BYTES;
        ESP_LOGW(TAG, "no answer to +++ -- sending bootloader escape");
        uart_write_bytes(dev->cfg.uart_port, escape, sizeof(escape) - 1);
        dev->diag.bootloader_escapes++;
        vTaskDelay(pdMS_TO_TICKS(XBEE_ESCAPE_SETTLE_MS));
        ok = xbee_transparent_probe(dev, &version, &dest);
        if (ok) {
            ESP_LOGW(TAG, "radio answered AFTER the bootloader escape -- check that "
                          "DIN is not held low across the module's reset");
        }
    }
    if (ok) {
        dev->diag.radio_verified = true;
        ESP_LOGI(TAG, "radio verified (transparent): ATVR = 0x%04X, sending to "
                      "DH:DL = %016llX%s", (unsigned)version,
                 (unsigned long long)dest,
                 dest == 0xFFFFull ? " (broadcast)" : "");
        return;
    }

    /* Still nothing. One more question is worth asking: is the module in
     * API mode? +++ is ignored there, but an API frame is not. */
    uart_flush_input(dev->cfg.uart_port);
    xbee_parser_reset(&dev->parser);
    if (xbee_probe(dev, &version)) {
        dev->diag.radio_verified = false;
        dev->diag.wrong_mode_detected = true;
        ESP_LOGE(TAG, "WRONG XBEE MODE -- this firmware is built for TRANSPARENT, but the "
                      "module answered an API frame (ATVR = 0x%04X): it is set to AP=1. "
                      "Either select API mode in menuconfig (XBee telemetry link) or set "
                      "AP=0 in XCTU. Nothing will get through until they agree.",
                 (unsigned)version);
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, ESP_ERR_INVALID_STATE, 1);
        return;
    }

    dev->diag.radio_verified = false;
    ESP_LOGE(TAG, "RADIO NOT VERIFIED -- no answer to +++ (transparent) or ATVR (API). "
                  "Check: baud %d? powered? GT shorter than %d ms? "
                  "Telemetry will still transmit blindly.",
             dev->cfg.baud_rate, XBEE_GUARD_MS);
    event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, ESP_ERR_TIMEOUT, 0);
#endif /* XBEE_TRANSPARENT_VERIFY */
}
#endif /* XBEE_TRANSPARENT */

static void xbee_bringup(struct telemetry_dev *dev)
{
#if XBEE_TRANSPARENT
    xbee_bringup_transparent(dev);
    return;
#else
    uint32_t version = 0;

#if XBEE_RAW_DUMP_MS > 0
    /* We have not written a single byte yet, so anything here is the
     * module talking entirely on its own. */
    xbee_dump_raw(dev, "power-on, we are silent");
#endif

    /* Discard whatever the module said while the UART was coming up --
     * a bootloader banner, or nothing. Either way it is not a frame and
     * feeding it to the parser wastes its resynchronisation. */
    uart_flush_input(dev->cfg.uart_port);
    xbee_parser_reset(&dev->parser);

    xbee_hard_reset(dev);

#if XBEE_RAW_DUMP_MS > 0
    /* Same again, but after a clean reset with DIN already idling high.
     * If this one is ASCII, the module boots into its bootloader. */
    xbee_dump_raw(dev, "after hard reset");
#endif

#if XBEE_RX_ACTIVITY_MS > 0 && !XBEE_TRANSPARENT
    /* The module has booted; now ask it something and watch GPIO4. */
    xbee_rx_activity_test(dev);
#endif

    uart_flush_input(dev->cfg.uart_port);
    xbee_parser_reset(&dev->parser);

    if (xbee_probe(dev, &version)) {
        dev->diag.radio_verified = true;
        ESP_LOGI(TAG, "radio verified: ATVR = 0x%04X", (unsigned)version);
        return;
    }

    /* No answer. Either the module is in its bootloader, or it is in
     * transparent mode, or it is not there at all. The escape costs one
     * short write and is harmless in API mode -- a byte outside a frame
     * is discarded by the module's own parser -- so it is worth trying
     * before giving up. */
    static const char escape[] = XBEE_ESCAPE_BYTES;
    ESP_LOGW(TAG, "no ATVR response -- sending bootloader escape");
    uart_write_bytes(dev->cfg.uart_port, escape, sizeof(escape) - 1);
    dev->diag.bootloader_escapes++;
    vTaskDelay(pdMS_TO_TICKS(XBEE_ESCAPE_SETTLE_MS));
    uart_flush_input(dev->cfg.uart_port);
    xbee_parser_reset(&dev->parser);

    if (xbee_probe(dev, &version)) {
        dev->diag.radio_verified = true;
        ESP_LOGW(TAG, "radio verified AFTER bootloader escape: ATVR = 0x%04X. "
                      "The module was in its bootloader at power-on -- check that "
                      "DIN is not held low across the module's reset.",
                 (unsigned)version);
        return;
    }

    /* Before giving up: is the module just in TRANSPARENT mode? That is
     * the commonest reason for silence here, and +++ answers it in about
     * two and a half seconds -- spent only on this already-failed path. */
    if (xbee_transparent_probe(dev, &version, NULL)) {
        dev->diag.radio_verified = false;
        dev->diag.wrong_mode_detected = true;
        ESP_LOGE(TAG, "WRONG XBEE MODE -- this firmware is built for API (AP=1), but the "
                      "module answered +++ (ATVR = 0x%04X): it is in TRANSPARENT mode "
                      "(AP=0). Either set AP=1 in XCTU or select Transparent in "
                      "menuconfig (XBee telemetry link). Nothing will get through "
                      "until they agree.", (unsigned)version);
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, ESP_ERR_INVALID_STATE, 1);
        return;
    }

    /* Deliberately not a failure. A vehicle that refuses to boot because
     * its downlink is deaf has traded a degraded flight for no flight,
     * and the SD card is the flight record that actually matters. Fly
     * with the warning; STATUS_FLAG_TELEM_OK will report the truth. */
    dev->diag.radio_verified = false;
    ESP_LOGE(TAG, "RADIO NOT VERIFIED -- no ATVR response before or after the "
                  "escape. Check: module in API mode (AP=1)? baud %d? powered? "
                  "Telemetry will still queue and transmit blindly.",
             dev->cfg.baud_rate);
    event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, ESP_ERR_TIMEOUT, 0);
#endif /* XBEE_TRANSPARENT */
}

/* ===================================================================
 *  TEMPORARY -- DIN contention test. Set XBEE_DIN_TEST_S to 0 to skip.
 *
 *  Answers one question with no extra hardware: when the ESP32 pulls
 *  DIN low, does it actually GET low, or is something else on that net
 *  holding it up?
 *
 *  A voltmeter alone cannot tell an actively-driven line from a weak
 *  pull-up -- both read 3.3 V into a 10 Mohm meter. Driving the line and
 *  measuring the result can, because a pull-up collapses under a real
 *  driver and another push-pull output does not.
 *
 *  Runs before the UART claims GPIO3, and hands the pin back afterwards.
 *
 *  HOW TO USE: probe the EXPLORER's DIN pin (not the board side -- R10
 *  is a 10k pull-up to 3V3 there and will read 3.3 V no matter what),
 *  referenced to board ground. Read the voltage during each window.
 *
 *  What the LOW window means, against the XBee's VIL of 0.2 x VCC
 *  (~0.66 V at 3.3 V):
 *
 *    below 0.3 V   clean. Nothing is fighting. Contention DISPROVEN --
 *                  the fault is elsewhere (baud, AP setting, wiring).
 *    0.3 - 0.66 V  something is pushing back, but the XBee can still
 *                  read a zero. Marginal; fix it anyway.
 *    above 0.66 V  the XBee CANNOT see this as a zero. Contention
 *                  CONFIRMED -- get the module off the explorer.
 *
 *  The HIGH window is a sanity check: ~3.3 V confirms the wire is
 *  intact and you are probing the right pin.
 * =================================================================== */
#define XBEE_DIN_TEST_S  0        /* seconds per window; 0 disables */

#if XBEE_DIN_TEST_S > 0
static void xbee_din_contention_test(int tx_pin)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << tx_pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGW(TAG, "DIN test: could not claim GPIO%d -- skipping", tx_pin);
        return;
    }

    ESP_LOGW(TAG, "=== DIN contention test on GPIO%d ===", tx_pin);
    ESP_LOGW(TAG, "Probe the EXPLORER's DIN pin against board ground.");

    gpio_set_level(tx_pin, 1);
    ESP_LOGW(TAG, ">>> DIN driven HIGH for %d s -- expect ~3.3 V", XBEE_DIN_TEST_S);
    vTaskDelay(pdMS_TO_TICKS(XBEE_DIN_TEST_S * 1000));

    gpio_set_level(tx_pin, 0);
    ESP_LOGW(TAG, ">>> DIN driven LOW for %d s -- THIS IS THE READING THAT MATTERS",
             XBEE_DIN_TEST_S);
    vTaskDelay(pdMS_TO_TICKS(XBEE_DIN_TEST_S * 1000));

    ESP_LOGW(TAG, "=== end DIN test ===");
    ESP_LOGW(TAG, "  <0.30 V  clean -- nothing is fighting, contention disproven");
    ESP_LOGW(TAG, "  <0.66 V  marginal -- XBee can still read a zero");
    ESP_LOGW(TAG, "  >0.66 V  CONFIRMED -- XBee cannot read a zero here");

    /* Hand the pin back high-impedance so uart_set_pin() starts clean.
     * R10 holds DIN at the idle level in the gap. */
    const gpio_config_t release = {
        .pin_bit_mask = 1ULL << tx_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&release);
}
#endif /* XBEE_DIN_TEST_S */

/* ===================================================================
 *  TEMPORARY -- DOUT path test. Set XBEE_DOUT_TEST_MS to 0 to skip.
 *
 *  The mirror of the DIN test, and it answers the question that
 *  measuring at the explorer's DOUT pin CANNOT: do the bytes the module
 *  emits actually arrive at GPIO4? A meter on the module's pin proves
 *  the module talks. It says nothing about the wire in between.
 *
 *  Method: park GPIO4 as an input with the internal pull-down on
 *  (~45 kohm) and sample hard. The XBee's DOUT driver is tens of ohms, so
 *  if it is connected it wins that contest every time. Then repeat with
 *  the internal pull-up. A pin with nothing attached follows whichever
 *  internal resistor is enabled, and that is the signature we are after.
 *
 *    pulldown sees HIGH   something external drives the line high.
 *                         DOUT is connected. Path is GOOD.
 *    neither sees anything the pin exactly follows the internal
 *                         resistors -- NOTHING IS ATTACHED. Broken wire,
 *                         wrong pin, or a dead GPIO4.
 *    pullup sees LOW only something holds it low: a short to ground, or
 *                         the module asserting a continuous break.
 *
 *  Runs before the UART claims GPIO4 and hands the pin back afterwards.
 * =================================================================== */
#define XBEE_DOUT_TEST_MS  200     /* per phase; 0 disables */

#if XBEE_DOUT_TEST_MS > 0
static uint32_t xbee_pin_watch(int pin, int want, int ms)
{
    uint32_t hits = 0, total = 0;
    const int64_t deadline = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < deadline) {
        total++;
        if (gpio_get_level(pin) == want) { hits++; }
    }
    (void)total;
    return hits;
}

static void xbee_dout_path_test(int rx_pin)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << rx_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGW(TAG, "DOUT test: could not claim GPIO%d -- skipping", rx_pin);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    const uint32_t high_vs_pulldown = xbee_pin_watch(rx_pin, 1, XBEE_DOUT_TEST_MS);

    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    (void)gpio_config(&io);
    vTaskDelay(pdMS_TO_TICKS(5));
    const uint32_t low_vs_pullup = xbee_pin_watch(rx_pin, 0, XBEE_DOUT_TEST_MS);

    ESP_LOGW(TAG, "=== DOUT path test on GPIO%d ===", rx_pin);
    ESP_LOGW(TAG, "  highs seen against internal pull-down: %u",
             (unsigned)high_vs_pulldown);
    ESP_LOGW(TAG, "  lows  seen against internal pull-up  : %u",
             (unsigned)low_vs_pullup);

    if (high_vs_pulldown > 0) {
        ESP_LOGW(TAG, "  verdict: DOUT IS CONNECTED -- something external "
                      "drives GPIO%d high against the pull-down. The wire is "
                      "good; if the UART still reads nothing, suspect baud.",
                 rx_pin);
    } else if (low_vs_pullup > 0) {
        ESP_LOGW(TAG, "  verdict: line held LOW -- short to ground, or the "
                      "module is asserting a continuous break. Not a normal "
                      "idle UART, which sits HIGH.");
    } else {
        ESP_LOGW(TAG, "  verdict: NOTHING ATTACHED -- GPIO%d follows the "
                      "internal resistors exactly. Broken wire, wrong pin at "
                      "the explorer, or a dead GPIO. This is why every raw "
                      "dump reads SILENT.", rx_pin);
    }

    io.pull_up_en = GPIO_PULLUP_DISABLE;
    (void)gpio_config(&io);
}
#endif /* XBEE_DOUT_TEST_MS */

esp_err_t telemetry_init(const telemetry_config_t *cfg, telemetry_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }

#if XBEE_DIN_TEST_S > 0
    /* Before the UART claims the pin. */
    xbee_din_contention_test(cfg->tx_pin);
#endif

#if XBEE_DOUT_TEST_MS > 0
    xbee_dout_path_test(cfg->rx_pin);
#endif

    struct telemetry_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < TELEMETRY_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", TELEMETRY_MAX_INSTANCES);
        return ESP_ERR_NO_MEM;
    }

    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    dev->diag_lock = tmp;
    dev->cfg = *cfg;
    if (dev->cfg.baud_rate == 0)      { dev->cfg.baud_rate = TELEMETRY_DEFAULT_BAUD; }
    if (dev->cfg.imu_decimation == 0) { dev->cfg.imu_decimation = TELEMETRY_IMU_DECIMATION; }
    dev->next_frame_id = 0;
    xbee_parser_reset(&dev->parser);
#if XBEE_TRANSPARENT
    uplink_stream_reset(&dev->ustream);
#endif
    dev->diag.transparent = XBEE_TRANSPARENT;

    uart_config_t uart_cfg = {
        .baud_rate  = dev->cfg.baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        /* CTS is wired to GPIO5 but not handed to the driver here. The
         * XBee raises it when its own buffer fills, and ignoring it
         * means bytes are dropped silently under a burst. Turning on
         * hardware flow control is the right fix once the pin is
         * plumbed through this config -- see the README. */
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(dev->cfg.uart_port,
                                        TELEMETRY_UART_BUF_SIZE,
                                        TELEMETRY_UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, err, 0);
        dev->in_use = false;
        return err;
    }

    err = uart_param_config(dev->cfg.uart_port, &uart_cfg);
    if (err == ESP_OK) {
        err = uart_set_pin(dev->cfg.uart_port, dev->cfg.tx_pin, dev->cfg.rx_pin,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart config failed: %s", esp_err_to_name(err));
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, err, 0);
        uart_driver_delete(dev->cfg.uart_port);
        dev->in_use = false;
        return err;
    }

    if (!ring_buffer_init(&dev->rb, dev->rb_storage,
                          sizeof(queue_item_t), TELEMETRY_QUEUE_DEPTH)) {
        ESP_LOGE(TAG, "ring buffer init failed");
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, ESP_FAIL, 0);
        uart_driver_delete(dev->cfg.uart_port);
        dev->in_use = false;
        return ESP_FAIL;
    }

    dev->initialized = true;

    /* Reset the module now that TX is driving DIN high, then prove a
     * radio is actually there. See the block comment in telemetry.h. */
    xbee_bringup(dev);

    *out = dev;

    /* The module must ALREADY be in the mode this build expects (AP=1,
     * or AP=0 for a TRANSPARENT build) at this baud. That
     * is deliberately not done here: switching an XBee into API mode
     * means +++, a guard-time wait, ATAP1, ATWR -- and if any of it
     * half-succeeds you have a radio in an unknown state at an unknown
     * baud. That belongs in bench configuration with XCTU, once, not in
     * a boot path that runs on a pad. If the module is left in
     * transparent mode this driver simply never sees a valid frame:
     * packets_acked stays 0 and last_rssi stays 0. */
    ESP_LOGI(TAG, "telemetry up: UART%d @ %d baud, tx=%d rx=%d, "
                  "block %d B / %d ms, IMU 1-in-%u  (%s mode, module must be AP=%d)",
             dev->cfg.uart_port, dev->cfg.baud_rate,
             dev->cfg.tx_pin, dev->cfg.rx_pin,
             TELEMETRY_BLOCK_BYTES, TELEMETRY_BLOCK_MAX_MS,
             (unsigned)dev->cfg.imu_decimation,
             XBEE_TRANSPARENT ? "TRANSPARENT" : "API", XBEE_TRANSPARENT ? 0 : 1);
    return ESP_OK;
}

esp_err_t telemetry_start(telemetry_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) { return ESP_ERR_INVALID_STATE; }
    if (dev->task) { return ESP_ERR_INVALID_STATE; }

    dev->task_should_run = true;
    dev->task = xTaskCreateStaticPinnedToCore(
        telemetry_task, "telemetry", TELEMETRY_STACK_WORDS,
        (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        dev->task_should_run = false;
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_TASK_FAILED, ESP_FAIL, 0);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t telemetry_stop(telemetry_handle_t dev)
{
    if (!dev) { return ESP_ERR_INVALID_ARG; }
    if (dev->task) {
        dev->task_should_run = false;
        for (int i = 0; i < 100 && dev->task; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (dev->initialized) {
        uart_driver_delete(dev->cfg.uart_port);
    }
    dev->initialized = false;
    dev->in_use      = false;
    return ESP_OK;
}

esp_err_t telemetry_write(telemetry_handle_t dev, sensor_stream_t stream,
                          const void *record, size_t len)
{
    if (!dev || !dev->initialized || !record) {
        return ESP_ERR_INVALID_ARG;
    }
    if (stream >= STREAM_COUNT || len == 0 || len > SENSOR_RECORD_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Decimation lives here, not in the IMU driver: SD must keep every
     * sample, and this is the only consumer that cannot afford them. */
    if (stream == STREAM_IMU) {
        uint8_t decim = flight_mode_is_fast() ? dev->cfg.imu_decimation
                                              : TELEMETRY_STANDBY_IMU_DECIM;
        dev->imu_phase++;
        if (dev->imu_phase < decim) {
            portENTER_CRITICAL(&dev->diag_lock);
            dev->diag.records_dropped_decimated++;
            portEXIT_CRITICAL(&dev->diag_lock);
            return ESP_OK;
        }
        dev->imu_phase = 0;
    }

    queue_item_t item;
    item.stream = (uint8_t)stream;
    item.len    = (uint8_t)len;
    memcpy(item.payload, record, len);

    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.records_queued++;
    portEXIT_CRITICAL(&dev->diag_lock);

    if (ring_buffer_push(&dev->rb, &item)) {
        portENTER_CRITICAL(&dev->diag_lock);
        dev->diag.queue_full_drops++;
        portEXIT_CRITICAL(&dev->diag_lock);
    }
    return ESP_OK;
}

bool telemetry_take_ping_request(telemetry_handle_t dev)
{
    if (!dev || !dev->ping_requested) {
        return false;
    }
    dev->ping_requested = false;
    return true;
}

bool telemetry_take_sd_eject_request(telemetry_handle_t dev)
{
    if (!dev || !dev->sd_eject_requested) {
        return false;
    }
    dev->sd_eject_requested = false;
    return true;
}

bool telemetry_take_config_apply_request(telemetry_handle_t dev)
{
    if (!dev || !dev->config_apply_requested) {
        return false;
    }
    dev->config_apply_requested = false;
    return true;
}

void telemetry_set_uplink_hook(telemetry_handle_t dev, telemetry_uplink_hook_t hook)
{
    if (dev) {
        dev->uplink_hook = hook;
    }
}

void telemetry_get_diagnostics(telemetry_handle_t dev, telemetry_diag_t *out)
{
    if (!dev || !out) { return; }
    portENTER_CRITICAL(&dev->diag_lock);
    *out = dev->diag;
    portEXIT_CRITICAL(&dev->diag_lock);
}
