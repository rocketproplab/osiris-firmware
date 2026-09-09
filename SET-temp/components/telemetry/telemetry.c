#include "telemetry.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "ring_buffer.h"
#include "event_log.h"
#include "xbee_api.h"
#include "flight_mode.h"

static const char *TAG = "TELEMETRY";

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

    /* Uplink command state. last_cmd_seq starts at 0 and no command may
     * use seq 0, so the very first real command is always newer. */
    uint16_t             last_cmd_seq;
    bool                 have_cmd_seq;
    volatile bool        ping_requested;

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
static uint8_t next_frame_id(struct telemetry_dev *dev)
{
    dev->next_frame_id++;
    if (dev->next_frame_id == 0) {
        dev->next_frame_id = 1;
    }
    return dev->next_frame_id;
}

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
    uint8_t frame[TELEMETRY_MAX_FRAME];
    uint64_t dest = dev->cfg.dest_addr ? dev->cfg.dest_addr : XBEE_ADDR_BROADCAST;
    size_t n = xbee_build_tx_request(frame_id, dest, packet, packet_len,
                                     frame, sizeof(frame));
    if (n == 0) {
        return false;
    }

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
}

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
            /* 0x90: type, src64[8], reserved[2], options, then payload. */
            if (flen > 12) {
                telemetry_handle_command(dev, &f[12], (uint16_t)(flen - 12));
            }
            break;
        default: break;
        }
    }
    portENTER_CRITICAL(&dev->diag_lock);
    dev->diag.bad_checksums = dev->parser.bad_checksums;
    portEXIT_CRITICAL(&dev->diag_lock);
}

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

static void telemetry_task(void *arg)
{
    struct telemetry_dev *dev = (struct telemetry_dev *)arg;
    queue_item_t item;

    ESP_LOGI(TAG, "telemetry task running on core %d (XBee API mode)",
             (int)xPortGetCoreID());

    dev->block_started_us = (uint64_t)esp_timer_get_time();

    while (dev->task_should_run) {
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

        if ((now - dev->last_rssi_poll_us) >= (uint64_t)TELEMETRY_RSSI_POLL_MS * 1000ULL) {
            uint8_t f[16];
            size_t n = xbee_build_at_query(next_frame_id(dev), 'D', 'B', f, sizeof(f));
            if (n) {
                uart_write_bytes(dev->cfg.uart_port, (const char *)f, n);
            }
            dev->last_rssi_poll_us = now;
        }

        if (!got_one) {
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

/* Reset, verify, and only escape if verification failed. */
static void xbee_bringup(struct telemetry_dev *dev)
{
    uint32_t version = 0;

    /* Discard whatever the module said while the UART was coming up --
     * a bootloader banner, or nothing. Either way it is not a frame and
     * feeding it to the parser wastes its resynchronisation. */
    uart_flush_input(dev->cfg.uart_port);
    xbee_parser_reset(&dev->parser);

    xbee_hard_reset(dev);
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
}

esp_err_t telemetry_init(const telemetry_config_t *cfg, telemetry_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }

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

    /* The module must ALREADY be in API mode (AP=1) at this baud. That
     * is deliberately not done here: switching an XBee into API mode
     * means +++, a guard-time wait, ATAP1, ATWR -- and if any of it
     * half-succeeds you have a radio in an unknown state at an unknown
     * baud. That belongs in bench configuration with XCTU, once, not in
     * a boot path that runs on a pad. If the module is left in
     * transparent mode this driver simply never sees a valid frame:
     * packets_acked stays 0 and last_rssi stays 0. */
    ESP_LOGI(TAG, "telemetry up: UART%d @ %d baud, tx=%d rx=%d, "
                  "block %d B / %d ms, IMU 1-in-%u  (module must be AP=1)",
             dev->cfg.uart_port, dev->cfg.baud_rate,
             dev->cfg.tx_pin, dev->cfg.rx_pin,
             TELEMETRY_BLOCK_BYTES, TELEMETRY_BLOCK_MAX_MS,
             (unsigned)dev->cfg.imu_decimation);
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

void telemetry_get_diagnostics(telemetry_handle_t dev, telemetry_diag_t *out)
{
    if (!dev || !out) { return; }
    portENTER_CRITICAL(&dev->diag_lock);
    *out = dev->diag;
    portEXIT_CRITICAL(&dev->diag_lock);
}
