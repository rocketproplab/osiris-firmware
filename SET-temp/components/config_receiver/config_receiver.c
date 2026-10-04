#include "config_receiver.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "event_log.h"
#include "sensor_records.h"

static const char *TAG = "CFG_RX";

/* The namespace and key deploy_logic_load_config() reads. They are
 * spelled out again here rather than shared through a header on
 * purpose: flight_logic is the reader and this is the writer, and a
 * component that could only be built alongside its counterpart would be
 * harder to test than one that agrees with it on two string literals.
 * The host test checks the two match. */
#define CONFIG_NVS_NAMESPACE   "flight_cfg"
#define CONFIG_NVS_KEY         "config_json"

static struct {
    uint8_t  buf[CONFIG_RX_MAX_BYTES];
    bool     have[CONFIG_RX_MAX_CHUNKS];
    bool     in_progress;
    uint16_t cfg_crc;
    uint16_t chunks_received;
    uint16_t chunks_expected;
    uint16_t total_len;

    uint16_t last_seq;
    bool     have_seq;

    uint32_t stored;
    uint32_t rejected;
    uint32_t dropped;
} s;

/* Byte-for-byte the same function as telemetry.c's and the Python
 * tool's. Duplicated rather than shared because this component does not
 * otherwise depend on telemetry, and the cross-check test pins all
 * three to the same vectors. */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool seq_is_newer(uint16_t seq, uint16_t last)
{
    return (uint16_t)(seq - last) != 0 && (uint16_t)(seq - last) < 0x8000;
}

static void transfer_reset(uint16_t cfg_crc, uint16_t chunks, uint16_t total_len)
{
    memset(s.have, 0, sizeof(s.have));
    s.in_progress      = true;
    s.cfg_crc          = cfg_crc;
    s.chunks_received  = 0;
    s.chunks_expected  = chunks;
    s.total_len        = total_len;
}

static void reject(uint8_t reason, uint32_t detail)
{
    s.rejected++;
    s.in_progress = false;
    event_log_event(EV_CONFIG_REJECTED, reason, detail);
    ESP_LOGW(TAG, "config upload rejected: reason %u detail %lu",
             (unsigned)reason, (unsigned long)detail);
}

static void commit(void)
{
    if (!s.in_progress) {
        reject(CFG_REJECT_INCOMPLETE, 0);
        return;
    }

    /* Report the FIRST missing chunk, not just the count. The ground
     * resends everything either way, but "chunk 7 never arrived" and
     * "nothing after chunk 7 arrived" look identical in a count and
     * point at different problems -- a dropout versus the link dying. */
    for (uint16_t i = 0; i < s.chunks_expected; i++) {
        if (!s.have[i]) {
            reject(CFG_REJECT_INCOMPLETE, i);
            return;
        }
    }

    const uint16_t got = crc16_ccitt(s.buf, s.total_len);
    if (got != s.cfg_crc) {
        reject(CFG_REJECT_CRC, got);
        return;
    }

    /* NUL-terminated because nvs_set_str wants a C string and because
     * deploy_logic_load_config() hands the result to cJSON_Parse. There
     * is always room: total_len was bounds-checked against
     * CONFIG_RX_MAX_BYTES - 1 when the transfer started. */
    s.buf[s.total_len] = '\0';

    nvs_handle_t h;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_str(h, CONFIG_NVS_KEY, (const char *)s.buf);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        reject(CFG_REJECT_NVS, (uint32_t)err);
        return;
    }

    s.stored++;
    s.in_progress = false;
    event_log_event(EV_CONFIG_STORED, 0, s.total_len);
    ESP_LOGW(TAG, "config stored: %u bytes, crc 0x%04X. It is in NVS now; "
                  "send CONFIG_APPLY or reboot to make it live.",
             (unsigned)s.total_len, (unsigned)s.cfg_crc);
}

void config_receiver_init(void)
{
    memset(&s, 0, sizeof(s));
}

bool config_receiver_handle_packet(const uint8_t *p, uint16_t len)
{
    if (len < CONFIG_RX_HDR_SIZE + 2 || p[0] != CONFIG_RX_MAGIC) {
        return false;          /* not ours -- let the command parser try */
    }
    /* From here on the packet IS ours, and every path returns true:
     * a malformed config packet must not fall through and get a second
     * reading as a command. */
    if ((p[1] >> 4) != CONFIG_RX_VERSION) {
        s.dropped++;
        ESP_LOGW(TAG, "config packet version %u, expected %u",
                 (unsigned)(p[1] >> 4), CONFIG_RX_VERSION);
        return true;
    }

    const uint8_t  sub         = p[1] & 0x0F;
    const uint16_t seq         = (uint16_t)(p[2] | ((uint16_t)p[3] << 8));
    const uint16_t cfg_crc     = (uint16_t)(p[4] | ((uint16_t)p[5] << 8));
    const uint16_t chunk_idx   = (uint16_t)(p[6] | ((uint16_t)p[7] << 8));
    const uint16_t chunk_count = (uint16_t)(p[8] | ((uint16_t)p[9] << 8));
    const uint16_t total_len   = (uint16_t)(p[10] | ((uint16_t)p[11] << 8));
    const uint8_t  payload_len = p[12];

    if ((uint16_t)(CONFIG_RX_HDR_SIZE + payload_len + 2) != len) {
        s.dropped++;
        return true;
    }
    const uint16_t crc_rx = (uint16_t)(p[CONFIG_RX_HDR_SIZE + payload_len] |
                    ((uint16_t)p[CONFIG_RX_HDR_SIZE + payload_len + 1] << 8));
    if (crc16_ccitt(p, (size_t)(CONFIG_RX_HDR_SIZE + payload_len)) != crc_rx) {
        s.dropped++;
        return true;
    }

    if (s.have_seq && !seq_is_newer(seq, s.last_seq)) {
        return true;           /* the XBee's own retransmit, or a replay */
    }
    s.last_seq = seq;
    s.have_seq = true;

    if (sub == CONFIG_RX_SUB_COMMIT) {
        commit();
        return true;
    }
    if (sub != CONFIG_RX_SUB_CHUNK) {
        s.dropped++;
        return true;
    }

    /* total_len needs room for the terminator commit() writes. */
    if (total_len == 0 || total_len >= CONFIG_RX_MAX_BYTES ||
        chunk_count == 0 || chunk_count > CONFIG_RX_MAX_CHUNKS ||
        chunk_idx >= chunk_count) {
        reject(CFG_REJECT_TOO_LARGE, total_len);
        return true;
    }

    /* A different cfg_crc is a different config: start over rather than
     * mixing two attempts into one buffer. */
    if (!s.in_progress || s.cfg_crc != cfg_crc ||
        s.chunks_expected != chunk_count || s.total_len != total_len) {
        transfer_reset(cfg_crc, chunk_count, total_len);
    }

    const size_t off = (size_t)chunk_idx * CONFIG_RX_CHUNK_BYTES;
    if (off + payload_len > (size_t)total_len) {
        /* The chunk claims bytes past the end of the config it belongs
         * to. Not survivable as a partial write -- drop the transfer
         * rather than commit something assembled from it. */
        reject(CFG_REJECT_TOO_LARGE, (uint32_t)(off + payload_len));
        return true;
    }

    memcpy(s.buf + off, p + CONFIG_RX_HDR_SIZE, payload_len);
    if (!s.have[chunk_idx]) {
        s.have[chunk_idx] = true;
        s.chunks_received++;
    }
    event_log_event(EV_CONFIG_RX, (uint8_t)(s.chunks_received & 0xFF), chunk_count);
    return true;
}

void config_receiver_get_status(config_receiver_status_t *out)
{
    if (!out) {
        return;
    }
    out->in_progress        = s.in_progress;
    out->cfg_crc            = s.cfg_crc;
    out->chunks_received    = s.chunks_received;
    out->chunks_expected    = s.chunks_expected;
    out->total_len          = s.total_len;
    out->transfers_stored   = s.stored;
    out->transfers_rejected = s.rejected;
    out->chunks_dropped     = s.dropped;
}
