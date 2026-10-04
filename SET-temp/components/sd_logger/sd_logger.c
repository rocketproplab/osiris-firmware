#include "sd_logger.h"

#include <string.h>
#include <stdio.h>

#include "ring_buffer.h"
#include "event_log.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SD_LOG";

/* ===================================================================
 *  SDMMC 4-bit slot pins -- fixed, not configurable.
 *
 *  The card plugs directly into the ESP32-P4's SDMMC peripheral; there
 *  is no wiring choice to make the way there is for the sensors' SPI
 *  CS/INT pins, so these live here as private constants rather than in
 *  sd_logger_config_t. Values match the validated working pinout.
 * =================================================================== */

#define SD_PIN_CLK   GPIO_NUM_43
#define SD_PIN_CMD   GPIO_NUM_44
#define SD_PIN_D0    GPIO_NUM_39
#define SD_PIN_D1    GPIO_NUM_40
#define SD_PIN_D2    GPIO_NUM_41
#define SD_PIN_D3    GPIO_NUM_42

/* Largest record across all streams, used to size the queue item. */
/* Largest record across all streams -- see sensor_records.h. Kept as a
 * local alias so existing code below doesn't need renaming. */
#define SD_MAX_RECORD_SIZE   SENSOR_RECORD_MAX_SIZE

/* ===================================================================
 *  Queue item -- one record plus which stream it belongs to.
 *
 *  Fixed-size so the queue can be statically allocated. The payload is
 *  oversized for the smaller streams; that wastes a few bytes of RAM
 *  per slot but avoids either dynamic allocation or three separate
 *  queues, both of which cost more than the padding does.
 * =================================================================== */

/* This used to be a private struct here called sd_queue_item_t; it's
 * now queue_item_t, shared with telemetry.c via sensor_records.h,
 * since the two were byte-for-byte identical. */

/* ===================================================================
 *  Per-stream state
 * =================================================================== */

typedef struct {
    FILE    *fp;
    uint8_t  buf[SD_LOGGER_BUF_SIZE];
    size_t   buf_used;
    uint8_t  record_size;
    const char *filename;
} sd_stream_state_t;

/* ===================================================================
 *  The real struct -- private to this file.
 * =================================================================== */

struct sd_logger_dev {
    bool                 in_use;
    sd_logger_config_t   cfg;
    bool                 initialized;
    bool                 mounted;

    sdmmc_card_t        *card;
    sd_pwr_ctrl_handle_t pwr_ctrl;   /* ESP32-P4 SDMMC IO rail -- see sd_mount_card */
    char                 mount_point[32];

    sd_stream_state_t    streams[STREAM_COUNT];

    /* One ring buffer PER STREAM now, not one shared across all three --
     * see the depth macros in sd_logger.h for why. Overwrite-oldest
     * semantics unchanged: a full buffer discards its OWN oldest
     * unwritten record, never a different sensor's. */
    /* ===================================================================
     *  EACH BUFFER IS SIZED TO ITS OWN STREAM'S RECORD.
     *
     *  These used to be arrays of queue_item_t -- a 42-byte envelope
     *  holding a stream tag, a length, and a payload padded to the
     *  largest record in the project. That is the right shape for
     *  telemetry, which owns ONE buffer carrying every stream mixed
     *  together and therefore has to hold the biggest thing it might
     *  ever see and label it. It is the wrong shape here.
     *
     *  sd_logger has seven buffers, one per stream, and the buffer is
     *  chosen by rb_for_stream() BEFORE the push. By the time a record
     *  reaches baro_rb the stream is already known and the length is
     *  implied by it, so both header bytes were re-stating what the
     *  buffer's own identity already said, and the payload was padded
     *  to 40 bytes to hold a record that is 24.
     *
     *  Sizing each buffer to sizeof(its record) recovers 3688 bytes --
     *  28% of what these queues cost -- with no change in depth. The
     *  worst offender was baro: 75 slots x 18 wasted bytes.
     * =================================================================== */
    ring_buffer_t        baro_rb;
    uint8_t              baro_rb_storage[BARO_QUEUE_DEPTH * sizeof(baro_record_t)];
    ring_buffer_t        imu_rb;
    uint8_t              imu_rb_storage[IMU_QUEUE_DEPTH * sizeof(imu_record_t)];
    ring_buffer_t        gps_rb;
    uint8_t              gps_rb_storage[GPS_QUEUE_DEPTH * sizeof(gps_record_t)];
    ring_buffer_t        batt_rb;
    uint8_t              batt_rb_storage[BATT_QUEUE_DEPTH * sizeof(batt_record_t)];
    ring_buffer_t        err_rb;
    uint8_t              err_rb_storage[ERR_QUEUE_DEPTH * sizeof(err_record_t)];
    ring_buffer_t        event_rb;
    uint8_t              event_rb_storage[EVENT_QUEUE_DEPTH * sizeof(event_record_t)];
    ring_buffer_t        status_rb;
    uint8_t              status_rb_storage[STATUS_QUEUE_DEPTH * sizeof(status_record_t)];

    StaticTask_t         tcb;
    StackType_t          stack[SD_LOGGER_STACK_WORDS];
    TaskHandle_t         task;
    volatile bool        task_should_run;

    portMUX_TYPE         diag_lock;
    sd_logger_diag_t     diag;
};

static struct sd_logger_dev s_pool[SD_LOGGER_MAX_INSTANCES];
static portMUX_TYPE         s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* Maps a stream to its own ring buffer -- the one place that knows
 * which stream owns which queue, so push/pop/init don't repeat this
 * three-way branch everywhere. */
static ring_buffer_t *rb_for_stream(struct sd_logger_dev *dev, sensor_stream_t stream)
{
    switch (stream) {
        case STREAM_BARO: return &dev->baro_rb;
        case STREAM_IMU:  return &dev->imu_rb;
        case STREAM_GPS:  return &dev->gps_rb;
        case STREAM_BATT: return &dev->batt_rb;
        case STREAM_ERR:  return &dev->err_rb;
        case STREAM_EVENT: return &dev->event_rb;
        case STREAM_STATUS: return &dev->status_rb;
        default:          return NULL;
    }
}

/* Every ring buffer in one array, indexed by sensor_stream_t. This
 * exists because the drain loop used to name baro/imu/gps by hand, so
 * adding a stream silently left its buffer to fill up and evict itself
 * forever -- which is exactly what happened to batt_rb. Indexing by
 * stream means a new stream is drained the moment rb_for_stream()
 * knows about it. */
static ring_buffer_t *rb_by_index(struct sd_logger_dev *dev, int i)
{
    return rb_for_stream(dev, (sensor_stream_t)i);
}

/* Record sizes come from sensor_record_size() in sensor_records.h --
 * this file used to keep its own table, which was a second thing to
 * update whenever a record changed. Telemetry needs the same mapping,
 * so it now lives once, beside the struct definitions. */

static const char *s_filenames[STREAM_COUNT] = {
    [STREAM_BARO] = "baro.bin",
    [STREAM_IMU]  = "imu.bin",
    [STREAM_GPS]  = "gps.bin",
    [STREAM_BATT] = "batt.bin",
    [STREAM_ERR]  = "err.bin",
    [STREAM_EVENT] = "event.bin",
    [STREAM_STATUS] = "status.bin",
};

/* CRC16-CCITT over one record, appended after the payload on disk so a
 * partially-written or corrupted record can be detected offline rather
 * than silently decoded as plausible garbage. */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ===================================================================
 *  Buffered write helpers -- run only on the writer task
 * =================================================================== */

/* Push one stream's buffer to the card. Returns ESP_OK even for an
 * empty buffer (nothing to do is not a failure). */
static esp_err_t sd_flush_stream(sd_logger_handle_t dev, sd_stream_state_t *st)
{
    if (st->buf_used == 0 || !st->fp) {
        return ESP_OK;
    }

    size_t written = fwrite(st->buf, 1, st->buf_used, st->fp);
    if (written != st->buf_used) {
        ESP_LOGE(TAG, "fwrite short: %u of %u bytes",
                 (unsigned)written, (unsigned)st->buf_used);
        dev->diag.write_errors++;
        event_log_error(ERR_SRC_SD, ERR_CODE_WRITE_FAILED, ESP_FAIL, dev->diag.write_errors);
        st->buf_used = 0;   /* drop it rather than retry forever */
        return ESP_FAIL;
    }

    /* fflush pushes the C library's own buffer into FATFS. Without it,
     * a power cut can lose data that fwrite() already "accepted". */
    if (fflush(st->fp) != 0) {
        ESP_LOGE(TAG, "fflush failed");
        dev->diag.write_errors++;
        event_log_error(ERR_SRC_SD, ERR_CODE_WRITE_FAILED, ESP_FAIL, dev->diag.write_errors);
    }

    st->buf_used = 0;
    dev->diag.flushes++;
    return ESP_OK;
}

/* Append one record to a stream's buffer, flushing first if it wouldn't
 * fit. */
static void sd_append_record(sd_logger_handle_t dev, sd_stream_state_t *st,
                             const uint8_t *record, size_t len)
{
    const size_t stride = len + SD_CRC_SIZE;

    /* Guard against an entry larger than the whole buffer -- can't happen
     * with the current record sizes, but the check costs nothing and
     * turns a would-be buffer overrun into a logged error. */
    if (stride > SD_LOGGER_BUF_SIZE) {
        ESP_LOGE(TAG, "entry (%u B) exceeds buffer (%u B)",
                 (unsigned)stride, (unsigned)SD_LOGGER_BUF_SIZE);
        dev->diag.write_errors++;
        event_log_error(ERR_SRC_SD, ERR_CODE_WRITE_FAILED, ESP_FAIL, dev->diag.write_errors);
        return;
    }

    if (st->buf_used + stride > SD_LOGGER_BUF_SIZE) {
        sd_flush_stream(dev, st);
    }

    memcpy(&st->buf[st->buf_used], record, len);
    st->buf_used += len;

    /* CRC little-endian, matching every other multi-byte field on disk. */
    uint16_t crc = crc16_ccitt(record, len);
    st->buf[st->buf_used++] = (uint8_t)(crc & 0xFF);
    st->buf[st->buf_used++] = (uint8_t)(crc >> 8);

    dev->diag.records_written++;
}

/* ===================================================================
 *  Writer task
 *
 *  This is the whole reason the ring buffer exists. SD cards stall -- a
 *  block erase can take tens of milliseconds with no warning. Doing
 *  fwrite() directly from the sensor tasks would mean a stalled card
 *  blocks the BMP388 reader mid-flight. Instead the sensor side does a
 *  non-blocking ring_buffer_push() and moves on; all the slow,
 *  unpredictable work happens here, on a task that's allowed to stall.
 * =================================================================== */

/* Drains one stream's ring buffer completely. Returns true if anything
 * was popped, so the caller can tell "genuinely idle" from "just
 * finished a batch" across all three buffers in one pass. */
static bool sd_drain_one_rb(sd_logger_handle_t dev, sensor_stream_t stream,
                            ring_buffer_t *rb)
{
    /* Sized to the largest record in the project, so it is big enough
     * for whichever stream this is. ring_buffer_pop() writes exactly
     * this buffer's elem_size bytes into it. */
    uint8_t rec[SENSOR_RECORD_MAX_SIZE];
    const uint8_t len = sensor_record_size(stream);
    bool got_one = false;

    if (stream >= STREAM_COUNT || len == 0) {
        return false;
    }

    while (ring_buffer_pop(rb, rec)) {
        got_one = true;
        /* The stream comes from the caller's loop index rather than from
         * a tag inside the record. It used to travel in-band, which meant
         * the drain was re-deriving something its caller already knew --
         * and paid two bytes per queued record to do it. */
        sd_append_record(dev, &dev->streams[stream], rec, len);
    }
    return got_one;
}

static void sd_logger_task(void *arg)
{
    /* Last drop count reported, so each pass reports only what is new.
     * Reporting from here rather than from sd_logger_write() is what
     * keeps the error path out of the push path -- see the note beside
     * the queue_full_drops increment. */
    uint32_t last_reported_drops = 0;
    sd_logger_handle_t dev = (sd_logger_handle_t)arg;
    TickType_t last_flush = xTaskGetTickCount();
    const TickType_t flush_interval = pdMS_TO_TICKS(SD_LOGGER_FLUSH_MS);

    ESP_LOGI(TAG, "writer task running on core %d", (int)xPortGetCoreID());

    while (dev->task_should_run) {
        /* Drain every stream's buffer each pass -- if any of them
         * had something, don't sleep; only back off once every buffer
         * is genuinely empty. Same trade-off as before regarding the
         * 10ms poll vs. a true blocking wait -- see the note this
         * comment used to carry, unchanged by the three-buffer split. */
        bool got_one = false;
        for (int i = 0; i < STREAM_COUNT; i++) {
            ring_buffer_t *rb = rb_by_index(dev, i);
            if (rb) {
                got_one |= sd_drain_one_rb(dev, (sensor_stream_t)i, rb);
            }
        }

        if (!got_one) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* Report new queue drops from out here, never from the push
         * path. Naturally rate-limited by this loop's own cadence. */
        uint32_t drops = dev->diag.queue_full_drops;
        if (drops != last_reported_drops) {
            event_log_error(ERR_SRC_SD, ERR_CODE_QUEUE_FULL, ESP_OK,
                            drops - last_reported_drops);
            last_reported_drops = drops;
        }

        /* Periodic flush: a 1 Hz GPS stream would otherwise sit in RAM
         * for ~15 minutes before its 512-byte buffer filled. */
        if ((xTaskGetTickCount() - last_flush) >= flush_interval) {
            for (int i = 0; i < STREAM_COUNT; i++) {
                sd_flush_stream(dev, &dev->streams[i]);
            }
            last_flush = xTaskGetTickCount();
        }
    }

    /* Shutdown path: drain whatever is still queued in every buffer,
     * then flush everything. */
    for (int i = 0; i < STREAM_COUNT; i++) {
        ring_buffer_t *rb = rb_by_index(dev, i);
        if (rb) {
            sd_drain_one_rb(dev, (sensor_stream_t)i, rb);
        }
    }
    for (int i = 0; i < STREAM_COUNT; i++) {
        sd_flush_stream(dev, &dev->streams[i]);
    }

    ESP_LOGI(TAG, "writer task exiting");
    vTaskDelete(NULL);
}

/* ===================================================================
 *  Mount + file setup
 * =================================================================== */

static esp_err_t sd_mount_card(sd_logger_handle_t dev)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = (dev->cfg.max_freq_khz > 0)
                      ? dev->cfg.max_freq_khz
                      : SDMMC_FREQ_DEFAULT;

    /* The ESP32-P4 does not power its SDMMC IO pins from a fixed rail --
     * an on-chip LDO has to be brought up explicitly first, or the card
     * never responds and the mount below fails with no obvious cause.
     * This is P4-specific; it has no equivalent on ESP32/S3. */
    sd_pwr_ctrl_ldo_config_t ldo_cfg = {
        .ldo_chan_id = dev->cfg.ldo_chan_id,
    };
    esp_err_t ldo_err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &dev->pwr_ctrl);
    if (ldo_err != ESP_OK) {
        ESP_LOGE(TAG, "SDMMC LDO power-up failed: %s", esp_err_to_name(ldo_err));
        return ldo_err;
    }
    host.pwr_ctrl_handle = dev->pwr_ctrl;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;                     /* 4-bit mode: 4x the data lines */
    slot.clk   = SD_PIN_CLK;
    slot.cmd   = SD_PIN_CMD;
    slot.d0    = SD_PIN_D0;
    slot.d1    = SD_PIN_D1;
    slot.d2    = SD_PIN_D2;
    slot.d3    = SD_PIN_D3;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = dev->cfg.format_if_mount_failed,
        .max_files              = STREAM_COUNT + 2,  /* +2 headroom */
        .allocation_unit_size   = 16 * 1024,
    };

    esp_err_t err = esp_vfs_fat_sdmmc_mount(dev->mount_point, &host, &slot,
                                            &mount_cfg, &dev->card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount failed: %s -- card inserted? pins right?",
                 esp_err_to_name(err));
        return err;
    }

    dev->mounted = true;
    ESP_LOGI(TAG, "card mounted at %s", dev->mount_point);
    sdmmc_card_print_info(stdout, dev->card);
    return ESP_OK;
}

/* Opens one file per stream and writes its 16-byte header. */
static esp_err_t sd_open_files(sd_logger_handle_t dev)
{
    for (int i = 0; i < STREAM_COUNT; i++) {
        sd_stream_state_t *st = &dev->streams[i];
        st->record_size = sensor_record_size((sensor_stream_t)i);
        st->filename    = s_filenames[i];
        st->buf_used    = 0;

        char path[64];
        snprintf(path, sizeof(path), "%s/%s", dev->mount_point, st->filename);

        /* "wb" truncates any existing file -- each power-up starts a
         * fresh log rather than appending to the last flight. */
        st->fp = fopen(path, "wb");
        if (!st->fp) {
            ESP_LOGE(TAG, "fopen %s failed", path);
            return ESP_FAIL;
        }

        sd_log_header_t hdr = {0};
        memcpy(hdr.magic, SD_LOG_MAGIC, sizeof(hdr.magic));
        hdr.version     = SD_LOG_VERSION;
        hdr.stream_id   = (uint8_t)i;
        hdr.record_size = st->record_size;

        if (fwrite(&hdr, 1, sizeof(hdr), st->fp) != sizeof(hdr)) {
            ESP_LOGE(TAG, "header write failed for %s", path);
            return ESP_FAIL;
        }
        fflush(st->fp);

        ESP_LOGI(TAG, "opened %s (record size %u B)", path, st->record_size);
    }
    return ESP_OK;
}

static void sd_close_files(sd_logger_handle_t dev)
{
    for (int i = 0; i < STREAM_COUNT; i++) {
        if (dev->streams[i].fp) {
            fclose(dev->streams[i].fp);
            dev->streams[i].fp = NULL;
        }
    }
}

/* ===================================================================
 *  Public API
 * =================================================================== */

esp_err_t sd_logger_init(const sd_logger_config_t *cfg, sd_logger_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    struct sd_logger_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < SD_LOGGER_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", SD_LOGGER_MAX_INSTANCES);
        return ESP_ERR_NO_MEM;
    }

    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    dev->diag_lock = tmp;

    dev->cfg = *cfg;   /* shallow copy -- cfg can die after this returns */

    /* mount_point is a const char* in the config, but the caller's
     * string could be a temporary. Copy the text, not the pointer. */
    const char *mp = cfg->mount_point ? cfg->mount_point : "/sdcard";
    strncpy(dev->mount_point, mp, sizeof(dev->mount_point) - 1);
    dev->mount_point[sizeof(dev->mount_point) - 1] = '\0';
    dev->cfg.mount_point = dev->mount_point;

    esp_err_t err = sd_mount_card(dev);
    if (err != ESP_OK) {
        dev->in_use = false;
        return err;
    }

    err = sd_open_files(dev);
    if (err != ESP_OK) {
        sd_close_files(dev);
        esp_vfs_fat_sdcard_unmount(dev->mount_point, dev->card);
        dev->mounted = false;
        dev->in_use  = false;
        return err;
    }

    bool rb_ok = true;
    /* Element size per buffer is sizeof(that stream's record). It MUST
     * equal sensor_record_size(stream) -- ring_buffer copies exactly
     * elem_size bytes from the pointer it is given, so a mismatch would
     * read past the caller's object on push and hand a short record to
     * the file on pop. The static asserts beside sd_logger_write() are
     * what keep the two definitions from drifting apart. */
    rb_ok &= ring_buffer_init(&dev->baro_rb, dev->baro_rb_storage,
                              sizeof(baro_record_t), BARO_QUEUE_DEPTH);
    rb_ok &= ring_buffer_init(&dev->imu_rb, dev->imu_rb_storage,
                              sizeof(imu_record_t), IMU_QUEUE_DEPTH);
    rb_ok &= ring_buffer_init(&dev->gps_rb, dev->gps_rb_storage,
                              sizeof(gps_record_t), GPS_QUEUE_DEPTH);
    rb_ok &= ring_buffer_init(&dev->batt_rb, dev->batt_rb_storage,
                              sizeof(batt_record_t), BATT_QUEUE_DEPTH);
    rb_ok &= ring_buffer_init(&dev->err_rb, dev->err_rb_storage,
                              sizeof(err_record_t), ERR_QUEUE_DEPTH);
    rb_ok &= ring_buffer_init(&dev->event_rb, dev->event_rb_storage,
                              sizeof(event_record_t), EVENT_QUEUE_DEPTH);
    rb_ok &= ring_buffer_init(&dev->status_rb, dev->status_rb_storage,
                              sizeof(status_record_t), STATUS_QUEUE_DEPTH);
    if (!rb_ok) {
        ESP_LOGE(TAG, "ring buffer init failed");
        sd_close_files(dev);
        esp_vfs_fat_sdcard_unmount(dev->mount_point, dev->card);
        dev->mounted = false;
        dev->in_use  = false;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "queue depths: baro=%d imu=%d gps=%d batt=%d err=%d event=%d status=%d",
             BARO_QUEUE_DEPTH, IMU_QUEUE_DEPTH, GPS_QUEUE_DEPTH, BATT_QUEUE_DEPTH,
             ERR_QUEUE_DEPTH, EVENT_QUEUE_DEPTH, STATUS_QUEUE_DEPTH);

    dev->initialized = true;
    *out = dev;
    ESP_LOGI(TAG, "SD logger initialized (SDMMC 4-bit, %d streams)", STREAM_COUNT);
    return ESP_OK;
}

esp_err_t sd_logger_start(sd_logger_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->task) {
        ESP_LOGW(TAG, "task already running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->task_should_run = true;
    dev->task = xTaskCreateStaticPinnedToCore(
        sd_logger_task, "sd_log", SD_LOGGER_STACK_WORDS,
        (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        dev->task_should_run = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sd_logger_stop(sd_logger_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Ask the task to finish its drain-and-flush, then wait for it to
     * delete itself. This is deliberately NOT vTaskDelete() from the
     * outside: killing the writer mid-flush would strand whatever is
     * still in the buffers. */
    if (dev->task) {
        dev->task_should_run = false;
        for (int i = 0; i < 50; i++) {           /* up to ~500 ms */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        dev->task = NULL;
    }

    sd_close_files(dev);

    if (dev->mounted) {
        esp_vfs_fat_sdcard_unmount(dev->mount_point, dev->card);
        dev->mounted = false;
        dev->card    = NULL;
    }

    if (dev->pwr_ctrl) {
        sd_pwr_ctrl_del_on_chip_ldo(dev->pwr_ctrl);
        dev->pwr_ctrl = NULL;
    }

    /* No explicit teardown needed for dev->rb -- unlike the old
     * QueueHandle_t, it isn't a separate FreeRTOS object, just a plain
     * struct and array living inside dev itself. It goes away when the
     * pool slot below is released and later reclaimed. */

    dev->initialized = false;
    dev->in_use      = false;
    ESP_LOGI(TAG, "SD logger stopped, card unmounted");
    return ESP_OK;
}

/* ===================================================================
 *  Record sizes, pinned.
 *
 *  Each ring buffer's element size is sizeof(its record struct), and
 *  sensor_record_size() is itself a switch returning sizeof() of those
 *  same structs -- so the queue and every other consumer cannot drift
 *  apart. That much is guaranteed by construction and needs no assert.
 *
 *  What is NOT guaranteed is that the sizes stay what the FILE FORMAT
 *  expects. Add a field to baro_record_t and every size in the firmware
 *  updates itself silently and consistently, while baro.bin quietly
 *  changes shape and tools/decode_logs.py starts reading garbage that
 *  looks plausible. These pin the wire format: change one and you are
 *  forced to come here, which is the moment to bump the log version and
 *  update the decoders.
 * =================================================================== */
_Static_assert(sizeof(baro_record_t)   == 24, "baro.bin format changed -- bump the log version");
_Static_assert(sizeof(imu_record_t)    == 36, "imu.bin format changed -- bump the log version");
_Static_assert(sizeof(gps_record_t)    == 34, "gps.bin format changed -- bump the log version");
_Static_assert(sizeof(batt_record_t)   == 20, "batt.bin format changed -- bump the log version");
_Static_assert(sizeof(err_record_t)    == 24, "err.bin format changed -- bump the log version");
_Static_assert(sizeof(event_record_t)  == 20, "event.bin format changed -- bump the log version");
_Static_assert(sizeof(status_record_t) == 28, "status.bin format changed -- bump the log version");

/* sd_drain_one_rb()'s scratch buffer is SENSOR_RECORD_MAX_SIZE bytes and
 * ring_buffer_pop() writes elem_size into it, so every record must fit. */
_Static_assert(sizeof(baro_record_t)   <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");
_Static_assert(sizeof(imu_record_t)    <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");
_Static_assert(sizeof(gps_record_t)    <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");
_Static_assert(sizeof(batt_record_t)   <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");
_Static_assert(sizeof(err_record_t)    <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");
_Static_assert(sizeof(event_record_t)  <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");
_Static_assert(sizeof(status_record_t) <= SENSOR_RECORD_MAX_SIZE, "drain scratch too small");

/* A new stream means a new buffer, a new file, and a new line in every
 * list above. This is the tripwire that makes forgetting a build error. */
_Static_assert(STREAM_COUNT == 7, "a stream was added -- update the buffers above");

esp_err_t sd_logger_write(sd_logger_handle_t dev, sensor_stream_t stream,
                          const void *record, size_t len)
{
    if (!dev || !dev->initialized || !record) {
        return ESP_ERR_INVALID_ARG;
    }
    if (stream >= STREAM_COUNT || len == 0 || len > SD_MAX_RECORD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* EXACT length, not "small enough". Each buffer's element size is
     * now sizeof(that stream's record), and ring_buffer_push() copies
     * exactly that many bytes from the pointer it is handed. A caller
     * passing a shorter object than the stream implies would have those
     * bytes read from past the end of it. Under the old 42-byte
     * envelope a wrong `len` merely produced a short record in the
     * file; here it is an out-of-bounds read, so the check is no longer
     * a sanity check but a bounds check, and it rejects rather than
     * truncates. */
    if (len != sensor_record_size(stream)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ring_buffer_t *rb = rb_for_stream(dev, stream);
    if (!rb) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->diag.records_queued++;

    /* ring_buffer_push() never blocks and never rejects: if THIS
     * stream's own buffer is full it discards its OWN oldest unwritten
     * record. Since each stream now has a separate buffer, this can
     * never evict a different sensor's data -- only this one's. */
    /* The record goes in raw -- no stream tag, no length. Which buffer
     * it landed in IS the stream, and the stream IS the length. */
    if (ring_buffer_push(rb, record)) {
        dev->diag.queue_full_drops++;
        /* Deliberately NOT event_log_error() here. This function IS the
         * push path: event_log_error() -> data_router_publish() ->
         * sd_logger_write() would land straight back in this function,
         * and a full queue would then recurse without bound. Drops are
         * reported from sd_logger_task() instead, which sees the same
         * counter from outside the push path. Do not "fix" this by
         * adding the obvious call. */
    }

    return ESP_OK;
}

esp_err_t sd_logger_flush(sd_logger_handle_t dev)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = ESP_OK;
    for (int i = 0; i < STREAM_COUNT; i++) {
        if (sd_flush_stream(dev, &dev->streams[i]) != ESP_OK) {
            result = ESP_FAIL;
        }
    }
    return result;
}

void sd_logger_get_diagnostics(sd_logger_handle_t dev, sd_logger_diag_t *out)
{
    if (!dev || !out) {
        return;
    }
    *out = dev->diag;
}
