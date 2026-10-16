#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/* Pulled in only when building under ESP-IDF. The tunables below fall
 * back to hardcoded defaults when it's absent, so this header also
 * works in a plain non-Kconfig build or a host-side test. */
#if defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"
#  endif
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_records.h"

#define SD_LOGGER_MAX_INSTANCES   1

/* All four tunables below come from menuconfig (see this component's
 * Kconfig). The #ifndef fallbacks keep the header usable if it is ever
 * compiled outside an ESP-IDF build -- e.g. by the host-side tests. */
#ifdef CONFIG_SD_LOGGER_STACK_WORDS
#define SD_LOGGER_STACK_WORDS     CONFIG_SD_LOGGER_STACK_WORDS
#else
#define SD_LOGGER_STACK_WORDS     4096
#endif

/* Queue depth: how many records can pile up while the SD card is busy.
 * SD writes stall unpredictably (wear levelling, block erase), so this
 * buffer is what keeps a slow write from ever blocking a sensor task.
 * 128 records at the worst-case combined rate (~155 Hz) is ~0.8 s of
 * headroom. */
/* Bytes buffered per stream before touching the card. 512 keeps each
 * single write within one SD sector; it does NOT guarantee flushes
 * land on aligned sector boundaries, since flush sizes vary by record
 * size and rarely divide evenly into 512 -- see sd_logger.c's flush
 * comments for the actual tradeoff this represents. */
#ifdef CONFIG_SD_LOGGER_BUF_SIZE
#define SD_LOGGER_BUF_SIZE        CONFIG_SD_LOGGER_BUF_SIZE
#else
#define SD_LOGGER_BUF_SIZE        512
#endif

/* Flush every stream at least this often even if its buffer isn't full,
 * so a slow stream (GPS at 1 Hz) doesn't sit unwritten for minutes. */
#ifdef CONFIG_SD_LOGGER_FLUSH_MS
#define SD_LOGGER_FLUSH_MS        CONFIG_SD_LOGGER_FLUSH_MS
#else
#define SD_LOGGER_FLUSH_MS        1000
#endif

/* Commit file sizes to the card (fsync) at every periodic flush, so a
 * card pulled without an eject keeps everything but the last interval.
 * A Kconfig bool is simply absent when off, so "is there a Kconfig build
 * at all" is told apart by CONFIG_SD_LOGGER_FLUSH_MS, which always
 * exists in one. With no sdkconfig.h (host tests) it defaults on. */
#if defined(CONFIG_SD_LOGGER_FLUSH_MS)
#  ifdef CONFIG_SD_LOGGER_SYNC
#    define SD_LOGGER_SYNC        1
#  else
#    define SD_LOGGER_SYNC        0
#  endif
#else
#  define SD_LOGGER_SYNC          1
#endif

/* Per-stream queue depths, derived from each sensor's own real rate --
 * not one shared depth for all three anymore. Previously one shared
 * ring buffer meant a burst from a fast sensor (IMU, 104 Hz) could
 * evict a slower sensor's (GPS, ~1 Hz) still-unwritten records purely
 * because they shared eviction pressure. Separate buffers mean a
 * sensor can now only ever evict its OWN backlog.
 *
 * Sizing: rate_hz * (SD_LOGGER_FLUSH_MS + margin) / 1000, giving each
 * buffer headroom for one full flush interval PLUS room to absorb one
 * extra card stall on top of that, using THAT sensor's true rate.
 * GPS's raw proportional result (under 2 items) is floored to a
 * sane minimum instead -- a queue that small has no room to absorb
 * even a brief scheduling hiccup. */
#ifdef CONFIG_SD_QUEUE_MARGIN_MS
#define SD_QUEUE_MARGIN_MS        CONFIG_SD_QUEUE_MARGIN_MS
#else
#define SD_QUEUE_MARGIN_MS        500
#endif

#define BMP388_RATE_HZ            50
#define LSM6DSOX_RATE_HZ          104
#define GPS_RATE_HZ               1
#define BATT_RATE_HZ              10
#define GPS_QUEUE_DEPTH_MIN       8

#define BARO_QUEUE_DEPTH  ((BMP388_RATE_HZ   * (SD_LOGGER_FLUSH_MS + SD_QUEUE_MARGIN_MS)) / 1000)
#define IMU_QUEUE_DEPTH   ((LSM6DSOX_RATE_HZ * (SD_LOGGER_FLUSH_MS + SD_QUEUE_MARGIN_MS)) / 1000)
#define GPS_QUEUE_DEPTH_RAW ((GPS_RATE_HZ    * (SD_LOGGER_FLUSH_MS + SD_QUEUE_MARGIN_MS)) / 1000)
#define GPS_QUEUE_DEPTH   (GPS_QUEUE_DEPTH_RAW > GPS_QUEUE_DEPTH_MIN ? GPS_QUEUE_DEPTH_RAW : GPS_QUEUE_DEPTH_MIN)
#define BATT_QUEUE_DEPTH  ((BATT_RATE_HZ     * (SD_LOGGER_FLUSH_MS + SD_QUEUE_MARGIN_MS)) / 1000)

/* The error and event streams are NOT rate-derived, because neither has
 * a rate. Errors are bursty -- zero for a whole flight, then one per
 * sample the moment a sensor starts failing -- and events are a couple
 * of dozen records total. Sizing either from a nominal Hz would be
 * inventing a number and then trusting it.
 *
 * ERR is sized to hold one full rate-limiter window from every source
 * at once (ERR_SRC_COUNT sources x the burst allowance, rounded up)
 * so a simultaneous multi-component failure at boot is captured whole.
 * EVENT is sized well past the number of events a flight can produce;
 * dropping one would tear a hole in the timeline. */
#define ERR_QUEUE_DEPTH           32
#define EVENT_QUEUE_DEPTH         16

/* Status is periodic but slow (1 Hz) and each record is a snapshot --
 * losing one costs a second of health history, not a measurement. Small
 * fixed depth; there is nothing to burst. */
#define STATUS_QUEUE_DEPTH        8

/* ===================================================================
 *  On-disk format
 *
 *  Every file: a 16-byte header, then a tight array of fixed-size
 *  entries. Each entry is one record struct followed by a 2-byte
 *  CRC16-CCITT over that record, so a torn or corrupted entry is
 *  detectable offline rather than decoding as plausible garbage.
 *  record_size in the header is the PAYLOAD size; on-disk stride is
 *  record_size + 2.
 *
 *  __attribute__((packed)) matters here: without it the compiler would
 *  insert alignment padding (exactly the 4-byte gap we found in
 *  bmp388_data_t), and those pad bytes would be written to the card as
 *  garbage the decoder would have to know about. Packed keeps the
 *  on-disk layout identical to the field list below.
 * =================================================================== */

#define SD_LOG_MAGIC        "SETLOG"
#define SD_LOG_VERSION      2   /* 2: status_record_t lost battery_v (32 -> 28 bytes) */

typedef struct __attribute__((packed)) {
    char     magic[6];        // "SETLOG"
    uint16_t version;
    uint8_t  stream_id;
    uint8_t  record_size;
    uint8_t  reserved[6];
} sd_log_header_t;            // 16 bytes

/* sensor_stream_t and {baro,imu,gps}_record_t now live in
 * sensor_records.h, shared with telemetry.h and flight_logic.h -- SD
 * was never supposed to be the naming authority for these. */

/* ===================================================================
 *  Config / diagnostics
 * =================================================================== */

typedef struct {
    /* No pin fields here. The SDMMC 4-bit slot's CLK/CMD/D0-D3 lines are
     * fixed by the ESP32-P4's SDMMC peripheral, and the card plugs
     * directly into that slot -- there's no wiring choice to make, so
     * unlike the sensor drivers' cs_pin/int_pin, this isn't something a
     * caller should be able to override. The pin numbers live as
     * private constants inside sd_logger.c instead. */

    /* ESP32-P4 powers its SDMMC IO pins from an on-chip LDO that must be
     * enabled before the card will respond. 4 = LDO_VO4, the standard
     * channel on P4 dev boards. */
    int         ldo_chan_id;

    const char *mount_point;   // e.g. "/sdcard"; NULL => "/sdcard"
    int         max_freq_khz;  // 0 => SDMMC_FREQ_DEFAULT (20 MHz)
    bool        format_if_mount_failed;
} sd_logger_config_t;

#define SD_CRC_SIZE   2   /* bytes appended after each record on disk */

typedef struct {
    uint32_t records_queued;    // handed to sd_logger_write()
    uint32_t records_written;   // actually made it into a file buffer
    uint32_t queue_full_drops;  // an unwritten record was overwritten by a
                                 // newer one because the card couldn't keep up
    uint32_t write_errors;      // fwrite/fflush/fsync failures
    uint32_t flushes;           // buffer -> FATFS operations
    uint32_t syncs;             // fsync()s: a file's size committed on the card
} sd_logger_diag_t;

/* ===================================================================
 *  Opaque handle -- struct sd_logger_dev lives only in sd_logger.c,
 *  same idiom as bmp388_handle_t / lsm6dsox_handle_t / gps_handle_t.
 * =================================================================== */

typedef struct sd_logger_dev *sd_logger_handle_t;

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

/* Claims a pool slot, brings up the SDMMC peripheral in 4-bit mode,
 * mounts FATFS, and opens one file per stream with its header written.
 * Files are named /<mount>/baro.bin, imu.bin, gps.bin, batt.bin,
 * err.bin, event.bin, status.bin. Does not start
 * the writer task yet. */
esp_err_t sd_logger_init(const sd_logger_config_t *cfg, sd_logger_handle_t *out);

/* Spawns the task that drains the queue and writes to the card. */
esp_err_t sd_logger_start(sd_logger_handle_t dev, UBaseType_t priority, BaseType_t core_id);

/* Flushes everything still buffered, closes the files, unmounts the
 * card, kills the task, releases the pool slot.
 *
 * With SD_LOGGER_SYNC on (the default), skipping this -- pulling the
 * card, or losing power -- costs at most the last SD_LOGGER_FLUSH_MS of
 * data. Still call it when you can: it is the only way to be sure no
 * write is in progress at the instant the card comes out. With
 * SD_LOGGER_SYNC off, skipping it loses EVERY file's contents, because
 * nothing else writes their sizes to the card. */
esp_err_t sd_logger_stop(sd_logger_handle_t dev);

/* ===================================================================
 *  Logging
 * =================================================================== */

/* Enqueue one record for the given stream. Non-blocking: if the buffer
 * is full (card stalled), the OLDEST unwritten record is discarded to
 * make room for this one, and queue_full_drops is incremented -- the
 * caller is never stalled. Safe to call from any task; NOT safe from
 * an ISR (use ring_buffer_push_from_isr()-style handling if that's ever
 * needed -- this driver does not currently do so). */
esp_err_t sd_logger_write(sd_logger_handle_t dev, sensor_stream_t stream,
                          const void *record, size_t len);

/* Force every stream's buffer out to the card right now, sizes included
 * (when SD_LOGGER_SYNC is on). Blocks until done. Use before a known
 * risky moment. Not needed for flight events: the writer task already
 * commits the moment an event record is drained.
 *
 * Touches the same FILE*s as the writer task without a lock, so do not
 * call it while that task is running. Nothing currently does. */
esp_err_t sd_logger_flush(sd_logger_handle_t dev);

void sd_logger_get_diagnostics(sd_logger_handle_t dev, sd_logger_diag_t *out);

#endif // SD_LOGGER_H
