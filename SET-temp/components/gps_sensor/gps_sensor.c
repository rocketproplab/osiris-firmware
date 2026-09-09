#include "gps_sensor.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "data_router.h"

static const char *TAG = "GPS_PA1616D";

#define GPS_BAUD_RATE       9600   // PA1616D default -- do not change unless you
                                    // send a $PMTK251 command first to reconfigure it
#define GPS_UART_BUF_SIZE   512
#define GPS_LINE_MAX        128

/* ===================================================================
 *  The real struct -- private to this file, same idiom as bmp388_dev
 *  and lsm6dsox_dev. Everything that used to be scattered file-scope
 *  statics (s_uart_port, s_latest, s_have_fix, s_mutex) now lives here.
 * =================================================================== */

struct gps_dev {
    bool                in_use;
    gps_config_t        cfg;
    bool                initialized;
    bool                en_level;   // last level driven on EN

    gps_data_t          latest;
    portMUX_TYPE        latest_lock;   /* replaces the old heap-allocated
                                         * mutex -- see notes below */

    StaticTask_t        tcb;
    StackType_t         stack[GPS_STACK_WORDS];
    TaskHandle_t        task;

    volatile uint64_t   pps_timestamp_us;  /* written by the PPS ISR */
    bool                pps_isr_installed;

    gps_diag_t          diag;
};

/* ===================================================================
 *  Static instance pool
 * =================================================================== */

static struct gps_dev s_pool[GPS_MAX_INSTANCES];
static portMUX_TYPE   s_pool_lock = portMUX_INITIALIZER_UNLOCKED;

/* ===================================================================
 *  NMEA parsing helpers -- pure functions, unchanged from the working
 *  version. Nothing about the hardware/task refactor touches these.
 * =================================================================== */

// Converts NMEA's "ddmm.mmmm" (or "dddmm.mmmm" for longitude) format into
// plain decimal degrees.
static double nmea_to_decimal_degrees(const char *field, int deg_digits)
{
    (void)deg_digits;  /* unused: the /100 split works the same regardless
                         * of whether degrees is 2 digits (lat) or 3 (lon) */
    double raw = atof(field);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    return degrees + (minutes / 60.0);
}

// Splits a comma-separated NMEA sentence into up to max_fields pointers.
// Modifies the input buffer in place (replaces commas with '\0').
static int split_fields(char *sentence, char *fields[], int max_fields)
{
    int count = 0;
    char *p = sentence;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

// Parses a GGA sentence (fix data: position, altitude, fix quality, satellites).
// Handles both $GPGGA (GPS only) and $GNGGA (GPS+GLONASS combined) -- the
// PA1616D uses $GN by default since it tracks both constellations.
static void parse_gga(gps_handle_t dev, char *sentence)
{
    char *fields[16] = {0};
    int n = split_fields(sentence, fields, 16);

    dev->diag.sentences_parsed++;

    if (n < 10) {
        dev->diag.parse_errors++;
        return;   // malformed/incomplete sentence
    }

    // Fields: [0]=sentence id [1]=time [2]=lat [3]=N/S [4]=lon [5]=E/W
    //         [6]=fix quality [7]=num satellites [8]=hdop [9]=altitude

    if (strlen(fields[2]) == 0 || strlen(fields[4]) == 0) {
        return;   // empty position fields -- no fix yet, nothing to parse
    }

    double lat = nmea_to_decimal_degrees(fields[2], 2);
    if (fields[3][0] == 'S') lat = -lat;

    double lon = nmea_to_decimal_degrees(fields[4], 3);
    if (fields[5][0] == 'W') lon = -lon;

    uint8_t fix_quality = (uint8_t)atoi(fields[6]);
    uint8_t satellites  = (uint8_t)atoi(fields[7]);
    float altitude      = atof(fields[9]);

    if (fix_quality > 0) {
        gps_data_t sample;
        sample.latitude    = lat;
        sample.longitude   = lon;
        sample.altitude_m  = altitude;
        sample.fix_quality = fix_quality;
        sample.satellites  = satellites;
        sample.timestamp_us = dev->pps_timestamp_us;  /* last hardware PPS edge --
                                                        * see the correlation note
                                                        * above gps_pps_isr() */
        sample.seq          = ++dev->diag.fixes;

        portENTER_CRITICAL(&dev->latest_lock);
        dev->latest = sample;
        portEXIT_CRITICAL(&dev->latest_lock);

        /* Same pattern as bmp388/lsm6dsox: pushed right where the fix
         * already exists, inside the same guard that only lets a
         * genuine fix through -- never pushes a fix_quality==0 sample. */
        gps_record_t rec = {
            .seq          = sample.seq,
            .timestamp_us = sample.timestamp_us,
            .latitude     = sample.latitude,
            .longitude    = sample.longitude,
            .altitude_m   = sample.altitude_m,
            .fix_quality  = sample.fix_quality,
            .satellites   = sample.satellites,
        };
        data_router_publish(STREAM_GPS, &rec, sizeof(rec));
    }
}

/* ===================================================================
 *  Reader task -- same byte-at-a-time UART loop as the working version,
 *  just reading dev->cfg.uart_port instead of a fixed global.
 * =================================================================== */

static void gps_task(void *arg)
{
    gps_handle_t dev = (gps_handle_t)arg;
    char line[GPS_LINE_MAX];
    int line_pos = 0;
    uint8_t byte;

    ESP_LOGI(TAG, "reader task running on core %d", (int)xPortGetCoreID());

    while (1) {
        int len = uart_read_bytes(dev->cfg.uart_port, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        if (byte == '\n') {
            line[line_pos] = '\0';

            if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
                parse_gga(dev, line);
            }

            line_pos = 0;
        } else if (byte == '\r') {
            // just skip it -- don't touch line_pos
        } else if (line_pos < GPS_LINE_MAX - 1) {
            line[line_pos++] = (char)byte;
        } else {
            dev->diag.line_overflows++;
            line_pos = 0;   // genuinely too long -- reset and resync
        }
    }
}

/* ===================================================================
 *  PPS (Pulse Per Second)
 *
 *  The PA1616D asserts this pin once a second, on the rising edge of
 *  each UTC second boundary -- far more precise than anything software
 *  timing could give you off the UART byte stream alone, same idea as
 *  the BMP388's DRDY pin giving a hardware-exact sample instant.
 *
 *  Correlation caveat, stated honestly rather than overclaimed: MTK
 *  chipsets (which the PA1616D is built on) emit the PPS pulse aligned
 *  to the start of the UTC second that the *next* NMEA sentence
 *  describes. That means "the most recent PPS edge at the moment this
 *  GGA sentence finished parsing" is the right edge to attach to it
 *  under normal conditions -- but this assumes the NMEA burst for that
 *  second arrives before the *next* PPS pulse, which is true at 9600
 *  baud with a single GGA sentence, but worth re-verifying against the
 *  PA1616D datasheet if you ever add more NMEA sentence types or slow
 *  the baud rate down.
 * =================================================================== */

static void IRAM_ATTR gps_pps_isr(void *arg)
{
    gps_handle_t dev = (gps_handle_t)arg;

    dev->pps_timestamp_us = (uint64_t)esp_timer_get_time();
    dev->diag.pps_pulses++;
}

static esp_err_t gps_setup_pps(gps_handle_t dev)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << dev->cfg.pps_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  /* PPS idles low, pulses high */
        .intr_type    = GPIO_INTR_POSEDGE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config (PPS) failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Shared service -- fine if another driver already installed it. */
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(dev->cfg.pps_pin, gps_pps_isr, (void *)dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add (PPS) failed: %s", esp_err_to_name(err));
        return err;
    }

    dev->pps_isr_installed = true;
    ESP_LOGI(TAG, "PPS armed on GPIO%d (rising edge)", (int)dev->cfg.pps_pin);
    return ESP_OK;
}

/* ===================================================================
 *  Public API
 * =================================================================== */

esp_err_t gps_init(const gps_config_t *cfg, gps_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->pps_pin < 0) {
        ESP_LOGE(TAG, "need a real PPS pin for hardware-timestamped fixes");
        return ESP_ERR_INVALID_ARG;
    }

    struct gps_dev *dev = NULL;
    portENTER_CRITICAL(&s_pool_lock);
    for (int i = 0; i < GPS_MAX_INSTANCES; i++) {
        if (!s_pool[i].in_use) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].in_use = true;
            dev = &s_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_pool_lock);

    if (!dev) {
        ESP_LOGE(TAG, "instance pool exhausted (%d slots)", GPS_MAX_INSTANCES);
        return ESP_ERR_NO_MEM;
    }

    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    dev->latest_lock = tmp;

    dev->cfg = *cfg;   /* shallow copy -- cfg can die after this call returns */

    /* EN first, before the UART exists. The module needs to be powered
     * and settled before anything tries to talk to it; bringing the UART
     * up first would just mean parsing noise from a module that is still
     * booting. */
    if (dev->cfg.en_pin != GPIO_NUM_NC) {
        gpio_config_t en_io = {
            .pin_bit_mask = 1ULL << dev->cfg.en_pin,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t en_err = gpio_config(&en_io);
        if (en_err != ESP_OK) {
            ESP_LOGE(TAG, "EN gpio_config failed: %s", esp_err_to_name(en_err));
            dev->in_use = false;
            return en_err;
        }
        gpio_set_level(dev->cfg.en_pin, 1);
        dev->en_level = true;
        ESP_LOGI(TAG, "GPS EN driven high on GPIO%d", (int)dev->cfg.en_pin);
    }

    uart_config_t uart_cfg = {
        .baud_rate  = GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(cfg->uart_port, GPS_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        dev->in_use = false;
        return err;
    }

    err = uart_param_config(cfg->uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        uart_driver_delete(cfg->uart_port);
        dev->in_use = false;
        return err;
    }

    err = uart_set_pin(cfg->uart_port, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        uart_driver_delete(cfg->uart_port);
        dev->in_use = false;
        return err;
    }

    err = gps_setup_pps(dev);
    if (err != ESP_OK) {
        uart_driver_delete(cfg->uart_port);
        dev->in_use = false;
        return err;
    }

    dev->initialized = true;
    *out = dev;
    ESP_LOGI(TAG, "GPS UART initialized (TX=%d, RX=%d, %d baud)",
             cfg->tx_pin, cfg->rx_pin, GPS_BAUD_RATE);
    return ESP_OK;
}

esp_err_t gps_start(gps_handle_t dev, UBaseType_t priority, BaseType_t core_id)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->task) {
        ESP_LOGW(TAG, "task already running");
        return ESP_ERR_INVALID_STATE;
    }

    dev->task = xTaskCreateStaticPinnedToCore(
        gps_task, "gps", GPS_STACK_WORDS,
        (void *)dev, priority, dev->stack, &dev->tcb, core_id);

    if (!dev->task) {
        ESP_LOGE(TAG, "task creation failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gps_stop(gps_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->pps_isr_installed) {
        gpio_isr_handler_remove(dev->cfg.pps_pin);
        dev->pps_isr_installed = false;
    }
    if (dev->task) {
        vTaskDelete(dev->task);
        dev->task = NULL;
    }
    if (dev->initialized) {
        uart_driver_delete(dev->cfg.uart_port);
    }

    dev->initialized = false;
    dev->in_use      = false;
    return ESP_OK;
}

esp_err_t gps_get_latest(gps_handle_t dev, gps_data_t *out)
{
    if (!dev || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&dev->latest_lock);
    *out = dev->latest;
    portEXIT_CRITICAL(&dev->latest_lock);

    /* seq starts at 0 and only increments once a real fix lands -- same
     * convention bmp388_zero_altitude() uses to detect "no sample yet." */
    return (out->seq == 0) ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t gps_wait_for_fix(gps_handle_t dev, uint32_t last_seq,
                           gps_data_t *out, TickType_t timeout)
{
    if (!dev || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    const TickType_t start = xTaskGetTickCount();
    const TickType_t poll  = pdMS_TO_TICKS(50);   /* fixes arrive at ~1 Hz,
                                                    * no need to poll fast */

    for (;;) {
        portENTER_CRITICAL(&dev->latest_lock);
        *out = dev->latest;
        portEXIT_CRITICAL(&dev->latest_lock);

        if (out->seq != last_seq && out->seq != 0) {
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(poll);
    }
}

void gps_get_diagnostics(gps_handle_t dev, gps_diag_t *out)
{
    if (!dev || !out) {
        return;
    }
    *out = dev->diag;
}

/* ===================================================================
 *  One-time provisioning: raise baud rate and fix update rate
 *
 *  NOT called during normal startup -- see the danger note in
 *  gps_sensor.h. UNTESTED against real hardware: the PMTK command
 *  strings and their checksums are from published MTK/Adafruit
 *  documentation, but this has not been run against an actual PA1616D.
 *  Verify on the bench before relying on it.
 * =================================================================== */

/* NMEA checksum: XOR of every byte between '$' and '*', printed as two
 * uppercase hex digits. Computed at runtime rather than hardcoded so
 * arbitrary baud/rate values work, not just the two documented ones. */
static void gps_send_pmtk(gps_handle_t dev, const char *body)
{
    uint8_t ck = 0;
    for (const char *p = body; *p; p++) {
        ck ^= (uint8_t)*p;
    }

    char sentence[96];
    int n = snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, ck);
    if (n <= 0 || n >= (int)sizeof(sentence)) {
        ESP_LOGE(TAG, "PMTK sentence too long, not sent");
        return;
    }

    uart_write_bytes(dev->cfg.uart_port, sentence, (size_t)n);
    uart_wait_tx_done(dev->cfg.uart_port, pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "sent: %s", body);
}

esp_err_t gps_reconfigure(gps_handle_t dev, uint32_t new_baud, uint32_t update_rate_ms)
{
    if (!dev || !dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char body[48];

    /* Update rate FIRST, while still at the known-good baud rate. If
     * this were done after the baud change and the baud change failed,
     * we'd be sending to a module we can no longer talk to. */
    if (update_rate_ms > 0) {
        snprintf(body, sizeof(body), "PMTK220,%lu", (unsigned long)update_rate_ms);
        gps_send_pmtk(dev, body);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    if (new_baud > 0 && new_baud != GPS_BAUD_RATE) {
        snprintf(body, sizeof(body), "PMTK251,%lu", (unsigned long)new_baud);
        gps_send_pmtk(dev, body);

        /* The module switches the instant it parses that sentence. Our
         * UART must follow immediately or the link is dead. The delay
         * gives the module time to finish applying it before we change
         * sides. */
        vTaskDelay(pdMS_TO_TICKS(250));

        esp_err_t err = uart_set_baudrate(dev->cfg.uart_port, new_baud);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uart_set_baudrate(%lu) failed: %s -- link is now BROKEN, "
                          "module is at the new rate but we are not",
                     (unsigned long)new_baud, esp_err_to_name(err));
            return err;
        }
        ESP_LOGW(TAG, "baud changed to %lu. Update GPS_BAUD_RATE in gps_sensor.c "
                      "to match, or the next boot will fail to parse.",
                 (unsigned long)new_baud);
    }

    return ESP_OK;
}

/* ===================================================================
 *  Power control
 * =================================================================== */

esp_err_t gps_set_enabled(gps_handle_t dev, bool enabled)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->cfg.en_pin == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "gps_set_enabled: no EN pin configured");
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = gpio_set_level(dev->cfg.en_pin, enabled ? 1 : 0);
    if (err != ESP_OK) {
        return err;
    }
    dev->en_level = enabled;

    if (enabled) {
        ESP_LOGI(TAG, "GPS enabled -- expect a cold start if it was off");
    } else {
        ESP_LOGW(TAG, "GPS disabled -- almanac and any custom baud are lost");
    }
    return ESP_OK;
}

bool gps_is_enabled(gps_handle_t dev)
{
    if (!dev) {
        return false;
    }
    /* No EN pin means the module's internal pull-up holds it enabled. */
    return dev->cfg.en_pin == GPIO_NUM_NC ? true : dev->en_level;
}
