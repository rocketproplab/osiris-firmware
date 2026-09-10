/*
 * Code written by Claude 7/24/2026. [DO NOT FLY]
 */

/*
 * PA1616D (MT3333) bring-up over UART.
 *
 * Settings are volatile without VBAT backup, and unreliable even with it, so
 * this runs unconditionally at every boot. It auto-detects the current baud
 * rather than assuming 9600, because a module left at 57600 by a previous
 * session is indistinguishable from a dead one otherwise.
 *
 * Order matters: trim sentences, then raise baud, then raise fix rate.
 * Raising the rate first overruns the link and truncates sentences.
 */

#include <stdio.h>
#include <string.h>
#include "ai_gps_config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPS_UART        UART_NUM_1
#define GPS_TX_PIN      /* your pin */ 17
#define GPS_RX_PIN      /* your pin */ 18
#define GPS_RX_BUF      2048

#define GPS_TARGET_BAUD 57600
#define GPS_TARGET_HZ   10

static const char *TAG = "ai_gps";

/* Baud rates to probe, most likely first. 9600 is the power-on default. */
static const int k_probe_bauds[] = { 9600, 57600, 115200, 38400 };

/* XOR of every byte between '$' and '*'. */
static uint8_t nmea_checksum(const char *body)
{
    uint8_t c = 0;
    for (const char *p = body; *p; p++) {
        c ^= (uint8_t)*p;
    }
    return c;
}

/* Frames body as $body*hh\r\n and writes it. Computing the checksum here
 * rather than hardcoding it means a mistyped command fails as "no ack"
 * instead of silently being ignored for a bad checksum. */
static void gps_send(const char *body)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, nmea_checksum(body));
    if (n < 0 || n >= (int)sizeof(buf)) {
        ESP_LOGE(TAG, "command too long: %s", body);
        return;
    }
    uart_write_bytes(GPS_UART, buf, n);
    uart_wait_tx_done(GPS_UART, pdMS_TO_TICKS(100));
}

/* Scans the RX stream for $PMTK001,<pkt>,3 — flag 3 is "valid, succeeded".
 * Flag 2 means the command parsed but the action failed, which is a distinct
 * and more interesting failure than no reply at all. */
static bool gps_wait_ack(int pkt_type, int timeout_ms)
{
    char want_ok[24], want_any[24];
    snprintf(want_ok,  sizeof(want_ok),  "$PMTK001,%d,3", pkt_type);
    snprintf(want_any, sizeof(want_any), "$PMTK001,%d,", pkt_type);

    char line[128];
    size_t len = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        uint8_t ch;
        int got = uart_read_bytes(GPS_UART, &ch, 1, pdMS_TO_TICKS(20));
        if (got != 1) continue;

        if (ch == '\n' || ch == '\r') {
            line[len] = '\0';
            if (len > 0) {
                if (strncmp(line, want_ok, strlen(want_ok)) == 0) return true;
                if (strncmp(line, want_any, strlen(want_any)) == 0) {
                    ESP_LOGW(TAG, "PMTK%d rejected: %s", pkt_type, line);
                    return false;
                }
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)ch;
        } else {
            len = 0;  /* overlong line, resync */
        }
    }
    ESP_LOGW(TAG, "PMTK%d: no ack", pkt_type);
    return false;
}

/* True if any well-formed NMEA sentence arrives. Does not require a fix —
 * the module talks before it has satellites, and we only need to know the
 * baud is right. */
static bool gps_sees_nmea(int timeout_ms)
{
    char line[128];
    size_t len = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        uint8_t ch;
        if (uart_read_bytes(GPS_UART, &ch, 1, pdMS_TO_TICKS(20)) != 1) continue;

        if (ch == '\n' || ch == '\r') {
            line[len] = '\0';
            /* '$' plus a '*' checksum delimiter is enough to rule out the
             * framing-error garbage you get at the wrong baud. */
            if (len > 6 && line[0] == '$' && strchr(line, '*') != NULL) {
                return true;
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)ch;
        } else {
            len = 0;
        }
    }
    return false;
}

static void gps_set_local_baud(int baud)
{
    ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART, baud));
    ESP_ERROR_CHECK(uart_flush_input(GPS_UART));
}

esp_err_t ai_gps_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = k_probe_bauds[0],
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, GPS_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Find the module. */
    int found = -1;
    for (size_t i = 0; i < sizeof(k_probe_bauds) / sizeof(k_probe_bauds[0]); i++) {
        gps_set_local_baud(k_probe_bauds[i]);
        if (gps_sees_nmea(1500)) {
            found = k_probe_bauds[i];
            ESP_LOGI(TAG, "found at %d baud", found);
            break;
        }
    }
    if (found < 0) {
        ESP_LOGE(TAG, "no NMEA at any probed baud, check wiring and 3V3");
        return ESP_ERR_NOT_FOUND;
    }

    /* 1. Trim sentences first, while the link is still slow.
     *    GSV is the expensive one: it enumerates every satellite in view. */
    gps_send("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");  /* GGA + RMC */
    gps_wait_ack(314, 1000);

    /* 2. Raise the link. The module switches the instant this lands, so
     *    everything after it is unreadable until the local side follows. */
    if (found != GPS_TARGET_BAUD) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "PMTK251,%d", GPS_TARGET_BAUD);
        gps_send(cmd);
        vTaskDelay(pdMS_TO_TICKS(100));
        gps_set_local_baud(GPS_TARGET_BAUD);

        /* No ack for 251 — it arrives at the new rate mid-switch and is not
         * reliably readable. Confirm by seeing valid sentences instead. */
        if (!gps_sees_nmea(1500)) {
            ESP_LOGE(TAG, "lost module after baud change to %d", GPS_TARGET_BAUD);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGI(TAG, "now at %d baud", GPS_TARGET_BAUD);
    }

    /* 3. Rate last, once the link can carry it. */
    char rate[32];
    snprintf(rate, sizeof(rate), "PMTK220,%d", 1000 / GPS_TARGET_HZ);
    gps_send(rate);
    gps_wait_ack(220, 1000);

    snprintf(rate, sizeof(rate), "PMTK300,%d,0,0,0,0", 1000 / GPS_TARGET_HZ);
    gps_send(rate);
    gps_wait_ack(300, 1000);

    /* 4. Aviation dynamic model. The default assumes pedestrian motion and
     *    filters against exactly the accelerations we care about. */
    gps_send("PMTK886,2");
    gps_wait_ack(886, 1000);

    /* 5. Firmware string into the log, so a post-flight anomaly can be tied
     *    to a specific module build. */
    gps_send("PMTK605");

    ESP_LOGI(TAG, "configured: GGA+RMC, %d baud, %d Hz, aviation mode",
             GPS_TARGET_BAUD, GPS_TARGET_HZ);
    return ESP_OK;
}