#include "data_router.h"

#include "esp_log.h"

static const char *TAG = "DATA_ROUTER";

static sd_logger_handle_t s_sd = NULL;
static telemetry_handle_t s_telemetry = NULL;
static bool                s_initialized = false;

esp_err_t data_router_init(sd_logger_handle_t sd, telemetry_handle_t telemetry)
{
    s_sd          = sd;
    s_telemetry   = telemetry;
    s_initialized = true;

    ESP_LOGI(TAG, "router initialized: sd=%s telemetry=%s",
             sd ? "yes" : "no", telemetry ? "yes" : "no");
    return ESP_OK;
}

esp_err_t data_router_publish(sensor_stream_t stream, const void *record, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bool any_destination = false;

    if (s_sd) {
        any_destination = true;
        sd_logger_write(s_sd, stream, record, len);
    }
    if (s_telemetry) {
        any_destination = true;
        telemetry_write(s_telemetry, stream, record, len);
    }

    return any_destination ? ESP_OK : ESP_ERR_NOT_FOUND;
}
