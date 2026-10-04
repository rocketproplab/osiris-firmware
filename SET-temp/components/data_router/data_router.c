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

sd_logger_handle_t data_router_detach_sd(void)
{
    sd_logger_handle_t sd = s_sd;
    s_sd = NULL;
    if (sd) {
        ESP_LOGW(TAG, "SD detached from the router; nothing more will be logged to the card");
    }
    return sd;
}

esp_err_t data_router_publish(sensor_stream_t stream, const void *record, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bool any_destination = false;

    /* Read each destination ONCE into a local. data_router_detach_sd()
     * can clear s_sd from another task between the test and the call,
     * and a re-read would turn that into a NULL dereference on a path
     * every sensor task runs at 104 Hz. */
    sd_logger_handle_t sd = s_sd;
    telemetry_handle_t tm = s_telemetry;

    if (sd) {
        any_destination = true;
        sd_logger_write(sd, stream, record, len);
    }
    if (tm) {
        any_destination = true;
        telemetry_write(tm, stream, record, len);
    }

    return any_destination ? ESP_OK : ESP_ERR_NOT_FOUND;
}
