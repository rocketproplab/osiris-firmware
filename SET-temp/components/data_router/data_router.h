#ifndef DATA_ROUTER_H
#define DATA_ROUTER_H

#include "esp_err.h"
#include "sensor_records.h"
#include "sd_logger.h"
#include "telemetry.h"

/* ===================================================================
 *  One publish call instead of two.
 *
 *  Flight logic used to be a third destination here, but it isn't a
 *  queue consumer at all: deploy_logic_update() wants the CURRENT
 *  sensor state, evaluated every control-loop tick, not a backlog of
 *  discrete records to drain whenever the writer task gets to them.
 *  Feeding it through a ring buffer would mean it could be reacting to
 *  a stale, several-samples-old reading during exactly the moments
 *  (liftoff, apogee) where that lag matters most.
 *
 *  Flight logic now runs as its own dedicated task in main.c, reading
 *  bmp388_get_latest()/lsm6dsox_get_latest() directly -- same shape as
 *  print_task, not the same shape as the log/telemetry bridge tasks.
 *
 *  SD and telemetry stay here since they both genuinely are "queue up
 *  discrete records for something slower to drain later" consumers.
 * =================================================================== */

esp_err_t data_router_init(sd_logger_handle_t sd, telemetry_handle_t telemetry);

/* Stop routing to the SD logger and return the handle that was
 * detached (NULL if there wasn't one). The caller then owns it and is
 * expected to sd_logger_stop() it.
 *
 * The order is the point. sd_logger_stop() flushes, closes and
 * unmounts; a sensor task that publishes in the middle of that would
 * be handing records to a logger whose files are already closed.
 * Detaching first means the window is closed before the teardown
 * starts. sd_logger_write() also guards on its own `initialized` flag,
 * so a task already inside publish() when this runs gets a rejected
 * write rather than anything worse -- but that is the backstop, not
 * the plan.
 *
 * Detaching is one way. There is no re-attach, because there is no
 * remount command for it to pair with. */
sd_logger_handle_t data_router_detach_sd(void);

esp_err_t data_router_publish(sensor_stream_t stream, const void *record, size_t len);

#endif // DATA_ROUTER_H
