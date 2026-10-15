#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "hal/adc_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ===================================================================
 *  Pack voltage monitor -- Osiris Avionics PCB v1.
 *
 *  The battery reaches the ESP32-P4 through a resistive divider:
 *  BT1+ -> R1 6.8k -> node -> R2 2.7k -> GND, then R3 1k into GPIO20
 *  with C2 100nF to ground. GPIO20 is ADC1_CH4.
 *
 *  This is the only stream in the project that is NOT a sensor. It gets
 *  the same treatment anyway -- its own task, its own record type, its
 *  own file on the card -- because the question it answers ("was the
 *  pack sagging when the log went strange?") is only answerable if the
 *  answer survives independently of whatever went strange.
 *
 *  R3 + C2 form a ~100 us RC. That is far faster than the 10 Hz sample
 *  rate, so it is an anti-alias/ESD filter, not a smoother: it does not
 *  remove sample-to-sample noise. Oversampling below does that.
 * =================================================================== */

#define BATTERY_MAX_INSTANCES     1
#define BATTERY_STACK_WORDS       3072

/* Divider ratio for PCB v1: 2.7 / (6.8 + 2.7). The 1k (R3) sits between
 * the divider node and the pin but draws no steady current into a CMOS
 * input, so it does not enter the ratio. */
#define BATTERY_DIVIDER_RATIO_V1  0.2842f

/* How many raw conversions are averaged into one logged sample. The P4's
 * SAR ADC is noisy enough that a single reading wanders by tens of mV;
 * 16 conversions costs microseconds and buys back ~4x in noise. */
#define BATTERY_OVERSAMPLE        16

/* Standby cadence. The pack still needs watching on the pad -- that is
 * where you find out it was sagging before launch -- but at 1 Hz rather
 * than 10, which is a tenth of the ADC work and a tenth of the records.
 * The task keys on flight_mode_is_fast() like every other driver. */
#define BATTERY_STANDBY_RATE_HZ   1

typedef struct {
    float    battery_v;      // reconstructed pack voltage
    float    adc_v;          // calibrated voltage at the pin
    uint64_t timestamp_us;
    uint32_t seq;            // increments once per completed sample
} battery_data_t;

typedef struct {
    adc_channel_t channel;        // ADC_CHANNEL_4 for GPIO20
    float         divider_ratio;  // 0 => BATTERY_DIVIDER_RATIO_V1
    uint32_t      sample_rate_hz; // 0 => 10
    uint8_t       oversample;     // 0 => BATTERY_OVERSAMPLE

    /* Logged-only thresholds: crossing one emits a warning in the log
     * but never changes behaviour. Deciding what to DO about a low pack
     * is flight_logic's business, not a monitor's. 0 disables. */
    float         warn_volts;
    float         critical_volts;

    /* Two-point-style trim applied AFTER the divider:
     *     battery_v = (adc_v / divider_ratio) * gain + offset_v
     * Corrects what the resistor tolerances and the ADC calibration
     * leave over, measured against a multimeter. adc_v is still logged
     * untrimmed, so a later, better calibration can be applied offline
     * to an old log. gain 0 => 1.0. */
    float         gain;
    float         offset_v;

    /* Highest pack voltage this divider must read -- a 2S LiPo at full
     * charge is 8.4 V. Used only for a boot-time check that the divider
     * keeps the pin inside the ADC's range. 0 skips the check. */
    float         max_pack_volts;
} battery_config_t;

typedef struct {
    uint32_t samples;
    uint32_t read_errors;
    uint32_t warn_events;
    uint32_t critical_events;
    float    min_volts;      // lowest pack voltage seen since start
    uint32_t clipped;        // samples at the ADC's full scale: the pack
                             // voltage is HIGHER than reported, by an
                             // unknown amount -- divider ratio too large
} battery_diag_t;

/* Usable top of the ADC at 12 dB attenuation, as a pin voltage. The same
 * figure the uncalibrated fallback scales by. Above this the reading is
 * pinned and the reported pack voltage is a floor, not a measurement. */
#define BATTERY_ADC_FULL_SCALE_V  3.1f

typedef struct battery_dev *battery_handle_t;

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

/* Claims a pool slot, brings up ADC1 in oneshot mode on the configured
 * channel at 12 dB attenuation, and installs curve-fitting calibration
 * if the chip has the fuses for it. Does not start the task yet.
 *
 * 12 dB is chosen for headroom, not for range: a 2S pack at 8.4 V lands
 * at 2.39 V through the divider, which the 11 dB-equivalent range covers
 * with room left for a 3S pack (11.6 V full scale) if this board ever
 * flies one. */
esp_err_t battery_init(const battery_config_t *cfg, battery_handle_t *out);

/* Spawns the sampling task. It publishes each sample to the data router
 * as STREAM_BATT, which is what lands it in batt.bin. */
esp_err_t battery_start(battery_handle_t dev, UBaseType_t priority, BaseType_t core_id);

esp_err_t battery_stop(battery_handle_t dev);

/* ===================================================================
 *  Data access
 * =================================================================== */

void battery_get_latest(battery_handle_t dev, battery_data_t *out);

/* One synchronous conversion, outside the task's cadence. Useful at
 * startup to log the pack voltage before anything else has run. */
esp_err_t battery_read_once(battery_handle_t dev, battery_data_t *out);

void battery_get_diagnostics(battery_handle_t dev, battery_diag_t *out);

#endif // BATTERY_MONITOR_H
