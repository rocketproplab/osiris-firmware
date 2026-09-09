#ifndef DISCRETE_IN_H
#define DISCRETE_IN_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_records.h"

/* ===================================================================
 *  Discrete signal lines from other flight hardware.
 *
 *  A "discrete" here is a single wire whose only job is to change level
 *  when something happens. This component watches such a line, captures
 *  the instant it changes, and records that as a flight event.
 *
 *  PCB v1 brings two of them onto the board, both from the Blue Raven
 *  altimeter:
 *
 *    J8  "staging"   -> GPIO22
 *    J7  "recovery"  -> GPIO23
 *
 *  ---------------- topology: this board is a TAP ----------------
 *
 *  Each Blue Raven output is SPLIT two ways. One branch goes to the
 *  actuation hardware that does the actual work; the other comes here.
 *  We are a passive observer on a live signal -- we listen, we never
 *  drive. Three consequences worth stating explicitly:
 *
 *  1. NEVER drive these pins. They are configured as inputs and must
 *     stay that way. Back-driving a net that is also wired to actuation
 *     hardware is the class of mistake that fires something. The
 *     output_pin field below drives a SEPARATE pin, never this one.
 *
 *  2. The divider is a parallel LOAD on the actuation net. 4.7k + 3.6k
 *     is 8.3k across the signal, drawing roughly 0.9 mA at 7.4 V.
 *     Negligible against anything that fires a charge, but not zero --
 *     worth knowing it is there if the actuation branch is ever
 *     redesigned around a higher-impedance source.
 *
 *  3. The signal's voltage is set by what the ACTUATION side needs, not
 *     by what is convenient for us. That is the best evidence we have
 *     that it sits at battery level, which is what the divider below
 *     assumes -- but it is evidence, not a measurement.
 *
 *  WORTH CHECKING SEPARATELY: if the actuation branch is inductive --
 *  a relay, a solenoid, anything with a coil -- switching it produces a
 *  flyback spike on the shared node. The divider passes 43.4% of
 *  whatever appears there, so a spike above ~7.6 V puts the ESP32-P4
 *  pin over 3.3 V. If there is a coil on that net, it wants a clamp.
 *
 *  ---------------- naming ----------------
 *
 *  The component is named for what it does rather than for who drives
 *  it. That is deliberate: the provenance of these lines has already
 *  been restated twice during development, and a component named after
 *  its data source has to be renamed every time that happens. The
 *  `source` field below records provenance per channel instead, where
 *  it can be corrected without touching a single symbol.
 *
 *  ---------------- electrical ----------------
 *
 *  Both arrive through the same divider -- 4.7k in series, 3.6k to
 *  ground -- so the pin sees the source's output x 0.434, and the 3.6k
 *  leg holds the line LOW whenever the driving output is tri-stated.
 *  No internal pulldown is used on either: R5/R8 already do that job,
 *  and the internal one (~45k) in parallel would shift the ratio to
 *  0.415 and make the trigger HARDER to reach, not safer.
 *
 *  VOLTAGE WINDOW -- STILL UNVERIFIED.
 *  The ESP32-P4 needs about 2.475 V (0.75 x VDD) to read HIGH and must
 *  not exceed 3.3 V, so through x 0.434 the Blue Raven's output has to
 *  sit between roughly 5.7 V and 7.6 V:
 *
 *     3.3 V source -> 1.43 V at the pin -> never fires
 *     5.0 V source -> 2.17 V at the pin -> never fires, or marginally
 *     7.4 V source -> 3.21 V at the pin -> correct
 *    12.0 V source -> 5.21 V at the pin -> over-voltage on the pin
 *
 *  The divider is sized for a 2S pack, which is consistent with the
 *  battery sense divider (6.8k/2.7k, 11.6 V full scale) and with the
 *  fact that these outputs also have to drive actuation hardware. That
 *  is a good argument, not a measurement. MEASURE BOTH LINES BEFORE
 *  FLIGHT. If the Blue Raven turns out to drive a logic-level output,
 *  neither line will ever trigger, and no amount of firmware can detect
 *  that: a signal that never arrives is indistinguishable from one that
 *  has not happened yet.
 *
 *  POLARITY -- confirmed idle LOW, LOW-to-HIGH transition triggers, for
 *  both lines. Each pin is therefore armed on the rising edge.
 *
 *  ---------------- why a pool ----------------
 *
 *  It began as staging.c, built around "there is exactly one staging
 *  line on this vehicle". That stopped being true when a second discrete
 *  arrived with identical handling, so this uses the static pool +
 *  opaque handle idiom the sensor drivers already use. A third discrete
 *  costs a config struct, not another component.
 * =================================================================== */

#define DISCRETE_IN_MAX_INSTANCES   2
#define DISCRETE_IN_STACK_WORDS     3072

/* Output convention, when a channel has an output pin at all. Note this
 * is the OPPOSITE of flight_logic.c's deployment channels, which idle
 * HIGH and drive LOW to fire (the open-drain-to-ground style the Blue
 * Raven uses). PCB v1 has no output pin on either channel, so this
 * never runs today -- it matters only if a later revision adds one. */
#define DISCRETE_IN_OUTPUT_IDLE_LEVEL    0
#define DISCRETE_IN_OUTPUT_ACTIVE_LEVEL  1

typedef struct {
    gpio_num_t   input_pin;    // watched for the LOW-to-HIGH transition
    gpio_num_t   output_pin;   // driven in response; GPIO_NUM_NC = none
    event_type_t event;        // EV_STAGING_SIGNAL or EV_RECOVERY_SIGNAL
    uint8_t      channel;      // index recorded in the event record
    const char  *name;         // "staging" / "recovery", for log lines
    const char  *source;       // which device drives this line, and how it
                                // reaches us. Logged at init so a flight log
                                // records what the firmware believed about
                                // provenance rather than leaving it to a
                                // comment nobody re-reads.
} discrete_in_config_t;

typedef struct {
    uint32_t triggers;         // edges seen on this channel
    uint64_t first_trigger_us; // ISR-captured time of the FIRST edge
} discrete_in_diag_t;

typedef struct discrete_in_dev *discrete_in_handle_t;

/* Claims a pool slot, configures both GPIOs, and arms the input
 * interrupt. Does not start the task yet. */
esp_err_t discrete_in_init(const discrete_in_config_t *cfg, discrete_in_handle_t *out);

/* Spawns the task that sleeps until the input fires, then records the
 * event and drives the output pin if there is one. */
esp_err_t discrete_in_start(discrete_in_handle_t dev, UBaseType_t priority, BaseType_t core_id);

esp_err_t discrete_in_stop(discrete_in_handle_t dev);

/* True once this channel's edge has been seen. */
bool discrete_in_fired(discrete_in_handle_t dev);

void discrete_in_get_diagnostics(discrete_in_handle_t dev, discrete_in_diag_t *out);

#endif // DISCRETE_IN_H
