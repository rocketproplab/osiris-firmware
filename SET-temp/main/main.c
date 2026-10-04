#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"


#include "sensor_bus.h"
#include "bmp388_sensor.h"
#include "lsm6dsox_sensor.h"
#include "gps_sensor.h"
#include "sd_logger.h"
#include "telemetry.h"
#include "flight_logic.h"
#include "data_router.h"
#include "config_receiver.h"
#include "nvs_flash.h"
#include "discrete_in.h"
#include "event_log.h"
#include "flight_mode.h"
#include "battery_monitor.h"

/* ===================================================================
 *  Task priorities
 *
 *  Higher number = higher priority. FreeRTOS runs the highest-priority
 *  READY task; equal priorities round-robin.
 *
 *  Defined here rather than pulled from menuconfig -- change a number,
 *  rebuild. The _Static_asserts below still enforce the ordering, so a
 *  typo that inverts two of these is a build error rather than a stall
 *  that only shows up in flight.
 * =================================================================== */

/* Tasks woken by the two Blue Raven discrete inputs -- staging (J8) and
 * recovery (J7). Both share this priority: neither is more urgent than
 * the other, and equal priorities round-robin. NOTE this is the TASK
 * priority. The ISR itself preempts every task listed here regardless,
 * because hardware interrupts are a different scale entirely (levels
 * 1-3, set via ESP_INTR_FLAG_LEVELn when the handler is installed). */
#define PRIO_DISCRETE      11

/* All three sensor readers share this. They sleep on an interrupt
 * notification and do a short SPI/UART read on wake -- prompt, but
 * brief. Sharing one priority is deliberate: none is more urgent than
 * the others, and equal priorities round-robin, which spreads CPU
 * fairly when two fire close together. */
#define PRIO_SENSORS       9

/* Consumes sensor data and decides on deployment events. Below the
 * sensors on purpose: it can only act on data they've already
 * produced, so starving a sensor to run this sooner would just make it
 * reason about staler numbers. */
#define PRIO_DEPLOY        7

/* XBee downlink. Above the SD writer because telemetry is the backup
 * record -- if the card is lost or corrupted, whatever got transmitted
 * is all that survives. A radio frame also has a hard deadline that a
 * disk write does not: miss the slot and the data is gone, whereas a
 * delayed SD write still lands. */
#define PRIO_TELEMETRY     5

/* Drains the log queue to the card. Lowest of the five despite SD
 * being the primary storage -- this is about latency tolerance, not
 * importance. SD writes stall unpredictably (block erase, wear
 * levelling) for tens of milliseconds, and a stall at a higher
 * priority would block genuinely time-critical work. The queue in
 * front of this task is what makes the low priority safe.
 *
 * If sd_logger_diag_t.queue_full_drops climbs, raise
 * SD_LOGGER_QUEUE_DEPTH before raising this. */
#define PRIO_SD_WRITER     3

/* Pack-voltage housekeeping. Below the SD writer because it is the only
 * task here whose data nothing else waits on: a battery sample arriving
 * a few ms late costs nothing, whereas delaying the writer costs queue
 * headroom. Still above print, because it produces flight record and
 * print produces console noise. */
/* Battery sampling and the status snapshot share this tier: both are
 * periodic housekeeping that nothing else waits on. */
#define PRIO_BATTERY       2
#define PRIO_STATUS        2

/* Debug console output only. Deliberately below everything above so a
 * printf can never delay flight work. */
#define PRIO_PRINT         1

/* ===================================================================
 *  Ordering guards -- a wrong priority becomes a build error, not a
 *  flight-time stall.
 * =================================================================== */

_Static_assert(PRIO_DISCRETE > PRIO_SENSORS,
    "Blue Raven discretes must outrank sensors: a staging or recovery event cannot wait behind a sensor read.");
_Static_assert(PRIO_SENSORS > PRIO_DEPLOY,
    "Sensors must outrank deployment logic, which consumes what they produce.");
_Static_assert(PRIO_DEPLOY > PRIO_TELEMETRY,
    "Deployment logic must outrank telemetry.");
_Static_assert(PRIO_TELEMETRY > PRIO_SD_WRITER,
    "Telemetry must outrank the SD writer: SD stalls must never delay a radio slot.");
_Static_assert(PRIO_SD_WRITER > PRIO_BATTERY,
    "The SD writer must outrank battery sampling: draining the queue "
    "matters more than adding one more record to it.");
_Static_assert(PRIO_BATTERY > PRIO_PRINT,
    "Battery sampling produces flight record; printing does not.");
_Static_assert(PRIO_SD_WRITER > PRIO_PRINT,
    "Debug printing must stay below all flight work.");
_Static_assert(PRIO_PRINT > 0,
    "Priority 0 is the idle task -- no application task may share it.");

static const char *TAG = "main";

/* ===================================================================
 *  Pin map -- SET I "Osiris Avionics" PCB v1 (SET_AV.kicad_pcb, 2026-09-03)
 *
 *  Taken from the board netlist, not guessed. "J1.n" is the footprint
 *  pad number on the ESP32-P4-ETH header; the GPIO number after it is
 *  what that pad actually is on the Waveshare module. The module is
 *  placed rotated 180 degrees relative to Waveshare's pinout drawing,
 *  so pad 1 is at the bottom of the board and pad 21 at the top.
 *
 *  Free header pads if you ever need to bodge a signal out: pad 22
 *  (GPIO47), pad 24 (GPIO46), pad 34 (GPIO21), pad 21 (GPIO48).
 *  Pads 16/17 (GPIO8/GPIO7) and 19/20 (GPIO25/GPIO24) are the module's
 *  internal I2C and USB -- leave those alone.
 * =================================================================== */

/* One bus shared by the barometer (J3) and the IMU (J4). Whether those
 * wires run as SPI or as I2C is a build-time choice made in menuconfig
 * (SET sensor bus -> Interface) and owned entirely by the sensor_bus
 * component -- it holds its own copy of these five pin numbers and sets
 * them up itself. main.c does not open the bus and does not pass pins,
 * hosts or clock rates to either driver.
 *
 * The five bus pins are listed here anyway, commented out, because this
 * block is the board's pin map and a reader looking for GPIO54 should
 * find it here. If the board is ever rewired, the numbers that matter
 * are the ones at the top of components/sensor_bus/sensor_bus.c.
 *
 *   #define PIN_SCLK        GPIO_NUM_54   J1.1  -> J3.4 SCL, J4.4 SCL
 *   #define PIN_MOSI        GPIO_NUM_19   J1.2  -> J3.6 SDA, J4.5 SDA
 *   #define PIN_MISO        GPIO_NUM_18   J1.4  -> J3.5 SDO, J4.6 DO
 *                                          (I2C build: address select, driven LOW)
 *   #define PIN_CS_BMP388   GPIO_NUM_14   J1.9  -> J3.7 CS
 *   #define PIN_CS_LSM6DSOX GPIO_NUM_17   J1.5  -> J4.7 CS
 *
 * The interrupt pins below are NOT bus pins -- each driver arms its own,
 * so they stay here and stay passed in through config. */
#define PIN_INT_BMP388    GPIO_NUM_6   /* J1.10 -> J3.8 INT */
#define PIN_INT_LSM6DSOX  GPIO_NUM_16  /* J1.6  -> J4.8 I1  */

/* J4.9 (I2, the LSM6DSOX's second interrupt) is wired to GPIO15 on
 * J1.7. The driver routes its hardware wake-up (liftoff) block to this
 * line -- see lsm6dsox_liftoff_detected(). */
#define PIN_INT2_LSM6DSOX GPIO_NUM_15  /* J1.7  -> J4.9 I2, liftoff */

/* GPS (J6, Adafruit PA1616D on JST-XH). TX/RX are named from the
 * ESP32's point of view: PIN_TX_GPS drives the module's RX input.
 * Plain ints, not gpio_num_t, to match uart_set_pin()'s signature. */
#define GPS_UART_PORT     1              /* UART_NUM_1 */
#define PIN_TX_GPS        27             /* J1.27 -> J6.4 RX  */
#define PIN_RX_GPS        32             /* J1.26 -> J6.5 TX  */
#define PIN_PPS_GPS       GPIO_NUM_26    /* J1.29 -> J6.1 PPS */
#define PIN_EN_GPS        GPIO_NUM_33    /* J1.25 -> J6.6 EN  */

/* XBee PRO S3B (J5). Same TX/RX convention as the GPS. R10 is a 10k
 * pull-up from 3V3 onto PIN_TX_XBEE, which holds the XBee's DIN at the
 * UART idle level while the ESP32 pin is still high-Z during boot. */
#define XBEE_UART_PORT    2              /* UART_NUM_2 */
#define PIN_TX_XBEE       3              /* J1.14 -> J5.4 DIN  */
#define PIN_RX_XBEE       4              /* J1.12 -> J5.3 DOUT */
#define PIN_RST_XBEE      GPIO_NUM_2     /* J1.15 -> J5.6 RST  */
#define PIN_CTS_XBEE      GPIO_NUM_5     /* J1.11 -> J5.14 CTS */
#define XBEE_BAUD_RATE    115200         /* must match the module's ATBD */

/* Blue Raven signal inputs. Both arrive through the same divider:
 * 4.7k in series, 3.6k to ground (J8/R6/R5 for staging, J7/R9/R8 for
 * recovery), so the pin sees the BR's output x 0.434.
 *
 * That divider only produces a valid logic HIGH for a source between
 * roughly 5.7 V and 7.6 V -- see the note in staging.h. The 3.6k leg
 * also means both lines are held LOW by hardware whenever the Blue
 * Raven's output is tri-stated, which is why no internal pulldown is
 * used on either pin. */
#define PIN_STAGING_IN    GPIO_NUM_22    /* J1.32 <- J8.2, Blue Raven staging  */
#define PIN_RECOVERY_IN   GPIO_NUM_23    /* J1.31 <- J7.2, Blue Raven recovery */

/* Battery sense: BT1+ through 6.8k/2.7k (R1/R2) = x 0.284, then 1k
 * (R3) into the pin with 100nF (C2) to ground. GPIO20 is ADC1_CH4.
 * 3.3 V full scale corresponds to 11.6 V at the battery. */
#define PIN_VBAT_SENSE    GPIO_NUM_20    /* J1.35, ADC1_CH4 */
#define VBAT_DIVIDER_RATIO  0.2842f      /* 2.7 / (6.8 + 2.7) */
#define VBAT_ADC_CHANNEL    ADC_CHANNEL_4 /* GPIO20 on ADC1 -- fixed by silicon */
#define VBAT_SAMPLE_RATE_HZ 10

/* Logged-only thresholds, sized for a 2S LiPo: 3.6 V/cell is the usual
 * "land now" point and 3.3 V/cell is where cell damage starts. Crossing
 * either writes a log line and bumps a counter; neither changes what the
 * vehicle does. */
#define VBAT_WARN_VOLTS     7.2f
#define VBAT_CRITICAL_VOLTS 6.6f

/* Staging OUTPUT: this board does not have one. J8 is a two-pin input
 * connector -- pin 2 is the signal through the divider above, pin 1 is
 * tied to ground through R4. There is no pad the firmware could drive
 * in response, so this is GPIO_NUM_NC and staging_init() skips output
 * configuration entirely. If a future board revision adds one, point
 * this at it and nothing else needs to change. */
#define PIN_STAGING_OUT   GPIO_NUM_NC

/* No SD pin defines here. Unlike the sensors above, the SD card plugs
 * directly into the ESP32-P4's fixed SDMMC slot -- there's no wiring
 * choice for main.c to make, so those pins live as private constants
 * inside sd_logger.c instead of being passed in through config. */

/* ===================================================================
 *  Sensor handles
 *
 *  static, not a local in app_main(): app_main() returns once its
 *  startup work is done, but print_task() keeps running and needs
 *  these handles for the rest of the program's life. A stack local in
 *  app_main() would be gone the moment app_main() returned, leaving
 *  print_task() holding a dangling pointer to freed stack space.
 * =================================================================== */

typedef struct {
    bmp388_handle_t   baro;
    lsm6dsox_handle_t imu;
    gps_handle_t      gps;
    sd_logger_handle_t sd;
    telemetry_handle_t telemetry;
    battery_handle_t   battery;
    discrete_in_handle_t discrete[2]; /* 0 = staging (J8), 1 = recovery (J7) */
} sensor_handles_t;

static sensor_handles_t s_sensors;

/* Flight logic's config is a plain struct, not an opaque handle -- it
 * has no hardware to hide behind one, just JSON-loaded thresholds and
 * per-channel runtime state. static for the same reason as s_sensors:
 * flight_logic_task() needs it to outlive app_main()'s return. */
static deploy_config_t s_deploy_cfg;

/* ===================================================================
 *  Static allocation for main.c's own tasks
 *
 *  Every sensor driver's background task uses static stack/TCB storage
 *  embedded in its own struct -- no heap involved. The logging tasks
 *  and print_task were the exceptions, originally created with plain
 *  xTaskCreatePinnedToCore() (heap-allocated stack + TCB via
 *  pvPortMalloc). That inconsistency is fixed here: same static
 *  pattern, just declared directly in main.c since these tasks don't
 *  belong to any one driver's struct.
 * =================================================================== */

/* BARO/IMU/GPS_LOG_STACK_WORDS removed along with their tasks -- the
 * router push now happens inside each sensor's own driver task. */
#define FLIGHT_LOGIC_TASK_STACK_WORDS 4096
#define PRINT_TASK_STACK_WORDS     4096

static StaticTask_t s_flight_logic_tcb;
static StackType_t  s_flight_logic_stack[FLIGHT_LOGIC_TASK_STACK_WORDS];

static StaticTask_t s_print_tcb;
static StackType_t  s_print_stack[PRINT_TASK_STACK_WORDS];

#define STATUS_TASK_STACK_WORDS  3072
static StaticTask_t s_status_tcb;
static StackType_t  s_status_stack[STATUS_TASK_STACK_WORDS];

#ifdef CONFIG_FC_MODE_TEST
/* ===================================================================
 *  TEST build: scripted mode sequence (menuconfig -> Flight Computer
 *  Operating Mode). One step every CONFIG_FC_TEST_STEP_S seconds,
 *  counted from when this task starts, i.e. once everything else is up:
 *
 *      STANDBY -> FLIGHT -> STANDBY -> FLIGHT -> STANDBY (and stays)
 *
 *  Uses flight_mode_test_force(), the only path out of FLIGHT, and it
 *  exists only in this build. The task deletes itself when done.
 * =================================================================== */
#define TEST_SEQ_STACK_WORDS  2048
static StaticTask_t s_test_seq_tcb;
static StackType_t  s_test_seq_stack[TEST_SEQ_STACK_WORDS];

static void test_sequence_task(void *arg)
{
    (void)arg;
    static const flight_mode_t steps[] = {
        MODE_FLIGHT, MODE_STANDBY, MODE_FLIGHT, MODE_STANDBY,
    };
    const int n = (int)(sizeof(steps) / sizeof(steps[0]));
    const TickType_t step = pdMS_TO_TICKS(CONFIG_FC_TEST_STEP_S * 1000);

    /* DelayUntil, not Delay: the steps stay exactly step apart however
     * long the logging inside the loop takes. */
    TickType_t last = xTaskGetTickCount();
    for (int i = 0; i < n; i++) {
        vTaskDelayUntil(&last, step);
        ESP_LOGW(TAG, "TEST step %d/%d -> %s", i + 1, n, flight_mode_name(steps[i]));
        flight_mode_test_force(steps[i]);
    }
    ESP_LOGW(TAG, "TEST sequence complete -- staying in STANDBY");
    vTaskDelete(NULL);
}
#endif /* CONFIG_FC_MODE_TEST */

/* ===================================================================
 *  BMP388 bring-up
 * =================================================================== */

static esp_err_t baro_start(bmp388_handle_t *out)
{
    /* No .host / .cs_pin / .spi_freq_hz: sensor_bus owns the bus, its
     * pins and its clock. Only this sensor's own interrupt line is
     * main.c's to hand over. */
    bmp388_config_t cfg = {
        .int_pin         = PIN_INT_BMP388,

        /* 50 Hz with 8x pressure / 2x temperature oversampling -- Bosch's
         * suggested "drone" profile. Temperature changes slowly and only
         * feeds the pressure math, so spending oversampling on it buys
         * little. */
        .osr_pressure    = BMP3_OVERSAMPLING_8X,
        .osr_temp        = BMP3_OVERSAMPLING_2X,
        .odr             = BMP3_ODR_25_HZ,

        /* IIR smooths pressure noise that would otherwise show up as fake
         * vertical velocity. Coefficient 3 is a reasonable start; higher
         * is smoother but adds lag -- worth characterizing on the bench
         * if this ever drives an apogee decision. */
        .iir_filter      = BMP3_IIR_FILTER_COEFF_3,

        .ref_pressure_pa = 0.0f,   /* 0 => sea level until bmp388_zero_altitude() */
    };

    bmp388_handle_t baro = NULL;
    esp_err_t err = bmp388_init(&cfg, &baro);
    if (err != ESP_OK) {
        return err;
    }

    /* Priority 10, pinned to core 0. Pinning matters: an unpinned task can
     * migrate between cores, and you want the sensor task's timing
     * predictable rather than at the mercy of the scheduler. */
    err = bmp388_start(baro, PRIO_SENSORS, 0);
    if (err != ESP_OK) {
        bmp388_stop(baro);
        return err;
    }

    *out = baro;
    return ESP_OK;
}

/* ===================================================================
 *  LSM6DSOX bring-up
 * =================================================================== */

static esp_err_t imu_start(lsm6dsox_handle_t *out)
{
    /* No .host / .cs_pin / .spi_freq_hz -- see the note in baro_start(). */
    lsm6dsox_config_t cfg = {
        .int_pin     = PIN_INT_LSM6DSOX,
        .int2_pin    = PIN_INT2_LSM6DSOX,

        /* Hardware liftoff detect on INT2. Deliberately a different
         * number from flight_logic.c's LIFTOFF_ACCEL_THRESHOLD_MS2
         * (20 m/s^2 = ~2 g): these are two independent detectors, and
         * tuning them to agree exactly would waste the independence
         * that makes having both worth anything. */
        .liftoff_threshold_g = 3.0f,
        .liftoff_dur_samples = 2,
    };

    lsm6dsox_handle_t imu = NULL;
    esp_err_t err = lsm6dsox_init(&cfg, &imu);
    if (err != ESP_OK) {
        return err;
    }

    /* Same priority tier as the barometer, also pinned to core 0 -- both
     * sensor tasks are short interrupt-driven reads, not compute-heavy,
     * so sharing the core with the barometer is fine. If profiling later
     * shows contention, that's the moment to consider splitting them
     * across cores. */
    err = lsm6dsox_start(imu, PRIO_SENSORS, 0);
    if (err != ESP_OK) {
        lsm6dsox_stop(imu);
        return err;
    }

    *out = imu;
    return ESP_OK;
}

/* ===================================================================
 *  GPS bring-up
 * =================================================================== */

static esp_err_t gps_start_sensor(gps_handle_t *out)
{
    gps_config_t cfg = {
        .uart_port = GPS_UART_PORT,
        .tx_pin    = PIN_TX_GPS,
        .rx_pin    = PIN_RX_GPS,
        .pps_pin   = PIN_PPS_GPS,
        .en_pin    = PIN_EN_GPS,
    };

    gps_handle_t gps = NULL;
    esp_err_t err = gps_init(&cfg, &gps);
    if (err != ESP_OK) {
        return err;
    }

    /* Lower priority than the bus sensor tasks -- GPS fixes arrive at
     * roughly 1 Hz, nowhere near as time-critical as the barometer or
     * IMU, so it doesn't need to preempt them. */
    err = gps_start(gps, PRIO_SENSORS, 0);
    if (err != ESP_OK) {
        gps_stop(gps);
        return err;
    }

    *out = gps;
    return ESP_OK;
}

/* ===================================================================
 *  SD logger bring-up
 * =================================================================== */

static esp_err_t sd_start(sd_logger_handle_t *out)
{
    sd_logger_config_t cfg = {
        .ldo_chan_id            = 4,      /* LDO_VO4 -- ESP32-P4 SDMMC IO rail */
        .mount_point            = "/sdcard",
        .max_freq_khz           = 0,      /* 0 => SDMMC_FREQ_DEFAULT (20 MHz).
                                           * Bump to SDMMC_FREQ_HIGHSPEED (40)
                                           * only after confirming your trace
                                           * lengths and card can take it. */
        .format_if_mount_failed = false,  /* false on purpose: silently
                                           * reformatting a card that failed
                                           * to mount would destroy a previous
                                           * flight's data. Format it
                                           * deliberately on a PC instead. */
    };

    sd_logger_handle_t sd = NULL;
    esp_err_t err = sd_logger_init(&cfg, &sd);
    if (err != ESP_OK) {
        return err;
    }

    /* Priority 4: below every sensor task (10) and below print (5).
     * SD writes are the slowest, least time-critical work here -- a
     * delayed log line costs nothing, a delayed sensor read costs data.
     * Pinned to core 1 to keep the blocking filesystem work off core 0
     * where the two interrupt-driven sensor tasks live. */
    err = sd_logger_start(sd, PRIO_SD_WRITER, 1);
    if (err != ESP_OK) {
        sd_logger_stop(sd);
        return err;
    }

    *out = sd;
    return ESP_OK;
}

/* ===================================================================
 *  Telemetry bring-up
 *
 *  This comment used to say "queue-only scaffolding, see the TODO on
 *  actual XBee transmission". That has not been true since the packet
 *  path was written: telemetry builds real 0x10 Transmit Request frames
 *  and writes them to the UART, and the host suite round-trips them.
 *
 *  telemetry_init() also resets the module and proves a radio answers
 *  before returning -- see the bring-up note in telemetry.h. It does not
 *  fail if the radio is deaf; it logs, sets diag.radio_verified false,
 *  and lets the vehicle fly on the SD card. Started at PRIO_TELEMETRY,
 *  the tier the ordering guards at the top of this file reserve for it.
 * =================================================================== */

static esp_err_t telem_start(telemetry_handle_t *out)
{
    telemetry_config_t cfg = {
        .uart_port      = XBEE_UART_PORT,
        .tx_pin         = PIN_TX_XBEE,
        .rx_pin         = PIN_RX_XBEE,
        /* Wired but unused until now. It is what lets the firmware give
         * the module a clean boot with DIN already idling high, instead
         * of hoping it never lands in its bootloader. */
        .rst_pin        = PIN_RST_XBEE,
        .baud_rate      = XBEE_BAUD_RATE,
        /* Broadcast. Set this to the ground station's 64-bit address
         * once it is known: a broadcast gets no per-packet delivery
         * ack on DigiMesh, which means the retry path has nothing to
         * trigger on. Point-to-point is what makes retry work. */
        .dest_addr      = 0,
        .imu_decimation = TELEMETRY_IMU_DECIMATION,
    };

    telemetry_handle_t telem = NULL;
    esp_err_t err = telemetry_init(&cfg, &telem);
    if (err != ESP_OK) {
        return err;
    }

    err = telemetry_start(telem, PRIO_TELEMETRY, 1);
    if (err != ESP_OK) {
        telemetry_stop(telem);
        return err;
    }

    *out = telem;
    return ESP_OK;
}

/* ===================================================================
 *  Flight logic bring-up
 *
 *  Unlike SD/telemetry, this isn't a queue-consuming subsystem -- see
 *  the comment at the top of data_router.h for why. deploy_logic_load_config()
 *  never fails outward (it falls back to safe, all-disabled defaults on
 *  any NVS/parse problem -- check the logs to tell defaults from a real
 *  loaded config), so this only returns non-OK if GPIO init itself fails.
 * =================================================================== */

static esp_err_t deploy_logic_bring_up(void)
{
    /* Deployment config is read from flash -- NVS has to be initialized
     * exactly once, before anything tries to open a namespace. This is
     * the standard ESP-IDF erase-and-retry pattern: a fresh chip or one
     * whose NVS partition layout changed reports ESP_ERR_NVS_NO_FREE_PAGES
     * or ESP_ERR_NVS_NEW_VERSION_FOUND on first init, and the documented
     * fix is to erase and initialize once more. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erasing (0x%x) -- erasing and retrying", err);
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Always succeeds outward -- see the doc comment on this function in
     * flight_logic.h for what happens when there's nothing usable in NVS. */
    deploy_logic_load_config(&s_deploy_cfg);

    return deploy_logic_init(&s_deploy_cfg);
}

/* ===================================================================
 *  Flight logic task
 *
 *  Deliberately NOT shaped like baro_log_task/imu_log_task: this is a
 *  synchronous control loop, not a queue producer. It blocks on the
 *  barometer (the fastest safety-relevant signal for liftoff/apogee),
 *  grabs whatever the IMU's latest reading is, and evaluates
 *  deploy_logic_update() against the CURRENT state every time -- never
 *  against a backlog of old samples.
 * =================================================================== */

static void flight_logic_task(void *arg)
{
    /* ===================================================================
     *  PACED BY THE IMU, NOT THE BAROMETER.
     *
     *  The loop blocks on the LSM6DSOX and takes whatever barometric
     *  sample happens to be current. That is the opposite of how this
     *  task used to work, and the reason is the gyro.
     *
     *  Tilt is computed by integrating angular rate. An integrator is
     *  only as good as its sampling: pacing on the 50 Hz barometer meant
     *  point-sampling a 104 Hz gyro stream at a rate that is neither
     *  synchronous with it nor an integer division of it, so some gyro
     *  samples were read twice and others never read at all. The
     *  rotation during a skipped sample was silently attributed to a
     *  neighbouring rate. For smooth body rates that is a bounded phase
     *  error; for coning -- a spin coupled with a transverse
     *  oscillation, which is what a marginally stable airframe does --
     *  it is a systematic attitude error that accumulates.
     *
     *  Reading every gyro sample removes that class of error entirely.
     *
     *  THE BAROMETER IS NOW THE OVERSAMPLED ONE, and that matters more
     *  than it first appears. At 104 Hz against a 50 Hz barometer, half
     *  the ticks carry an altitude identical to the previous tick. A
     *  naive (alt - prev_alt)/dt would then read zero on those ticks and
     *  double on the others -- a velocity estimate alternating between
     *  0 and 2x truth. deploy_logic_update() therefore keeps two
     *  timelines: everything derived from the gyro and accelerometer
     *  advances every tick, and everything derived from the barometer
     *  advances only when baro.seq changes, on the barometer's own
     *  interval. See the block comment in flight_logic.c.
     * =================================================================== */
    sensor_handles_t *h = (sensor_handles_t *)arg;
    uint32_t last = 0;

    for (;;) {
        /* ---------------------------------------------------------------
         *  Applying an uploaded config.
         *
         *  Done HERE, by the task that reads s_deploy_cfg, rather than
         *  by whoever received the command. deploy_logic_update() is
         *  handed a pointer to this struct on every tick; another task
         *  overwriting it would be rewriting deployment thresholds and
         *  per-channel arm state underneath a running estimator, and
         *  there is no moment in that struct's layout where a partial
         *  overwrite is harmless. At the top of this loop, between
         *  ticks, there is no read in flight.
         *
         *  telemetry has already refused the command unless the vehicle
         *  is in STANDBY or LANDED, so this cannot run during a flight.
         * --------------------------------------------------------------- */
        if (h->telemetry && telemetry_take_config_apply_request(h->telemetry)) {
            esp_err_t cerr = deploy_logic_load_config(&s_deploy_cfg);
            deploy_logic_init(&s_deploy_cfg);
            event_log_event(EV_CONFIG_APPLIED, (uint8_t)flight_mode_get(),
                            (uint32_t)cerr);
            ESP_LOGW(TAG, "deployment config re-loaded from NVS and applied");
        }

        lsm6_data_t imu;
        if (lsm6dsox_wait_for_sample(h->imu, last, &imu, pdMS_TO_TICKS(200)) != ESP_OK) {
            continue;
        }
        last = imu.seq;

        /* Latest available, possibly the same one as last tick. The
         * estimator detects that by sequence number, not by comparing
         * altitudes -- two genuinely identical readings are ordinary. */
        bmp388_data_t baro;
        bmp388_get_latest(h->baro, &baro);

        uint32_t now_ms = (uint32_t)(imu.timestamp_us / 1000ULL);
        deploy_logic_update(&s_deploy_cfg, &baro, &imu, now_ms);
    }
}

/* ===================================================================
 *  Logging task -- pulls new samples from each sensor and hands them
 *  to the SD logger.
 *
 *  Deliberately separate from print_task: printing is a debug
 *  convenience, logging is flight data. Tying them together would mean
 *  a change to console output could quietly change what lands on the
 *  card.
 * =================================================================== */

/* ===================================================================
 *  Logging bridge tasks -- REMOVED.
 *
 *  baro_log_task/imu_log_task/gps_log_task used to exist here, each
 *  blocking on one sensor's wait_for_sample()/wait_for_fix() and then
 *  calling data_router_publish(). That push now happens directly
 *  inside bmp388_task/lsm6dsox_task/gps_task themselves, right where
 *  each sample is already built -- eliminating both these three
 *  separate tasks AND the extra lock-and-copy round trip they required
 *  (wait_for_sample() had to re-fetch under lock what the sensor task
 *  had just published).
 *
 *  The trade-off: bmp388_sensor.c/lsm6dsox_sensor.c/gps_sensor.c are no
 *  longer SD/telemetry-agnostic -- each now includes data_router.h and
 *  can't be compiled or reused without the router and everything it
 *  depends on. That coupling was discussed and deliberately accepted;
 *  see the comment block in data_router.h for the reasoning.
 * =================================================================== */

/* ===================================================================
 *  Consumer task -- this is where your flight logic would go
 * =================================================================== */

static void print_task(void *arg)
{
    sensor_handles_t *h = (sensor_handles_t *)arg;
    bmp388_data_t baro_data;
    lsm6_data_t   imu;
    gps_data_t    gps;
    uint32_t baro_last_seq = 0;
    uint32_t imu_last_seq  = 0;
    uint32_t gps_last_seq  = 0;
    uint64_t baro_last_us  = 0;

    /* Consecutive prints in a row where the barometer produced nothing
     * new. Unlike GPS/IMU, a stale barometer for more than a couple of
     * ticks is worth a diagnostic dump, not just a "(stale)" tag --
     * see the comment further down for why. */
    int baro_stale_streak = 0;

    /* The IMU's hardware liftoff latch never clears, so this tracks
     * whether it has already been announced. */
    bool liftoff_reported = false;

    /* Give the sensors a moment to produce their first samples, then treat
     * the pad pressure as zero altitude. */
    vTaskDelay(pdMS_TO_TICKS(500));
    bmp388_zero_altitude(h->baro);

    for (;;) {
        /* Fixed 500ms cadence, decoupled from any one sensor's rate --
         * this replaced blocking on bmp388_wait_for_sample(), which
         * meant IMU/GPS were only ever checked whenever the barometer
         * happened to produce a sample. Same fix already applied to
         * baro_log_task/imu_log_task/gps_log_task; print_task just
         * hadn't been updated yet. All three sensors are now peeked
         * non-blocking and uniformly on this task's own schedule. */
        vTaskDelay(pdMS_TO_TICKS(500));

        bmp388_get_latest(h->baro, &baro_data);
        bool baro_is_new = (baro_data.seq != baro_last_seq) && (baro_data.seq != 0);
        uint32_t dt_us = (uint32_t)(baro_data.timestamp_us - baro_last_us);
        if (baro_is_new) {
            baro_last_us  = baro_data.timestamp_us;
            baro_last_seq = baro_data.seq;
            baro_stale_streak = 0;
        } else {
            baro_stale_streak++;
        }

        lsm6dsox_get_latest(h->imu, &imu);
        bool imu_is_new = (imu.seq != imu_last_seq);
        imu_last_seq = imu.seq;

        bool have_fix = (gps_get_latest(h->gps, &gps) == ESP_OK);
        bool gps_is_new = have_fix && (gps.seq != gps_last_seq);
        if (have_fix) {
            gps_last_seq = gps.seq;
        }

        /* Two ticks (~1 second) of a stale barometer is worth real
         * suspicion -- at 50 Hz it should have produced ~25 fresh
         * samples in that window. This is the same diagnostic dump the
         * old blocking design gave you automatically on timeout, just
         * triggered by "hasn't moved in a while" instead of "a single
         * wait timed out," since this loop no longer waits on it at all. */
        if (baro_stale_streak >= 2) {
            bmp388_diag_t diag;
            bmp388_get_diagnostics(h->baro, &diag);
            ESP_LOGW(TAG, "BMP388: no new sample in >=1s "
                          "(irq=%lu timeouts=%lu errors=%lu)",
                     (unsigned long)diag.irq_count,
                     (unsigned long)diag.irq_timeouts,
                     (unsigned long)diag.read_errors);
        }

        printf("[%8llu us] T=%6.2f C  P=%9.1f Pa  alt=%7.2f m  "
               "dt=%5lu us  seq=%lu %s\n",
               (unsigned long long)baro_data.timestamp_us,
               baro_data.temperature_c,
               baro_data.pressure_pa,
               baro_data.altitude_m,
               (unsigned long)dt_us,
               (unsigned long)baro_data.seq,
               baro_data.seq == 0 ? "(no data yet)" : (baro_is_new ? "(new)" : "(stale)"));

        if (imu_is_new || imu.seq != 0) {
            printf("             IMU: a=(%6.2f,%6.2f,%6.2f) m/s^2  "
                   "g=(%7.2f,%7.2f,%7.2f) dps  seq=%lu %s\n",
                   imu.ax, imu.ay, imu.az,
                   imu.gx, imu.gy, imu.gz,
                   (unsigned long)imu.seq,
                   imu_is_new ? "(new)" : "(stale)");
        }

        if (have_fix) {
            printf("             GPS: lat=%.6f lon=%.6f alt=%.1fm "
                   "sats=%u fixQ=%u %s seq=%lu\n",
                   gps.latitude, gps.longitude, gps.altitude_m,
                   gps.satellites, gps.fix_quality,
                   gps_is_new ? "(new)" : "(stale)",
                   (unsigned long)gps.seq);
        } else {
            /* The counters say WHY there is no fix, which "no fix yet"
             * alone cannot:
             *   all zero            -> no bytes at all: wiring/power
             *   overflows climbing  -> bytes but no line ends: wrong baud
             *   GGA climbing        -> sentences arrive, module has no fix
             *   PPS climbing        -> the module itself HAS a fix */
            gps_diag_t gd = {0};
            gps_get_diagnostics(h->gps, &gd);
            printf("             GPS: no fix yet  (GGA seen=%lu  errors=%lu  "
                   "overflows=%lu  PPS=%lu)\n",
                   (unsigned long)gd.sentences_parsed,
                   (unsigned long)gd.parse_errors,
                   (unsigned long)gd.line_overflows,
                   (unsigned long)gd.pps_pulses);
        }

        if (h->battery) {
            battery_data_t batt;
            battery_get_latest(h->battery, &batt);
            if (batt.seq != 0) {
                printf("             BATT: %5.2f V  (pin %4.2f V)  seq=%lu\n",
                       batt.battery_v, batt.adc_v, (unsigned long)batt.seq);
            }
        }

        /* Printed once, on the transition. The hardware detector latches,
         * so without this guard it would print on every pass for the
         * remainder of the flight. */
        lsm6dsox_liftoff_t lift;
        lsm6dsox_get_liftoff(h->imu, &lift);
        if (lift.detected && !liftoff_reported) {
            liftoff_reported = true;
            /* Carries the ISR-captured edge time, not the time this
             * 2 Hz loop noticed it, so it stays comparable with
             * EV_LIFTOFF_SW from flight_logic. */
            event_log_event_at(EV_LIFTOFF_HW, 0, lift.count, lift.timestamp_us);
            printf("             *** HARDWARE LIFTOFF at %llu us (INT2, edge #%lu) ***\n",
                   (unsigned long long)lift.timestamp_us,
                   (unsigned long)lift.count);
        }
    }
}

/* ===================================================================
 *  Battery monitor bring-up
 *
 *  Started AFTER the SD logger, unlike every sensor above. Battery
 *  samples are the one stream whose whole value is explaining what the
 *  other streams did, so there is no point producing them before there
 *  is somewhere to put them. It also means the first sample, taken
 *  synchronously below, lands in batt.bin as the pad voltage.
 * =================================================================== */

static esp_err_t battery_start_monitor(battery_handle_t *out)
{
    battery_config_t cfg = {
        .channel        = VBAT_ADC_CHANNEL,
        .divider_ratio  = VBAT_DIVIDER_RATIO,
        .sample_rate_hz = VBAT_SAMPLE_RATE_HZ,
        .oversample     = BATTERY_OVERSAMPLE,
        .warn_volts     = VBAT_WARN_VOLTS,
        .critical_volts = VBAT_CRITICAL_VOLTS,
    };

    battery_handle_t batt = NULL;
    esp_err_t err = battery_init(&cfg, &batt);
    if (err != ESP_OK) {
        return err;
    }

    /* Core 1 with the other non-time-critical work. The sensor tasks
     * own core 0 and there is no reason to make them share it with a
     * 10 Hz ADC read. */
    err = battery_start(batt, PRIO_BATTERY, 1);
    if (err != ESP_OK) {
        battery_stop(batt);
        return err;
    }

    *out = batt;
    return ESP_OK;
}

/* ===================================================================
 *  Status snapshot
 *
 *  One record a second describing the vehicle's own health, published
 *  as STREAM_STATUS so it lands in status.bin AND goes out over the
 *  radio. It is assembled here rather than in any driver because it is
 *  the only place that can see all of them at once.
 *
 *  It also answers CMD_PING immediately, which is what lets the ground
 *  station ask "are you alive?" without waiting out the next tick.
 * =================================================================== */

#define STATUS_PERIOD_MS   1000

/* How long the card stays mounted with no eject command. 0 disables the
 * timer entirely and restores the old behaviour (mounted until an
 * uplink eject or power down). */
#ifdef CONFIG_FC_SD_AUTO_EJECT_S
#define SD_AUTO_EJECT_S    CONFIG_FC_SD_AUTO_EJECT_S
#else
#define SD_AUTO_EJECT_S    (6 * 60 * 60)
#endif

/* ===================================================================
 *  Operator-commanded SD eject.
 *
 *  telemetry validated the command and the flight mode; this is the
 *  part that actually touches the card. It runs in status_task because
 *  that task already polls telemetry for pings, already holds every
 *  handle, and is the only one that can afford to stop for the ~0.5 s
 *  sd_logger_stop() spends waiting for the writer task to drain. Doing
 *  it in the telemetry task instead would stall the radio pump for the
 *  same half second, and doing it in a sensor task would stall a
 *  sensor.
 *
 *  ORDER IS EVERYTHING HERE:
 *
 *    1. log the event, while the router still has an SD destination,
 *       so it lands on the card as the last record in event.bin
 *    2. detach the router, closing the window where another task can
 *       hand records to a logger that is being torn down
 *    3. clear our own handle, which is what STATUS_FLAG_SD_OK reports
 *    4. stop the logger: drain, flush, close, unmount
 *
 *  Get 1 and 2 the wrong way round and the log never explains its own
 *  ending.
 * =================================================================== */
static void status_eject_sd(sensor_handles_t *h, uint32_t cause)
{
    event_log_event(EV_SD_UNMOUNTED, (uint8_t)flight_mode_get(), cause);

    sd_logger_handle_t sd = data_router_detach_sd();
    h->sd = NULL;

    if (!sd) {
        return;
    }

    esp_err_t err = sd_logger_stop(sd);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "SD card unmounted on uplink command -- safe to remove. "
                      "Power-cycle the flight computer to log again.");
    } else {
        /* The card is off the router either way, so logging has stopped
         * regardless; what failed is the clean close. Say so on the
         * downlink, since the operator is about to pull the card and
         * this is the difference between a clean filesystem and a
         * chkdsk. */
        ESP_LOGE(TAG, "SD unmount FAILED: %s -- the card may not be clean",
                 esp_err_to_name(err));
        event_log_error(ERR_SRC_SD, ERR_CODE_WRITE_FAILED, err, 0);
    }
}

static void status_task(void *arg)
{
    sensor_handles_t *h = (sensor_handles_t *)arg;
    uint32_t seq = 0;
    uint32_t last_sd_drops = 0;
    bool auto_eject_deferred = false;
    const uint64_t boot_us = (uint64_t)esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "status task running on core %d", (int)xPortGetCoreID());

    for (;;) {
        /* Performed before this tick's record is built, so the very
         * next status the ground receives is already reporting
         * STATUS_FLAG_SD_OK clear. telemetry sets the ping request
         * alongside the eject request, so that record goes out now
         * rather than at the next 1 Hz tick. */
        if (h->telemetry && telemetry_take_sd_eject_request(h->telemetry)) {
            /* Taken unconditionally, acted on only if there is a card.
             * Guarding the take itself on h->sd would leave the request
             * latched forever on a vehicle whose card never mounted --
             * a flag that is set and never cleared is the kind of thing
             * that fires at the worst possible moment later. */
            if (h->sd) {
                status_eject_sd(h, SD_UNMOUNT_CAUSE_UPLINK);
            } else {
                ESP_LOGW(TAG, "SD eject commanded, but no card is mounted");
            }
        }

        /* ---------------------------------------------------------------
         *  Auto-eject.
         *
         *  A vehicle left powered on the pad all day would otherwise sit
         *  with a mounted card until somebody remembers to eject it or
         *  pulls the battery, and pulling the battery is the case this
         *  whole path exists to avoid. After SD_AUTO_EJECT_S of uptime
         *  the card unmounts itself.
         *
         *  DEFERRED, never refused, while the vehicle is ARMED or in
         *  FLIGHT: exactly the same gate the uplink command uses, so
         *  there is one rule about when an eject is legal rather than
         *  two. A long pad hold that turns into a launch at T+5:59
         *  therefore cannot cost the flight -- the unmount simply waits
         *  until the vehicle is back in STANDBY or LANDED.
         * --------------------------------------------------------------- */
        /* esp_timer_get_time() rather than rec.timestamp_us: this runs
         * BEFORE this tick's record is built, so that the first status
         * to go out after an auto-eject already reports SD_OK clear.
         * rec does not exist yet at this point. */
        const uint32_t uptime_s =
            (uint32_t)(((uint64_t)esp_timer_get_time() - boot_us) / 1000000ULL);
        if (h->sd && SD_AUTO_EJECT_S > 0 && uptime_s >= SD_AUTO_EJECT_S) {
            flight_mode_t m = flight_mode_get();
            if (m == MODE_STANDBY || m == MODE_LANDED) {
                ESP_LOGW(TAG, "auto-eject: %d s of uptime reached, unmounting the card",
                         (int)SD_AUTO_EJECT_S);
                status_eject_sd(h, SD_UNMOUNT_CAUSE_TIMER);
            } else if (!auto_eject_deferred) {
                /* Once, not once a second. */
                auto_eject_deferred = true;
                ESP_LOGW(TAG, "auto-eject deadline reached in %s -- deferred until "
                              "the vehicle is back in STANDBY or LANDED",
                         flight_mode_name(m));
            }
        }
        bool pinged = h->telemetry && telemetry_take_ping_request(h->telemetry);

        status_record_t rec = {0};
        rec.seq          = ++seq;
        rec.timestamp_us = (uint64_t)esp_timer_get_time();
        rec.mode         = (uint8_t)flight_mode_get();
        rec.uptime_s     = (uint16_t)((rec.timestamp_us - boot_us) / 1000000ULL);

        uint8_t flags = 0;
        if (h->sd)        { flags |= STATUS_FLAG_SD_OK; }
        /* TELEM_OK used to mean "the handle is non-NULL", which is true
         * whenever the UART opened and says nothing about whether a radio
         * is on the other end. It now means the module answered an ATVR
         * at boot. Reading this over the downlink and seeing it clear is
         * not a contradiction -- it means data is flowing but the probe
         * did not, which is worth knowing. It is on the card either way. */
        if (h->telemetry) {
            telemetry_diag_t td0; telemetry_get_diagnostics(h->telemetry, &td0);
            if (td0.radio_verified) { flags |= STATUS_FLAG_TELEM_OK; }
        }
        if (flight_mode_disarm_allowed()) { flags |= STATUS_FLAG_DISARM_OK; }

        /* AGL comes from flight_logic, which owns the pad reference.
         * This used to report bmp388's raw MSL altitude in a field named
         * altitude_agl_m -- off by the launch site's elevation, which at
         * a desert site is thousands of feet of silent error in the one
         * number the ground station watches during ascent. There is one
         * definition of "above the pad" in this firmware and it lives in
         * the estimator that does the ground calibration. */
        deploy_state_t fs;
        deploy_logic_get_state(&fs);
        rec.altitude_agl_m = fs.altitude_agl_m;
        if (fs.liftoff) { flags |= STATUS_FLAG_LIFTOFF_SW; }
        if (fs.apogee)  { flags |= STATUS_FLAG_APOGEE; }
        if (h->imu && lsm6dsox_liftoff_detected(h->imu)) {
            flags |= STATUS_FLAG_LIFTOFF_HW;
        }
        if (h->gps) {
            gps_data_t g;
            if (gps_get_latest(h->gps, &g) == ESP_OK && g.fix_quality > 0) {
                flags |= STATUS_FLAG_GPS_FIX;
            }
        }
        if (flight_mode_get() == MODE_LANDED) { flags |= STATUS_FLAG_LANDED; }
        rec.flags = flags;

        /* Latched rather than read straight through: after an eject
         * there is no handle to ask, and a field that fell back to 0
         * would read on the ground as "the drop counter cleared"
         * instead of "this is where it stopped". The last real value
         * keeps being reported. */
        if (h->sd) {
            sd_logger_diag_t sdd; sd_logger_get_diagnostics(h->sd, &sdd);
            last_sd_drops = sdd.queue_full_drops;
        }
        rec.sd_drops = last_sd_drops;
        if (h->telemetry) {
            telemetry_diag_t td; telemetry_get_diagnostics(h->telemetry, &td);
            rec.telem_drops = td.queue_full_drops;
        }

        data_router_publish(STREAM_STATUS, &rec, sizeof(rec));

        if (pinged) {
            /* A ping is answered on the spot and then the cadence
             * resumes from now, so a burst of pings cannot pile up
             * behind a stale deadline. */
            last_wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

/* ===================================================================
 *  Entry point
 * =================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 flight sensors, interrupt driven");

    /* Armed before anything else starts. PRIO_DISCRETE outranks every
     * other task in this project on purpose -- see the _Static_assert
     * block near the top of this file. Input polarity is confirmed on
     * both lines (LOW-to-HIGH); the divider's usable voltage window is
     * not. See discrete_in.h before trusting this near real hardware.
     *
     * event_log comes up first so that anything failing below has
     * somewhere to report to. It has no hardware of its own to fail. */
    /* Sink installed later, once the router exists -- see below. Errors
     * reported before that are counted but not recorded, which is the
     * honest behaviour: there is genuinely nowhere to put them yet. */
    event_log_init(NULL);

    /* Before any task that reads the mode is started. Boot state is
     * STANDBY: slow rates, gyro powered down, minimal airtime. */
    flight_mode_init();

#ifdef CONFIG_FC_MODE_TEST
    ESP_LOGE(TAG, "=====================================================");
    ESP_LOGE(TAG, "  TEST BUILD -- NOT FOR FLIGHT");
    ESP_LOGE(TAG, "  Scripted STANDBY/FLIGHT cycle, %d s per step.",
             CONFIG_FC_TEST_STEP_S);
    ESP_LOGE(TAG, "  menuconfig -> Flight Computer Operating Mode");
    ESP_LOGE(TAG, "=====================================================");
#endif

#if !defined(CONFIG_FC_ENABLE_DISCRETE_IN)
    /* Deliberately loud, and at error level. This is the board's only
     * independent record of when the Blue Raven fired; a build that has
     * stopped watching for it must not be able to look like a normal
     * boot. See FC_ENABLE_DISCRETE_IN in menuconfig. */
    ESP_LOGE(TAG, "=====================================================");
    ESP_LOGE(TAG, " BLUE RAVEN DISCRETE INPUTS ARE DISABLED IN THIS BUILD");
    ESP_LOGE(TAG, " Staging (J8) and recovery (J7) are NOT being watched.");
    ESP_LOGE(TAG, " event.bin will contain no EV_STAGING_SIGNAL and no");
    ESP_LOGE(TAG, " EV_RECOVERY_SIGNAL. Bench builds only -- turn this");
    ESP_LOGE(TAG, " back on (FC_ENABLE_DISCRETE_IN) before flying.");
    ESP_LOGE(TAG, "=====================================================");
#else
    static const discrete_in_config_t discrete_cfgs[2] = {
        { .input_pin = PIN_STAGING_IN,  .output_pin = PIN_STAGING_OUT,
          .event = EV_STAGING_SIGNAL,  .channel = 0, .name = "staging",
          .source = "Blue Raven staging out (tapped from the actuation branch)" },
        { .input_pin = PIN_RECOVERY_IN, .output_pin = GPIO_NUM_NC,
          .event = EV_RECOVERY_SIGNAL, .channel = 1, .name = "recovery",
          .source = "Blue Raven recovery out (tapped from the actuation branch)" },
    };
    for (int i = 0; i < 2; i++) {
        esp_err_t err = discrete_in_init(&discrete_cfgs[i], &s_sensors.discrete[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s signal init failed -- refusing to continue", discrete_cfgs[i].name);
            return;
        }
        err = discrete_in_start(s_sensors.discrete[i], PRIO_DISCRETE, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s task creation failed -- refusing to continue", discrete_cfgs[i].name);
            return;
        }
    }
#endif /* CONFIG_FC_ENABLE_DISCRETE_IN */

    /* The shared sensor bus, before either driver's init. Which interface
     * this brings up -- SPI or I2C -- is decided at build time in
     * menuconfig (SET sensor bus -> Interface); nothing here changes
     * between the two. AFTER SWITCHING, REMOVE ALL POWER before testing:
     * the BMP388 latches into SPI on its first CS edge and stays there
     * until it loses power, and the reset button does not cut sensor
     * power. */
    esp_err_t bus_err = sensor_bus_init();
    if (bus_err != ESP_OK) {
        ESP_LOGE(TAG, "sensor bus (%s) init failed: %s",
                 sensor_bus_name(), esp_err_to_name(bus_err));
        event_log_error(ERR_SRC_SYSTEM, ERR_CODE_INIT_FAILED, bus_err, 0);
        return;
    }

    /* Reads BMP388 CHIP_ID and LSM6DSOX WHO_AM_I and logs a verdict for
     * each. Deliberately not a gate: a bad ID here is worth seeing in the
     * log next to whichever driver then fails, rather than stopping boot
     * before either of them gets to report. */
    if (sensor_bus_check_ids() != ESP_OK) {
        ESP_LOGW(TAG, "sensor bus (%s): chip-ID check failed -- see the lines "
                      "above; sensor init below will almost certainly fail too",
                 sensor_bus_name());
    }

    /* Each start function is called exactly once and its result kept.
     * The previous form chained all three with || inside the if, which
     * failed two ways: || short-circuits, so a sensor that failed early
     * stopped the later ones from ever being started; and the diagnostic
     * block below then called every start function a SECOND time, so a
     * failing sensor was re-initialised and reported twice, and a sensor
     * skipped by the short-circuit was reported on its first real
     * attempt. Keeping the codes also means event.bin records what
     * actually went wrong instead of a flat ESP_FAIL. */
    esp_err_t baro_err = baro_start(&s_sensors.baro);
    esp_err_t imu_err  = imu_start(&s_sensors.imu);
    esp_err_t gps_err  = gps_start_sensor(&s_sensors.gps);

    if (baro_err != ESP_OK || imu_err != ESP_OK || gps_err != ESP_OK) {
        ESP_LOGE(TAG, "One or more sensors failed to start");

        if (baro_err != ESP_OK) {
            ESP_LOGE(TAG, "barometer failed to start: %s", esp_err_to_name(baro_err));
            event_log_error(ERR_SRC_BMP388, ERR_CODE_INIT_FAILED, baro_err, 0);
        }
        if (imu_err != ESP_OK) {
            ESP_LOGE(TAG, "IMU failed to start: %s", esp_err_to_name(imu_err));
            event_log_error(ERR_SRC_LSM6DSOX, ERR_CODE_INIT_FAILED, imu_err, 0);
        }
        if (gps_err != ESP_OK) {
            ESP_LOGE(TAG, "GPS failed to start: %s", esp_err_to_name(gps_err));
            event_log_error(ERR_SRC_GPS, ERR_CODE_INIT_FAILED, gps_err, 0);
        }
        return;
    }
    /*
    
*/
    /* SD and telemetry are each started independently and neither
     * failure is fatal -- losing one destination shouldn't cost you the
     * other. Each failure sets its handle to NULL; data_router_init()
     * is fine with any combination of NULLs, and data_router_publish()
     * just skips whichever destinations aren't there. */
    if (sd_start(&s_sensors.sd) != ESP_OK) {
        ESP_LOGE(TAG, "SD logger failed to start -- FLYING WITHOUT SD LOGGING");
        event_log_error(ERR_SRC_SD, ERR_CODE_MOUNT_FAILED, ESP_FAIL, 0);
        s_sensors.sd = NULL;
    }

    if (telem_start(&s_sensors.telemetry) != ESP_OK) {
        ESP_LOGE(TAG, "telemetry failed to start -- FLYING WITHOUT TELEMETRY");
        event_log_error(ERR_SRC_TELEMETRY, ERR_CODE_INIT_FAILED, ESP_FAIL, 0);
        s_sensors.telemetry = NULL;
    }

    data_router_init(s_sensors.sd, s_sensors.telemetry);

    /* Installed after the router, because the receiver's first act on a
     * chunk is to log an event, and an event logged before the router
     * exists goes nowhere. */
    config_receiver_init();
    if (s_sensors.telemetry) {
        telemetry_set_uplink_hook(s_sensors.telemetry, config_receiver_handle_packet);
    }

    /* Announced only now: before data_router_init() there was nowhere
     * for the record to go, so an earlier call would have been dropped
     * on the floor. */
    event_log_init(data_router_publish);
    event_log_event(EV_BOOT, 0, 0);
    if (s_sensors.sd) {
        event_log_event(EV_SD_MOUNTED, 0, 0);
    }

    /* Battery monitoring is non-fatal in the same way SD and telemetry
     * are: knowing the pack voltage is valuable, but a rocket that
     * refuses to fly because an ADC would not initialize is worse than
     * one that flies without the housekeeping stream. */
    if (battery_start_monitor(&s_sensors.battery) != ESP_OK) {
        ESP_LOGE(TAG, "battery monitor failed to start -- no pack voltage logging");
        event_log_error(ERR_SRC_BATTERY, ERR_CODE_INIT_FAILED, ESP_FAIL, 0);
        s_sensors.battery = NULL;
    } else {
        /* One synchronous reading now, so the pad voltage is the first
         * record in batt.bin rather than whatever the task happens to
         * catch 100 ms later. */
        battery_data_t pad;
        if (battery_read_once(s_sensors.battery, &pad) == ESP_OK) {
            ESP_LOGI(TAG, "pack voltage on the pad: %.2f V", pad.battery_v);
            if (pad.battery_v < VBAT_WARN_VOLTS) {
                ESP_LOGW(TAG, "pack is already below the %.1f V warning "
                              "threshold before flight", VBAT_WARN_VOLTS);
            }
        }
    }

    /* Flight logic is NOT part of the router -- see data_router.h for
     * why. GPIO init failure here IS treated as worth logging loudly:
     * a deployment channel that never got configured as an output is a
     * real safety concern, not a "missing nice-to-have" like a lost
     * telemetry link. */
    if (deploy_logic_bring_up() != ESP_OK) {
        ESP_LOGE(TAG, "flight logic GPIO init failed -- DEPLOYMENT CHANNELS NOT CONFIGURED");
        event_log_error(ERR_SRC_FLIGHT_LOGIC, ERR_CODE_INIT_FAILED, ESP_FAIL, 0);
    } else {
        TaskHandle_t deploy_handle = xTaskCreateStaticPinnedToCore(
            flight_logic_task, "flight_logic", FLIGHT_LOGIC_TASK_STACK_WORDS,
            (void *)&s_sensors, PRIO_DEPLOY, s_flight_logic_stack,
            &s_flight_logic_tcb, 1);
        if (!deploy_handle) {
            ESP_LOGE(TAG, "flight_logic_task creation failed -- NO DEPLOYMENT LOGIC RUNNING");
            event_log_error(ERR_SRC_FLIGHT_LOGIC, ERR_CODE_TASK_FAILED, ESP_FAIL, 0);
        }
    }

    /* No bridge tasks to create anymore -- bmp388_task/lsm6dsox_task/
     * gps_task now call data_router_publish() themselves, directly
     * where each sample is built. Those three sensor tasks were
     * already started above by baro_start()/imu_start()/gps_start(). */

    TaskHandle_t status_handle = xTaskCreateStaticPinnedToCore(
        status_task, "status", STATUS_TASK_STACK_WORDS, (void *)&s_sensors,
        PRIO_STATUS, s_status_stack, &s_status_tcb, 1);
    if (!status_handle) {
        ESP_LOGE(TAG, "status_task creation failed -- no health snapshots");
        event_log_error(ERR_SRC_SYSTEM, ERR_CODE_TASK_FAILED, ESP_FAIL, 0);
    }

    /* print_task is debug-only; it deliberately runs BELOW everything
     * on the priority list so console output can never delay flight work. */
    TaskHandle_t print_handle = xTaskCreateStaticPinnedToCore(
        print_task, "print", PRINT_TASK_STACK_WORDS, (void *)&s_sensors,
        PRIO_PRINT, s_print_stack, &s_print_tcb, 1);
    if (!print_handle) {
        /* Debug-only output -- worth a log line, not worth treating as
         * fatal the way a sensor or SD failure would be. */
        ESP_LOGE(TAG, "print_task creation failed -- continuing without console output");
    }

#ifdef CONFIG_FC_MODE_TEST
    /* Last, so the first step is timed from "everything is running". */
    TaskHandle_t test_handle = xTaskCreateStaticPinnedToCore(
        test_sequence_task, "test_seq", TEST_SEQ_STACK_WORDS, NULL,
        PRIO_STATUS, s_test_seq_stack, &s_test_seq_tcb, 1);
    if (!test_handle) {
        ESP_LOGE(TAG, "test_sequence_task creation failed -- no TEST sequence");
    }
#endif
}
