#include "bus_bringup.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "driver/spi_master.h"
#include "esp_rom_gpio.h"
#include "soc/spi_periph.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

static const char *TAG = "BRINGUP";

/* Half a bit period. 10 us -> ~50 kHz, slow enough that trace length,
 * stray capacitance and the breakouts' own pull-ups cannot be blamed. */
#define BB_HALF_US  10

/* ===================================================================
 *  Phase 1 -- pin toggle
 * =================================================================== */

static void toggle_pin(const char *name, gpio_num_t pin, int cycles)
{
    if (pin == GPIO_NUM_NC) {
        return;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGE(TAG, "  %-5s GPIO%-2d  gpio_config FAILED -- pin not usable as output",
                 name, (int)pin);
        return;
    }

    ESP_LOGI(TAG, "  %-5s GPIO%-2d  toggling for %d s -- probe the header pad now",
             name, (int)pin, cycles / 5);
    for (int i = 0; i < cycles; i++) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Back to high-Z so the next phase starts from a known state. */
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

/* ===================================================================
 *  Phase 2 -- bit-banged register read
 *
 *  SPI mode 0/3 as both these parts implement it: the controller sets
 *  MOSI while the clock is low, the peripheral shifts its own bit out
 *  on the falling edge, and both sample on the rising edge. Idling the
 *  clock low (mode 0) and taking the first rising edge as the first
 *  sample point is what the peripheral expects.
 * =================================================================== */

static void bb_pins_setup(const bus_bringup_cfg_t *cfg)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << cfg->sclk) | (1ULL << cfg->mosi) |
                        (1ULL << cfg->cs_imu) | (1ULL << cfg->cs_baro),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    /* MISO gets a pull-DOWN, deliberately. If a chip is answering it
     * will drive this line hard and the pull-down is irrelevant; if
     * nothing is answering, the line reads 0 rather than floating to
     * whatever the last edge left behind -- which is how a dead bus can
     * masquerade as a plausible-looking value. */
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << cfg->miso,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    gpio_set_level(cfg->cs_imu, 1);
    gpio_set_level(cfg->cs_baro, 1);
    gpio_set_level(cfg->sclk, 0);
    gpio_set_level(cfg->mosi, 0);
    esp_rom_delay_us(100);
}

static uint8_t bb_xfer_byte(const bus_bringup_cfg_t *cfg, uint8_t out)
{
    uint8_t in = 0;

    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(cfg->mosi, (out >> bit) & 1);
        esp_rom_delay_us(BB_HALF_US);

        gpio_set_level(cfg->sclk, 1);
        esp_rom_delay_us(BB_HALF_US);
        in = (uint8_t)((in << 1) | (gpio_get_level(cfg->miso) & 1));

        gpio_set_level(cfg->sclk, 0);
    }
    return in;
}

/* Returns the register value, and reports the raw MISO idle level so a
 * stuck-low line is distinguishable from a chip that answers zero. */
static uint8_t bb_read_reg(const bus_bringup_cfg_t *cfg, gpio_num_t cs,
                           uint8_t reg, int dummy_bytes)
{
    gpio_set_level(cs, 0);
    esp_rom_delay_us(BB_HALF_US);

    (void)bb_xfer_byte(cfg, (uint8_t)(reg | 0x80));   /* bit7 = read */
    for (int i = 0; i < dummy_bytes; i++) {
        (void)bb_xfer_byte(cfg, 0xFF);
    }
    uint8_t val = bb_xfer_byte(cfg, 0xFF);

    esp_rom_delay_us(BB_HALF_US);
    gpio_set_level(cs, 1);
    esp_rom_delay_us(100);
    return val;
}

static void bb_probe(const bus_bringup_cfg_t *cfg)
{
    bb_pins_setup(cfg);

    /* With both chip selects high every sensor should be tri-stated, so
     * MISO belongs to the pull-down alone. A 1 here means something on
     * the net is driving it -- a breakout strapped for I2C addressing,
     * or a chip that never released the bus. */
    int idle = gpio_get_level(cfg->miso);
    ESP_LOGI(TAG, "  MISO idle level with both CS high: %d %s", idle,
             idle ? "<-- SOMETHING IS DRIVING MISO" : "(expected)");

    uint8_t imu = bb_read_reg(cfg, cfg->cs_imu, 0x0F, 0);
    ESP_LOGI(TAG, "  LSM6DSOX WHO_AM_I = 0x%02X (expect 0x6C)  %s",
             imu, imu == 0x6C ? "OK" : "MISMATCH");

    /* The BMP388 returns one dummy byte after the address in SPI mode.
     * It also powers up talking I2C and only switches to SPI on the
     * first CS falling edge, so the first read after reset is expected
     * to be garbage -- Bosch's own driver throws one away for exactly
     * this reason. Both are logged so a part that is genuinely absent
     * is distinguishable from one that just needed the wake-up. */
    uint8_t baro_first = bb_read_reg(cfg, cfg->cs_baro, 0x00, 1);
    uint8_t baro       = bb_read_reg(cfg, cfg->cs_baro, 0x00, 1);
    ESP_LOGI(TAG, "  BMP388   CHIP_ID  = 0x%02X (first read 0x%02X, expect 0x50) %s",
             baro, baro_first,
             (baro == 0x50 || baro == 0x60) ? "OK" : "MISMATCH");
    if (baro == 0x60) {
        ESP_LOGW(TAG, "  0x60 is a BMP390, not a BMP388 -- different part, same bus.");
    }

    if (imu == 0x6C || baro == 0x50) {
        ESP_LOGW(TAG, "  >> Bit-bang WORKS. Wiring, power and the parts are fine.");
        ESP_LOGW(TAG, "  >> The fault is in the SPI peripheral setup, not the board.");
    } else if (imu == 0xFF && baro == 0xFF) {
        ESP_LOGW(TAG, "  >> All ones: MISO is stuck high. Suspect a chip that is");
        ESP_LOGW(TAG, "  >> not powered, or CS never reaching the breakouts.");
    } else {
        ESP_LOGW(TAG, "  >> Both dead at 50 kHz with the peripheral out of the");
        ESP_LOGW(TAG, "  >> picture. Check 3V3 and GND at J3/J4 first, then that");
        ESP_LOGW(TAG, "  >> phase 1's square waves really appear on J3.4/J4.4.");
    }

    /* Release everything. */
    gpio_set_direction(cfg->sclk, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->mosi, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->cs_imu, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->cs_baro, GPIO_MODE_INPUT);
}

/* ===================================================================
 *  Phase 3 -- peripheral variant matrix
 *
 *  Phase 2 proved the chip answers when the four pins are driven by
 *  hand. So the pins, the power and the part are all fine, and exactly
 *  one thing is left: something about how the SPI peripheral is being
 *  driven. Rather than guess which knob, turn each one and read the
 *  same register through the peripheral every time.
 *
 *  The variables worth separating:
 *
 *    hardware vs software CS -- with spics_io_num the peripheral owns
 *      the chip select and asserts it around each transaction. With -1
 *      the driver never touches it and this code toggles it as a plain
 *      GPIO, exactly as phase 2 did. If software CS reads 0x6C and
 *      hardware CS does not, the chip select is never reaching the part
 *      and every byte the peripheral sends is being ignored -- which
 *      would explain both sensors failing identically.
 *
 *    mode 0 vs mode 3 -- these differ only in the clock's idle level,
 *      and this part accepts both, so a difference here would point at
 *      the first clock edge landing inside the CS setup window rather
 *      than at the mode itself.
 *
 *    1 MHz vs 100 kHz -- phase 2 succeeded at 50 kHz. If only the slow
 *      variant works, it is a timing problem and input_delay_ns is the
 *      next thing to reach for.
 * =================================================================== */

/* Static and word-aligned: never let a stack array's alignment become
 * one more variable in an experiment about buffers. */
static WORD_ALIGNED_ATTR uint8_t s_tx[4];
static WORD_ALIGNED_ATTR uint8_t s_rx[4];

typedef struct {
    const char *name;
    uint8_t     mode;
    bool        hw_cs;
    int         hz;
} periph_variant_t;

static uint8_t periph_read_imu_id(const bus_bringup_cfg_t *cfg,
                                  const periph_variant_t *v, esp_err_t *err_out)
{
    *err_out = ESP_OK;

    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) { *err_out = err; return 0; }

    if (!v->hw_cs) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->cs_imu,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level(cfg->cs_imu, 1);
    }

    spi_device_interface_config_t devcfg = {
        .mode           = v->mode,
        .clock_speed_hz = v->hz,
        .spics_io_num   = v->hw_cs ? cfg->cs_imu : -1,
        .queue_size     = 1,
    };
    spi_device_handle_t dev;
    err = spi_bus_add_device(SPI2_HOST, &devcfg, &dev);
    if (err != ESP_OK) {
        *err_out = err;
        spi_bus_free(SPI2_HOST);
        return 0;
    }

    s_tx[0] = 0x0F | 0x80;   /* WHO_AM_I, read */
    s_tx[1] = 0xFF;
    s_rx[0] = s_rx[1] = 0;

    spi_transaction_t t = {
        .length    = 16,
        .rxlength  = 16,
        .tx_buffer = s_tx,
        .rx_buffer = s_rx,
    };

    if (!v->hw_cs) {
        gpio_set_level(cfg->cs_imu, 0);
        esp_rom_delay_us(2);
    }
    err = spi_device_polling_transmit(dev, &t);
    if (!v->hw_cs) {
        esp_rom_delay_us(2);
        gpio_set_level(cfg->cs_imu, 1);
    }
    *err_out = err;

    spi_bus_remove_device(dev);
    spi_bus_free(SPI2_HOST);
    esp_rom_delay_us(200);

    return s_rx[1];
}

static void periph_probe(const bus_bringup_cfg_t *cfg)
{
    /* All four modes, not just 0 and 3. Modes 0 and 3 both sample while
     * the clock is HIGH; modes 1 and 2 sample while it is LOW. The old
     * table only ever tried the clock-high pair, so it could not tell a
     * dead bus from a half-period phase error.
     *
     * That distinction matters because of how phase 2 succeeds: the
     * bit-banger sets MOSI, raises the clock, waits a FULL half period,
     * and only then samples MISO. Sampling that late reads the settled
     * value no matter which edge the sensor drove it on, so bit-bang
     * cannot distinguish the two -- while the peripheral, sampling right
     * at the edge, catches a transition and reads zeros.
     *
     * The 50 kHz rows match the bit-banger's rate exactly, so a row that
     * differs from phase 2 differs by mode alone and nothing else. */
    static const periph_variant_t variants[] = {
        { "mode0  hwCS  1 MHz",   0, true,  1000000 },
        { "mode1  hwCS  1 MHz",   1, true,  1000000 },
        { "mode2  hwCS  1 MHz",   2, true,  1000000 },
        { "mode3  hwCS  1 MHz",   3, true,  1000000 },
        { "mode0  swCS  100 kHz", 0, false,  100000 },
        { "mode1  swCS  100 kHz", 1, false,  100000 },
        { "mode2  swCS  100 kHz", 2, false,  100000 },
        { "mode3  swCS  100 kHz", 3, false,  100000 },
        { "mode0  swCS  50 kHz",  0, false,   50000 },
        { "mode1  swCS  50 kHz",  1, false,   50000 },
        { "mode2  swCS  50 kHz",  2, false,   50000 },
        { "mode3  swCS  50 kHz",  3, false,   50000 },
    };

    /* Park the barometer's chip select high for the whole phase.
     * bb_probe() left it as a floating input, and a floating CS means the
     * BMP388 may also consider itself selected -- two chips driving MISO
     * at once reads back as zeros no matter what the peripheral does.
     * Phase 2 drove BOTH chip selects explicitly, which is a difference
     * between it and this phase that the variant table never accounted
     * for. Same reason the real drivers fail: whichever sensor inits
     * first does so while the other one's CS has never been configured. */
    gpio_config_t cs_park = {
        .pin_bit_mask = 1ULL << cfg->cs_baro,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_park);
    gpio_set_level(cfg->cs_baro, 1);

    int hw_ok = 0, sw_ok = 0;

    for (size_t i = 0; i < sizeof(variants) / sizeof(variants[0]); i++) {
        esp_err_t err;
        uint8_t got = periph_read_imu_id(cfg, &variants[i], &err);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "  %-20s -> %s", variants[i].name, esp_err_to_name(err));
            continue;
        }
        bool ok = (got == 0x6C);
        ESP_LOGI(TAG, "  %-20s -> 0x%02X %s", variants[i].name, got, ok ? "OK" : "");
        if (ok) {
            if (variants[i].hw_cs) { hw_ok++; } else { sw_ok++; }
        }
    }

    if (sw_ok && !hw_ok) {
        ESP_LOGW(TAG, "  >> Software CS works, hardware CS does not.");
        ESP_LOGW(TAG, "  >> The peripheral is not asserting the chip select, so the");
        ESP_LOGW(TAG, "  >> sensors never see a frame. Fix: spics_io_num = -1 in both");
        ESP_LOGW(TAG, "  >> drivers and drive CS from the driver around each transfer.");
    } else if (hw_ok || sw_ok) {
        ESP_LOGW(TAG, "  >> At least one variant reads 0x6C -- copy that combination");
        ESP_LOGW(TAG, "  >> into lsm6dsox_init()/bmp388_init() and re-test.");
    } else {
        ESP_LOGW(TAG, "  >> No peripheral variant works while bit-bang does. That");
        ESP_LOGW(TAG, "  >> points at pin routing through the GPIO matrix rather");
        ESP_LOGW(TAG, "  >> than at any device setting. Run phase 4 (loopback).");
    }
}

/* ===================================================================
 *  Phase 4 -- peripheral loopback (needs a MOSI->MISO jumper)
 * =================================================================== */

static void loopback(const bus_bringup_cfg_t *cfg)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  spi_bus_initialize failed: %s", esp_err_to_name(err));
        return;
    }

    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    spi_device_handle_t dev;
    err = spi_bus_add_device(SPI2_HOST, &devcfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        return;
    }

    s_tx[0] = 0xA5; s_tx[1] = 0x5A; s_tx[2] = 0x6C; s_tx[3] = 0x50;
    memset(s_rx, 0, sizeof(s_rx));

    spi_transaction_t t = {
        .length    = 32,
        .rxlength  = 32,
        .tx_buffer = s_tx,
        .rx_buffer = s_rx,
    };

    err = spi_device_polling_transmit(dev, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  transmit failed: %s", esp_err_to_name(err));
    } else if (memcmp(s_tx, s_rx, 4) == 0) {
        ESP_LOGW(TAG, "  >> Loopback OK: the peripheral drives MOSI and hears MISO.");
    } else {
        ESP_LOGW(TAG, "  >> Loopback FAILED: sent %02X %02X %02X %02X, got %02X %02X %02X %02X",
                 s_tx[0], s_tx[1], s_tx[2], s_tx[3],
                 s_rx[0], s_rx[1], s_rx[2], s_rx[3]);
        ESP_LOGW(TAG, "  >> All zeros with the jumper fitted means GPIO%d never",
                 (int)cfg->miso);
        ESP_LOGW(TAG, "  >> reaches the peripheral's MISO input.");
    }

    spi_bus_remove_device(dev);
    spi_bus_free(SPI2_HOST);
}

/* ===================================================================
 *  Phase 5 -- signal routing autopsy
 *
 *  Phase 2 read 0x6C by hand; phase 3 could not read it through the
 *  peripheral under any combination of mode, chip-select ownership or
 *  clock rate. The pins work and the peripheral does not, so what is
 *  left is the path between them: the GPIO matrix.
 *
 *  These three tests need no jumper and no meter. Each one asks the
 *  peripheral to listen to a pin the peripheral itself is driving, by
 *  re-routing the MISO *input* signal onto that pad through the matrix.
 *  If the data comes back, both directions of that pad work; if it does
 *  not, that pad is where the signal is being lost.
 *
 *    A -- listen on the MOSI pad. Tests the peripheral core, its clock
 *         generation, the MOSI output route, and the matrix input path.
 *         If A fails, nothing about this bus works and the pin numbers
 *         are irrelevant.
 *
 *    B -- drive the MISO pad with the MOSI signal and listen to it.
 *         Tests that pad in both directions.
 *
 *    C -- listen on the SCLK pad. In mode 0 the peripheral samples its
 *         input on the rising clock edge, which is precisely when the
 *         clock it is driving is high -- so a working clock pad reads
 *         back 0xFF on every byte, and a pad the clock never reaches
 *         reads 0x00. This is the one that matters if A and B pass.
 * =================================================================== */

/* drive_signal is the peripheral output that must keep driving this pad
 * while the peripheral listens back on it. mode picks the SPI mode, which
 * only matters for the clock pad -- see the note in routing_autopsy(). */
static uint8_t routing_test(const bus_bringup_cfg_t *cfg, gpio_num_t listen_pin,
                            uint32_t drive_signal, uint8_t mode,
                            esp_err_t *err_out)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    *err_out = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (*err_out != ESP_OK) { return 0; }

    spi_device_interface_config_t devcfg = {
        .mode           = mode,
        .clock_speed_hz = 1000000,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    spi_device_handle_t dev;
    *err_out = spi_bus_add_device(SPI2_HOST, &devcfg, &dev);
    if (*err_out != ESP_OK) { spi_bus_free(SPI2_HOST); return 0; }

    /* gpio_set_direction() must never appear here. It calls
     * gpio_output_enable(), which calls gpio_hal_matrix_out_default() --
     * "no peripheral output signal routed to the pin, just as a simple
     * GPIO output" -- so it unroutes the very signal this test needs on
     * the pad, and every pin then reads back a static level regardless of
     * whether the peripheral works. Route the output, enable the input
     * buffer (which leaves the output routing alone), then point the
     * peripheral's MISO input at this pad -- the same order the SPI
     * driver itself uses in spi_common.c. */
    esp_rom_gpio_connect_out_signal(listen_pin, drive_signal, false, false);
    gpio_input_enable(listen_pin);
    esp_rom_gpio_connect_in_signal(listen_pin,
                                   spi_periph_signal[SPI2_HOST].spiq_in,
                                   false);

    s_tx[0] = 0xA5; s_tx[1] = 0x5A; s_tx[2] = 0x6C; s_tx[3] = 0x50;
    memset(s_rx, 0, sizeof(s_rx));

    spi_transaction_t t = {
        .length    = 32,
        .rxlength  = 32,
        .tx_buffer = s_tx,
        .rx_buffer = s_rx,
    };
    *err_out = spi_device_polling_transmit(dev, &t);

    spi_bus_remove_device(dev);
    spi_bus_free(SPI2_HOST);
    esp_rom_delay_us(200);
    return 0;
}

static void routing_autopsy(const bus_bringup_cfg_t *cfg)
{
    esp_err_t err;

    routing_test(cfg, cfg->mosi, spi_periph_signal[SPI2_HOST].spid_out, 0, &err);
    bool a_ok = (err == ESP_OK) && (memcmp(s_tx, s_rx, 4) == 0);
    ESP_LOGI(TAG, "  A  listen on MOSI GPIO%-2d -> %02X %02X %02X %02X  %s",
             (int)cfg->mosi, s_rx[0], s_rx[1], s_rx[2], s_rx[3],
             a_ok ? "OK (echoes A5 5A 6C 50)" : "FAIL");

    routing_test(cfg, cfg->miso, spi_periph_signal[SPI2_HOST].spid_out, 0, &err);
    bool b_ok = (err == ESP_OK) && (memcmp(s_tx, s_rx, 4) == 0);
    ESP_LOGI(TAG, "  B  listen on MISO GPIO%-2d -> %02X %02X %02X %02X  %s",
             (int)cfg->miso, s_rx[0], s_rx[1], s_rx[2], s_rx[3],
             b_ok ? "OK (echoes A5 5A 6C 50)" : "FAIL");

    /* The clock pad needs two readings, not one. Sampling happens on the
     * rising edge in mode 0 and on the falling edge in mode 1, so a clock
     * that is really toggling reads back FF in mode 0 and 00 in mode 1.
     * A single mode-0 reading of FF proves nothing: a pad the clock never
     * reaches, left floating high, reads exactly the same. Only the
     * DIFFERENCE between the two modes is evidence. */
    routing_test(cfg, cfg->sclk, spi_periph_signal[SPI2_HOST].spiclk_out, 0, &err);
    bool c0_ff = (err == ESP_OK) && (s_rx[0] == 0xFF) && (s_rx[1] == 0xFF) &&
                 (s_rx[2] == 0xFF) && (s_rx[3] == 0xFF);
    uint8_t c0[4] = { s_rx[0], s_rx[1], s_rx[2], s_rx[3] };

    routing_test(cfg, cfg->sclk, spi_periph_signal[SPI2_HOST].spiclk_out, 1, &err);
    bool c1_00 = (err == ESP_OK) && (s_rx[0] == 0x00) && (s_rx[1] == 0x00) &&
                 (s_rx[2] == 0x00) && (s_rx[3] == 0x00);

    bool c_ok = c0_ff && c1_00;
    ESP_LOGI(TAG, "  C  listen on SCLK GPIO%-2d -> mode0 %02X %02X %02X %02X, "
                  "mode1 %02X %02X %02X %02X  %s",
             (int)cfg->sclk, c0[0], c0[1], c0[2], c0[3],
             s_rx[0], s_rx[1], s_rx[2], s_rx[3],
             c_ok ? "OK (clock really toggles)"
                  : "FAIL (need FF.. in mode0 and 00.. in mode1)");

    if (a_ok && b_ok && !c_ok) {
        ESP_LOGW(TAG, "  >> Data pins route fine; the CLOCK does not reach GPIO%d.",
                 (int)cfg->sclk);
        ESP_LOGW(TAG, "  >> The peripheral is shifting bytes out into a bus with no");
        ESP_LOGW(TAG, "  >> clock, which is why every sensor reads back zeros while");
        ESP_LOGW(TAG, "  >> bit-banging the same pin by hand works.");
        ESP_LOGW(TAG, "  >> Next: bodge SCLK to a free pad -- GPIO47, 46, 21 or 48 --");
        ESP_LOGW(TAG, "  >> and change PIN_SCLK to match. If that fixes it, GPIO54 is");
        ESP_LOGW(TAG, "  >> the problem and PCB v2 should move the clock.");
    } else if (!a_ok) {
        ESP_LOGW(TAG, "  >> Even the peripheral's own MOSI pad does not echo. The");
        ESP_LOGW(TAG, "  >> fault is in the SPI2 peripheral or its clock, not in any");
        ESP_LOGW(TAG, "  >> pin assignment. Check that SPI2 is not claimed elsewhere.");
    } else if (a_ok && b_ok && c_ok) {
        ESP_LOGW(TAG, "  >> All three pads route correctly, which contradicts phase 3.");
        ESP_LOGW(TAG, "  >> Suspect the chip-select timing next: try cs_ena_pretrans");
        ESP_LOGW(TAG, "  >> and cs_ena_posttrans of 2 or more.");
    } else {
        ESP_LOGW(TAG, "  >> Mixed result -- the FAIL line above names the bad pad.");
    }

    /* Undo the re-routing before the real sensor_bus_init() runs. Each
     * test left a peripheral output signal on its pad -- MISO in
     * particular ends up driven by the controller's own data signal,
     * which would fight the sensors' SDO pins. Here gpio_set_direction()
     * is the right tool precisely because it unroutes the matrix output
     * and hands the pad back to plain GPIO. */
    gpio_set_direction(cfg->mosi, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->miso, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->sclk, GPIO_MODE_INPUT);
}

/* ===================================================================
 *  Phase 6 -- how fast is the clock, really?
 *
 *  Every test so far is blind to clock FREQUENCY. Phase 5's test C reads
 *  the clock pad at the sampling instant, and that instant is locked to
 *  the clock edge by construction -- so it reads FF in mode 0 whether the
 *  clock is 50 kHz or 50 MHz. Phase 3 sweeps a requested rate, but never
 *  checks that the peripheral produced it.
 *
 *  So time a long transfer and divide. A transfer of N bits at H hertz
 *  must take N/H seconds; anything much shorter means the divider is not
 *  producing the rate that was asked for, and a clock far above the
 *  part's 10 MHz limit would explain a sensor that answers a hand-driven
 *  pin and ignores the peripheral.
 * =================================================================== */

static WORD_ALIGNED_ATTR uint8_t s_long_tx[64];
static WORD_ALIGNED_ATTR uint8_t s_long_rx[64];

static void clock_rate_check(const bus_bringup_cfg_t *cfg, int requested_hz)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) {
        return;
    }

    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .clock_speed_hz = requested_hz,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    spi_device_handle_t dev;
    if (spi_bus_add_device(SPI2_HOST, &devcfg, &dev) != ESP_OK) {
        spi_bus_free(SPI2_HOST);
        return;
    }

    memset(s_long_tx, 0xA5, sizeof(s_long_tx));
    spi_transaction_t t = {
        .length    = sizeof(s_long_tx) * 8,
        .rxlength  = sizeof(s_long_rx) * 8,
        .tx_buffer = s_long_tx,
        .rx_buffer = s_long_rx,
    };

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = spi_device_polling_transmit(dev, &t);
    int64_t t1 = esp_timer_get_time();

    spi_bus_remove_device(dev);
    spi_bus_free(SPI2_HOST);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  %8d Hz requested -> transmit failed: %s",
                 requested_hz, esp_err_to_name(err));
        return;
    }

    int64_t us = t1 - t0;
    int bits = (int)(sizeof(s_long_tx) * 8);
    /* Includes driver overhead, so the measurement is a floor on the
     * period and therefore a ceiling on the rate -- good enough to catch
     * an order-of-magnitude error, which is all this is looking for. */
    int measured = us > 0 ? (int)((int64_t)bits * 1000000 / us) : 0;

    ESP_LOGI(TAG, "  %8d Hz requested -> %d bits in %lld us = ~%d Hz %s",
             requested_hz, bits, (long long)us, measured,
             (measured > requested_hz * 4) ? "<-- FAR TOO FAST" : "");
}

static void clock_rate_probe(const bus_bringup_cfg_t *cfg)
{
    clock_rate_check(cfg, 50000);
    clock_rate_check(cfg, 1000000);
}

/* ===================================================================
 *  Phase 7 -- what did spi_bus_initialize() actually wire up?
 *
 *  Phase 5 proved the peripheral can read the MISO pad, but it proved it
 *  using routing this file set up BY HAND -- including an explicit
 *  gpio_input_enable(). It never showed that the SPI driver does the
 *  same. If the driver leaves the MISO input unrouted, or leaves the
 *  pad's input buffer switched off, every read returns a clean 0x00 while
 *  every transaction still reports ESP_OK -- which is exactly the symptom
 *  and is invisible to all six phases above.
 *
 *  So stop inferring and read the matrix registers back.
 * =================================================================== */

static inline uint32_t reg_rd(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

/* IN_SEL registers are indexed by SIGNAL; the header only defines FUNC1
 * onward, so index from there. OUT_SEL registers are indexed by PAD. */
#define IN_SEL_REG(sig)   (GPIO_FUNC1_IN_SEL_CFG_REG + 4 * ((sig) - 1))
#define OUT_SEL_REG(pad)  (GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * (pad))
#define IOMUX_PAD_REG(p)  (REG_IO_MUX_BASE + 0x4 + 4 * (p))

static void matrix_dump(const bus_bringup_cfg_t *cfg)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  spi_bus_initialize failed: %s", esp_err_to_name(err));
        return;
    }

    uint32_t q_sig   = spi_periph_signal[SPI2_HOST].spiq_in;
    uint32_t in_raw  = reg_rd(IN_SEL_REG(q_sig));
    int      in_gpio = (int)(in_raw & 0x3F);
    int      use_mtx = (int)((in_raw >> 7) & 1);

    /* 0x3F means "constant high", 0x3E means "constant low" -- either one
     * is a peripheral wired to nothing at all. */
    ESP_LOGI(TAG, "  spiq_in (MISO in, sig %u): GPIO%d, via-matrix=%d, raw=0x%08X",
             (unsigned)q_sig, in_gpio, use_mtx, (unsigned)in_raw);
    ESP_LOGI(TAG, "     expected GPIO%d  %s", (int)cfg->miso,
             (in_gpio == (int)cfg->miso && use_mtx) ? "OK"
                 : "<-- MISO IS NOT WIRED TO THE PERIPHERAL");

    uint32_t miso_pad = reg_rd(IOMUX_PAD_REG(cfg->miso));
    ESP_LOGI(TAG, "  MISO pad GPIO%d IO_MUX raw=0x%08X input-buffer=%s",
             (int)cfg->miso, (unsigned)miso_pad,
             (miso_pad & FUN_IE) ? "ENABLED" : "DISABLED <-- reads always 0");

    ESP_LOGI(TAG, "  MOSI pad GPIO%-2d out_sel=%u (expect spid_out %u)",
             (int)cfg->mosi, (unsigned)(reg_rd(OUT_SEL_REG(cfg->mosi)) & 0x1FF),
             (unsigned)spi_periph_signal[SPI2_HOST].spid_out);
    ESP_LOGI(TAG, "  SCLK pad GPIO%-2d out_sel=%u (expect spiclk_out %u)",
             (int)cfg->sclk, (unsigned)(reg_rd(OUT_SEL_REG(cfg->sclk)) & 0x1FF),
             (unsigned)spi_periph_signal[SPI2_HOST].spiclk_out);

    spi_bus_free(SPI2_HOST);
}

/* ===================================================================
 *  Entry point
 * =================================================================== */

void bus_bringup_run(const bus_bringup_cfg_t *cfg)
{
    ESP_LOGW(TAG, "==== sensor SPI bring-up test ====");

    ESP_LOGW(TAG, "phase 1: pin toggle (2.5 Hz square wave, 4 s per pin)");
    toggle_pin("SCLK", cfg->sclk,    20);
    toggle_pin("MOSI", cfg->mosi,    20);
    toggle_pin("CS_I", cfg->cs_imu,  20);
    toggle_pin("CS_B", cfg->cs_baro, 20);

    ESP_LOGW(TAG, "phase 2: bit-banged ID read at ~50 kHz (peripheral bypassed)");
    bb_probe(cfg);

    ESP_LOGW(TAG, "phase 3: same read through the SPI peripheral, one knob at a time");
    periph_probe(cfg);

    ESP_LOGW(TAG, "phase 5: signal routing autopsy (no jumper needed)");
    routing_autopsy(cfg);

    ESP_LOGW(TAG, "phase 6: measured clock rate vs requested");
    clock_rate_probe(cfg);

    ESP_LOGW(TAG, "phase 7: what spi_bus_initialize() actually wired up");
    matrix_dump(cfg);

    if (cfg->run_loopback) {
        ESP_LOGW(TAG, "phase 4: peripheral loopback (MOSI->MISO jumper expected)");
        loopback(cfg);
    } else {
        ESP_LOGW(TAG, "phase 4: skipped -- set run_loopback once the jumper is on");
    }

    ESP_LOGW(TAG, "==== bring-up test done ====");
}
