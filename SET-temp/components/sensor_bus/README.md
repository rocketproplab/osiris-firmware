# sensor_bus — one bus layer for the BMP388 + LSM6DSOX

The barometer (J3) and the IMU (J4) share three wires. A single build-time
switch decides whether those wires run as I2C or as SPI. Every driver talks
through this component; none of them touch `spi_*` or `i2c_*` directly, and
`main.c` passes no pins, host or clock to either sensor.

| wire   | I2C build                        | SPI build         |
|--------|----------------------------------|-------------------|
| GPIO54 | clock (SCL)                      | clock (SCK)       |
| GPIO19 | data, both directions (SDA)      | data in (MOSI)    |
| GPIO18 | address select — driven LOW here | data out (MISO)   |
| GPIO14 | held HIGH (HIGH = "use I2C")     | BMP388 CS         |
| GPIO17 | held HIGH                        | LSM6DSOX CS       |

Pin numbers live at the top of `sensor_bus.c` and nowhere else. `main.c`
keeps them only as a commented-out entry in its board pin map.

## Status

**I2C works. SPI is not implemented.** The SPI branch of `sensor_bus.c` is
four stubs that return `ESP_ERR_NOT_SUPPORTED`; selecting SPI builds and
boots, but no sensor comes up. The Kconfig default is therefore I2C.

An earlier SPI transfer implementation is parked in an `#if 0` block at the
end of that branch. It was never confirmed working on the board — both chips
read back `0x00` while every transaction reported `ESP_OK` — so it is a
starting point, not a working driver. Delete the block once SPI is written
for real.

## Switching

    idf.py menuconfig      # SET sensor bus (BMP388 + LSM6DSOX) -> Interface
    idf.py build flash

or one line in `sdkconfig.defaults`: `CONFIG_SET_SENSOR_BUS_I2C=y`.

**Then remove all power — battery and USB — before testing.** The BMP388
locks itself into SPI the first time its CS goes low and stays locked until
it loses power. The reset button does not cut sensor power.

## What a good boot looks like

    I (sensor_bus) I2C scan: 2 device(s): 0x6A 0x76
    I (sensor_bus) BMP388 at 0x76
    I (sensor_bus) LSM6DSOX at 0x6A
    I (sensor_bus) sensor bus: I2C, 400000 Hz
    I (sensor_bus) [I2C] BMP3xx chip id 0x50 = BMP388  OK
    I (sensor_bus) [I2C] LSM6DSOX WHO_AM_I 0x6C  OK

The address scan runs once, at boot only — not on the recovery path, where
112 probe timeouts would hold the bus lock for most of a second. If it
prints nothing, the problem is wiring or power, not firmware.
`sensor_bus_check_ids()` is the real pass/fail test.

## When a transfer fails

All fault handling is in `bus_note_result()`, called after every transfer:

| condition                       | what happens                        |
|---------------------------------|-------------------------------------|
| any success                     | consecutive-error count resets       |
| 1–2 failures in a row           | counted, otherwise ignored           |
| 3 in a row                      | `i2c_master_bus_reset()` — clock SDA free |
| that reset fails                | full teardown and rebuild of the bus |
| rebuild fails                   | logged; retried no faster than every 250 ms |

The rebuild step matters. `i2c_master_bus_reset()` fails ("clear bus
failed") whenever a slave is still clamping SDA through all nine recovery
clocks, and after that the ESP-IDF driver returns `ESP_ERR_INVALID_STATE`
to everything forever — the old code treated only `ESP_ERR_TIMEOUT` as
recoverable, so one glitch bricked both sensors until reboot. Deleting the
device handles and the master bus and calling `bus_open()` again is the only
thing that clears that state.

Recovering is not the same as fixing. If you see repeated `bus rebuild
failed`, the fault is off-chip: the 3V3 rail, the pull-ups, wire length, or
a sensor holding SDA low. Check `sensor_bus_get_diag()` —
`errors` climbing against `xfers`, and `i2c_resets` incrementing, is the
signature.

## How main.c uses it

```c
#include "sensor_bus.h"

esp_err_t bus_err = sensor_bus_init();   /* before either sensor's init */
if (bus_err != ESP_OK) { /* refuse to continue */ }

sensor_bus_check_ids();   /* logs a verdict; deliberately does not gate boot */
```

Then each driver's config carries only its own interrupt pin — no `.host`,
no `.cs_pin`, no `.spi_freq_hz`. `main`'s `CMakeLists.txt` lists
`sensor_bus` in `REQUIRES`, as do `bmp388_sensor` and `lsm6dsox_sensor`.
