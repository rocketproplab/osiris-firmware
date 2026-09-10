# sensor-spi

SPI bring-up for an Adafruit LSM6DSOX breakout on a Waveshare ESP32-P4-ETH.
Resets the IMU, verifies `WHO_AM_I`, then streams accelerometer, gyroscope and
temperature at 4 Hz. Moving the board should move the numbers.

## Wiring

The breakout is silkscreened for I2C. In 4-wire SPI those pins take their
alternate functions:

| Breakout | SPI function | ESP32-P4 |
| -------- | ------------ | -------- |
| SCL      | SCLK         | GPIO54   |
| SDA      | MOSI (SDI)   | GPIO19   |
| DO       | MISO (SDO)   | GPIO18   |
| CS       | CS           | GPIO17   |
| I1       | INT1         | GPIO16   |
| I2       | INT2         | GPIO15   |

INT1/INT2 are wired but unused; this project polls. Pins live at the top of
`main/main.c`, which is the only board-specific file here.

## Build

```
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Expected output

```
I (xxx) ai_spi: host 1 up: sclk=54 mosi=19 miso=18
I (xxx) ai_spi: device on host 1: cs=17 mode=3 1000000 Hz
I (xxx) ai_lsm6dsox: initialised: WHO_AM_I 0x6C, ...
I (xxx) sensor-spi: accel [g]   0.002  -0.011   1.001 | gyro [dps] ...
```

## If WHO_AM_I does not read 0x6C

`0x00` or `0xFF` means the bus is not talking. In rough order of likelihood:

1. **Pin conflict.** GPIO54/19/18/17 were not checked against the
   ESP32-P4-ETH schematic; the Ethernet PHY's RMII interface takes a lot of
   pins. This is the first thing to rule out.
2. MOSI/MISO swapped — `SDA` is MOSI, `DO` is MISO.
3. SPI mode. The LSM6DSOX wants mode 3; see `SPI_MODE` in `main/main.c`.
4. Clock too fast for the breakout's I2C pull-ups. `SPI_CLOCK_HZ` starts at
   1 MHz, well under the part's 10 MHz limit.

## Components

- `ai_spi` — board-agnostic SPI master wrapper with register read/write
  helpers. Reference-counts the bus so several devices can share a host.
- `ai_lsm6dsox` — bus-agnostic LSM6DSOX driver. Reaches the part through a
  two-callback transport, so an I2C backend drops in without touching it.

`main.c` supplies the glue between the two.
