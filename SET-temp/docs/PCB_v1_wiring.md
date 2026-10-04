# Osiris Avionics PCB v1 — wiring reference

Source: `SET_AV.kicad_pcb` / `SET_AV.kicad_sch`, 2026-09-03.
Every row below is read out of the board netlist.

## Module placement

The ESP32-P4-ETH is placed **rotated 180°** relative to Waveshare's
pinout drawing. Footprint pad 1 is at the bottom of the board and pad 21
at the top, so:

- pads 1–20 (inner column, bottom→top) = Waveshare's *left* column, in order
- pads 21–40 (outer column, top→bottom) = Waveshare's *right* column, reversed

The board's auto-generated net names encode the GPIO directly —
`Net-(J1-54)` on pad 1 is GPIO54 — which is what makes this checkable
rather than inferred.

## Sensor SPI bus — J3 (BMP388) + J4 (LSM6DSOX)

The breakouts label their SPI pins with I²C names. SCL is the clock,
SDA is data *into* the sensor, SDO/DO is data *out* of it.

| Signal | GPIO | Pad | Goes to |
|---|---|---|---|
| SCK  | 54 | J1.1  | J3.4 SCL, J4.4 SCL |
| MOSI | 19 | J1.2  | J3.6 SDA, J4.5 SDA |
| MISO | 18 | J1.4  | J3.5 SDO, J4.6 DO |
| CS — BMP388   | 14 | J1.9  | J3.7 CS |
| INT — BMP388  | 6  | J1.10 | J3.8 INT |
| CS — LSM6DSOX | 17 | J1.5  | J4.7 CS |
| INT1 — LSM6DSOX | 16 | J1.6 | J4.8 I1 |
| INT2 — LSM6DSOX | 15 | J1.7 | J4.9 I2 — hardware liftoff latch → flight mode |

## GPS — J6 (Adafruit PA1616D, JST-XH)

TX/RX named from the ESP32's side.

| Signal | GPIO | Pad | Goes to |
|---|---|---|---|
| TX (ESP→GPS) | 27 | J1.27 | J6.4 RX |
| RX (GPS→ESP) | 32 | J1.26 | J6.5 TX |
| PPS | 26 | J1.29 | J6.1 PPS |
| EN  | 33 | J1.25 | J6.6 EN — wired, unused by firmware |

J6.2 = 3V3, J6.3 = GND.

## Telemetry — J5 (XBee PRO S3B)

| Signal | GPIO | Pad | Goes to |
|---|---|---|---|
| TX (ESP→XBee) | 3 | J1.14 | J5.4 DIN |
| RX (XBee→ESP) | 4 | J1.12 | J5.3 DOUT |
| RST | 2 | J1.15 | J5.6 RST — wired, unused |
| CTS | 5 | J1.11 | J5.14 CTS — wired, unused |

R10 is a 10k pull-up from 3V3 onto GPIO3, holding the XBee's DIN at the
UART idle level while the ESP32 pin is still high-Z during boot.
J5.1/J5.11 = GND, J5.2 = 3V3.

## Blue Raven discrete inputs — J7 (recovery), J8 (staging)

Both are **inputs**, both from the Blue Raven altimeter. Each BR output
is split two ways: one branch to the actuation hardware, one to this
board. We tap the signal; we never drive it. Identical divider on each:
4.7k series, 3.6k to ground, giving ×0.434 at the pin.

| Signal | GPIO | Pad | Path |
|---|---|---|---|
| Staging  | 22 | J1.32 | J8.2 → R6 4.7k → GPIO22, R5 3.6k → GND |
| Recovery | 23 | J1.31 | J7.2 → R9 4.7k → GPIO23, R8 3.6k → GND |

The 8.3 kΩ divider is a parallel load on the actuation net (~0.9 mA at
7.4 V). If that branch is inductive, its flyback appears here too —
the divider passes 43.4% of it, so a spike over ~7.6 V exceeds the pin's
3.3 V limit. Check for a coil.

J7.1 and J8.1 are tied to ground through 1k (R7, R4).

**Voltage window.** The P4 needs ~2.475 V to read HIGH and must stay
under 3.3 V, so through ×0.434 the source has to sit between about
**5.7 V and 7.6 V**:

| BR output | At the pin | Result |
|---|---|---|
| 3.3 V | 1.43 V | never fires |
| 5.0 V | 2.17 V | never fires, or marginal |
| 7.4 V | 3.21 V | correct |
| 12 V  | 5.21 V | over-voltage on the pin |

Sized for a 2S pack, consistent with the battery sense divider and with
the fact that these outputs must also drive actuation. **Measure the
real BR outputs before flight.**

No internal pulldown is enabled on either pin: R5/R8 already hold the
line LOW when the BR output is tri-stated, and the internal pulldown
(~45k) in parallel would shift the ratio to 0.415 and make the trigger
*harder* to reach.

## Battery sense

BT1+ → R1 6.8k → node → R2 2.7k → GND (×0.284), then R3 1k into
GPIO20 with C2 100nF to ground.

GPIO20 is **ADC1_CH4**. 3.3 V full scale = 11.6 V at the battery.
Wired and correct; the firmware does not read it yet.

## Power

BT1+ → J2.3 (AP63203 buck VIN) → J2.1 3V3 → J1.36 and every peripheral
header. J2.4 (EN) unconnected. Bulk/decoupling: C1, C5, C7 47µF;
C3, C4, C6, C8 100nF.

## SD card

On the module, via its own SDMMC slot on GPIO39–44. Never reaches the
header, so `sd_logger.c`'s constants need no change and cannot collide
with anything above.

## What the board does NOT have

- **No deployment/pyro outputs.** The Blue Raven fires the charges.
- **No staging or recovery output.** J7 and J8 are input only — the
  board observes the Blue Raven, it does not command it.

Free header pads for a future revision: 22 (GPIO47), 24 (GPIO46),
34 (GPIO21), 21 (GPIO48). Avoid pads 16/17 (GPIO8/7, module-internal
I²C) and 19/20 (GPIO25/24, USB).

## Full header map

| Pad | Function | Net | Pad | Function | Net |
|---|---|---|---|---|---|
| 1 | GPIO54 | SCK | 21 | GPIO48 | — |
| 2 | GPIO19 | MOSI | 22 | GPIO47 | — |
| 3 | GND | GND | 23 | GND | GND |
| 4 | GPIO18 | MISO | 24 | GPIO46 | — |
| 5 | GPIO17 | IMU CS | 25 | GPIO33 | GPS EN |
| 6 | GPIO16 | IMU INT1 | 26 | GPIO32 | GPS TX→ESP |
| 7 | GPIO15 | IMU INT2 | 27 | GPIO27 | ESP→GPS RX |
| 8 | GND | GND | 28 | GND | GND |
| 9 | GPIO14 | BMP CS | 29 | GPIO26 | GPS PPS |
| 10 | GPIO6 | BMP INT | 30 | RUN | — |
| 11 | GPIO5 | XBee CTS | 31 | GPIO23 | BR recovery |
| 12 | GPIO4 | XBee DOUT→ESP | 32 | GPIO22 | BR staging |
| 13 | GND | GND | 33 | GND | GND |
| 14 | GPIO3 | ESP→XBee DIN | 34 | GPIO21 | — |
| 15 | GPIO2 | XBee RST | 35 | GPIO20 | VBAT sense |
| 16 | GPIO8 | internal SCL | 36 | 3V3 | 3V3 in |
| 17 | GPIO7 | internal SDA | 37 | EN | — |
| 18 | GND | GND | 38 | GND | GND |
| 19 | GPIO24 | USB DM | 39 | VSYS | — |
| 20 | GPIO25 | USB DP | 40 | VBUS | — |
