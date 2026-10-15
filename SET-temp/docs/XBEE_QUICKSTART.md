# XBee link quickstart

How to get telemetry down and commands up: the radio settings, the ground
station and the link emulator. For every other menuconfig option see
`docs/menuconfig_options.md`.

## The link

```
flight computer --UART2 @ 115200--> vehicle XBee-PRO S3B ~~ air ~~ ground XBee --USB--> laptop
                                                                                       ground_station.py
```

- **Downlink:** telemetry packets (magic `0xA5`). About 2 per second in
  flight, one every 5 s on the pad.
- **Uplink:** commands (`0x5A`, e.g. ARM) and deployment config chunks (`0x5B`).
- The UART pins are `PIN_TX_XBEE`, `PIN_RX_XBEE` and `PIN_RST_XBEE` near the top
  of `main/main.c`. The baud is `XBEE_BAUD_RATE` there and must equal the
  module's `BD`.

## 1. Set up the radios (XCTU, once)

Both radios need the same **network ID (`ID`)**, **preamble (`HP`)** and **baud
(`BD` = 115200)**.

| Radio | `AP` | Other |
|---|---|---|
| Ground | **1 (API)**, always | — |
| Vehicle, normal | **1 (API)** | — |
| Vehicle, backup | **0 (transparent)** | `DH`/`DL` = ground radio's address, or leave at broadcast (default). Leave `RO` and `GT` at their defaults. |

Write the settings with `WR` before unplugging.

## 2. menuconfig (vehicle firmware)

`idf.py menuconfig` → **Component config → XBee telemetry link**

| Option | Default | Set it to |
|---|---|---|
| XBee serial mode: `TELEMETRY_XBEE_API` / `TELEMETRY_XBEE_TRANSPARENT` | API | **Whatever the vehicle module's `AP` is.** If they disagree, nothing gets through. |
| `TELEMETRY_XBEE_TRANSPARENT_VERIFY` | on | Transparent only. Checks the radio at boot with `+++` / `ATVR` (about 2.5 s). Leave it on. |
| `TELEMETRY_XBEE_GUARD_MS` | 1100 | Transparent only. Must be longer than the module's `GT` (1000 ms default). |

Then `idf.py build flash monitor`.

**Read the boot log:**

| You see | Meaning |
|---|---|
| `radio verified: ATVR = 0x....` | API mode, radio answering. Good. |
| `radio verified (transparent): ... DH:DL = ...` | Transparent mode, radio answering. It also shows where packets will go. |
| `WRONG XBEE MODE` | The module's `AP` and the menuconfig setting disagree. The message says which way round. Fix one of them. |
| `RADIO NOT VERIFIED` | No answer in either mode. Check power, baud, wiring. The board still boots and logs to SD. |

The firmware never changes the module's settings itself; that's always done in
XCTU.

## 3. Ground station

```
pip install pyserial
python tools/ground_station.py                       # GUI; type the port, press Connect
python tools/ground_station.py --port COM5           # GUI, connect straight away
python tools/ground_station.py --port COM5 --headless          # terminal only
python tools/ground_station.py --replay sessions/<dir>/raw.bin --headless
```

- **Port:** the ground XBee's COM port, or `socket://127.0.0.1:9750` for the
  emulator (below). Other flags: `--baud` (115200), `--dest <hex>` (uplink
  address, default broadcast), `--out <dir>`, `--no-capture`.
- **Buttons:**
  - **ARM**, **DISARM**, **PING** (asks for a status record now; the easiest
    link check), **MARK** (writes a marker into the vehicle's event log), and
    **EJECT SD** (only in STANDBY/LANDED; one way until a power cycle).
  - **Flight parameters...** edits the deployment config, then *Send to
    vehicle* and *Apply on vehicle*.
  - **Plots...** shows accel, gyro and GPS.
- **Confirmation comes from the vehicle**, not the button: watch the log for
  the vehicle's `UPLINK_COMMAND` event, or the mode changing.
- **Every session** is saved to `sessions/<timestamp>/`: one CSV per stream,
  plus `raw.bin` with every byte received (replay it with `--replay`).
- **Vehicle in transparent mode:** nothing to change on the ground. If the log
  says *"reassembling"*, that's expected: the vehicle's radio sometimes splits
  or merges packets, and the station puts them back together.

**Commands get ignored as "STALE"?** Every command carries a sequence
number that must keep increasing. The ground keeps its counter in
`tools/.uplink_seq`. The vehicle forgets its own on reboot, so power-cycling
the flight computer always clears this.

**Without the GUI:**

```
python tools/uplink_command.py --port COM5 ping
python tools/uplink_command.py --port COM5 mark --arg 3
python tools/decode_telemetry.py --port COM5                 # live summary + CSVs
python tools/decode_telemetry.py sessions/<dir>/raw.bin      # decode a capture
python tools/decode_telemetry.py capture.bin --transparent   # capture off the vehicle UART in transparent mode
```

## 4. Link emulator (no radios needed)

`xbee_link_emulator.py` pretends to be **both** XBees, so the flight computer
and the ground station talk over a USB-serial adapter.

**Wiring** (SparkFun XBee Explorer with no XBee fitted, or any 3.3 V
USB-serial adapter). Cross TX/RX:

| Adapter | Board |
|---|---|
| DOUT / RX into the PC | the board's XBee TX pin (`PIN_TX_XBEE`, goes to J5 DIN) |
| DIN / TX out of the PC | the board's XBee RX pin (`PIN_RX_XBEE`, comes from J5 DOUT) |
| GND | GND |
| 3V3 | leave unconnected |

**Run, in this order:**

```
python tools/xbee_link_emulator.py --vehicle-port COM4            # firmware in API mode
python tools/xbee_link_emulator.py --vehicle-port COM4 --vehicle-transparent   # firmware in transparent mode
python tools/ground_station.py --port socket://127.0.0.1:9750
```

Then **reset the flight computer**, so its boot check is answered by the
emulator. The emulator prints a status line every 5 s: packets each way, how
many were lost, bad checksums.

Useful flags:

| Flag | Does |
|---|---|
| `--vehicle-transparent` | Vehicle side acts like an `AP=0` module, including `+++` command mode. Use it with a firmware built for transparent mode. |
| `--drop 0.2` | Lose 20 % of over-the-air packets, to test lost-packet handling. |
| `--capture file.bin` | Save everything the ground side receives. |
| `--listen 127.0.0.1:9750` | Where the ground station connects. |
| `--max-payload 256` | Largest RF packet, like the module's `NP`. |

The ground side of the emulator is always API mode, same as the real ground
radio.


