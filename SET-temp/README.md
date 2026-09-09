# SET Rocket Flight Computer (ESP32-P4)

## Build

```
idf.py set-target esp32p4
idf.py build flash monitor
```

## Architecture

```
BMP388  ─┐
LSM6DSOX ┤─ each source's own task pushes directly ─→ data_router_publish()
PA1616D  ┤                                                  │
VBAT ADC ─┘                                                 │
                                          ┌─────────────────┴─────────────────┐
                                     sd_logger                          telemetry
                                  (7 ring buffers, one                (1 ring buffer,
                                   per stream, each                    all streams mixed,
                                   sized to its record)                blocks <-> XBee)
                                          │                                  │
                                  sd_logger_task                       uplink commands
                                  → baro.bin / imu.bin / gps.bin             │
                                    batt.bin / err.bin / event.bin           ▼
                                    status.bin                          flight_mode
                                                                             ▲
                                                          LSM6DSOX INT2 wake-up
                                                          (the no-radio fallback)

flight_logic  ── separate task, PACED BY THE IMU; reads *_get_latest() directly
                 (NOT via router). Two timelines inside: gyro/accel every tick,
                 barometer only when baro.seq changes.
discrete_in   ── two GPIO ISRs → two tasks (staging J8, recovery J7),
                 independent of everything above
event_log     ── errors + flight events → data_router → err.bin / event.bin
flight_mode   ── STANDBY / ARMED / FLIGHT / LANDED; every driver reads it to
                 pick its sample rate. Nothing writes it except the two
                 triggers above and flight_logic's landing detect.
status_task   ── 1 Hz snapshot of mode, battery, queue health, link → status.bin
```

Flight logic is deliberately outside the router: it needs *current* sensor
state every tick, not a queued backlog.

## Tasks (11 total)

| Task | Priority | Core |
|---|---|---|
| `staging` / `recovery` (discrete_in) | 11 | 0 |
| `bmp388_task` / `lsm6dsox_task` / `gps_task` | 9 | 0 |
| `flight_logic_task` | 7 | 1 |
| `telemetry_task` | 5 | 1 |
| `sd_logger_task` | 3 | 1 |
| `battery_task` | 2 | 1 |
| `status_task` | 2 | 1 |
| `print_task` | 1 | 1 |

Priority ordering is enforced by `_Static_assert` in `main.c` — a
misordered priority is a build error, not a flight-time surprise.

## Per-sensor queue depths

Derived from each sensor's own rate:
`rate_hz × (SD_LOGGER_FLUSH_MS + SD_QUEUE_MARGIN_MS) / 1000`

| Stream | Rate | Depth |
|---|---|---|
| IMU | 104 Hz | 156 |
| Baro | 50 Hz | 75 |
| Batt | 10 Hz | 15 |
| GPS | 1 Hz | 8 (floored from 1) |
| Err | — | 32 (fixed) |
| Event | — | 16 (fixed) |
| Status | 1 Hz | 8 (fixed) |

Err, Event and Status are not rate-derived, because none of them has a
rate worth deriving from. Errors are bursty — zero for a whole flight, then
one per sample the moment a sensor starts failing. Events are a couple of
dozen records total.

Separate buffers mean a fast sensor can only ever evict **its own**
backlog, never another sensor's.

### Each buffer's element is that stream's record, not a shared envelope

`queue_item_t` — a 1-byte stream tag, a 1-byte length, and a payload padded to
the largest record in the project, 42 bytes in total — is the right shape for
`telemetry`, which owns *one* buffer carrying every stream mixed together and
therefore has to hold the biggest thing it might see and label it.

It is the wrong shape here. `sd_logger` has seven buffers, one per stream, and
`rb_for_stream()` picks the buffer *before* the push — so by the time a record
reaches `baro_rb` the stream is already known and the length is implied by it.
Both header bytes were re-stating what the buffer's own identity already said,
and the payload was padded to 40 bytes to hold a record that is 24.

| Stream | Depth | Record | ×42 (was) | ×record (now) |
|---|---|---|---|---|
| IMU | 156 | 36 | 6552 | 5616 |
| Baro | 75 | 24 | 3150 | 1800 |
| Err | 32 | 24 | 1344 | 768 |
| Event | 16 | 20 | 672 | 320 |
| Batt | 15 | 20 | 630 | 300 |
| GPS | 8 | 34 | 336 | 272 |
| Status | 8 | 32 | 336 | 256 |
| | | | **13020** | **9332** |

3688 bytes recovered, 28%, with no change in depth. Telemetry's single mixed
buffer keeps `queue_item_t` unchanged, because it genuinely needs it.

The cost is that element size becomes a per-buffer contract nothing re-checks at
runtime: `ring_buffer_push()` copies exactly `elem_size` bytes from the pointer
it is handed, so a caller passing a shorter object than the stream implies would
have bytes read from past the end of it. `sd_logger_write()` therefore requires
`len == sensor_record_size(stream)` **exactly** and returns
`ESP_ERR_INVALID_SIZE` otherwise — under the old envelope a wrong length merely
produced a short record on the card; here it would be an out-of-bounds read.


---

## Pin map — from the board, not guessed

All pin assignments now come from `SET_AV.kicad_pcb` (Osiris Avionics
PCB v1, 2026-09-03). See `docs/PCB_v1_wiring.md` for the full table.

The ESP32-P4-ETH module is placed **rotated 180°** relative to
Waveshare's pinout drawing: footprint pad 1 is at the bottom of the
board, pad 21 at the top. The board's own net names (`Net-(J1-54)` etc.)
already encode the GPIO number, which is what makes the map checkable.

## UNVERIFIED — confirm before flight

**Blue Raven signal voltage** (`discrete_in.h`) — both discretes arrive
through a 4.7k/3.6k divider (×0.434), which only yields a valid logic
HIGH for a source between roughly **5.7 V and 7.6 V**. A 3.3 V or 5 V
logic-level output will never trigger; much above 7.6 V over-drives the
pin. The divider is sized for a 2S pack, consistent with the battery
sense divider and with the fact that these outputs also have to drive
actuation hardware — a good argument, not a measurement. **Measure both
lines before flight.** Input *polarity* is confirmed (LOW-to-HIGH).

**Flyback on the shared node** — if the actuation branch is inductive,
switching it spikes the net both branches share. Check for a coil; if
there is one, the tap wants a clamp.

**No deployment outputs on this board.** J7 and J8 are inputs. The Blue
Raven fires every charge. `flight_logic` no longer has pin defines, a
`gpio_pin` field, or a single call that can drive a level — see *Flight
logic is a second opinion* below. Adding real outputs is a board
revision, and they should land in a new component rather than back in
the evaluator; free header pads are 22 (GPIO47), 24 (GPIO46), 34
(GPIO21) and 21 (GPIO48).

**No staging or recovery output.** Both `discrete_in` channels have
`output_pin = GPIO_NUM_NC`; the component skips output configuration.

**Discrete output polarity** (`discrete_in.h`) — idle LOW, driven HIGH.
Only matters if a later revision adds the pin.

**`gps_reconfigure()`** (`gps_sensor.c`) — PMTK command strings are
from published MTK/Adafruit docs but have never been run against real
hardware. Also: the moment the baud command is sent, the module
switches immediately. If the local UART reconfiguration that follows
fails, the link is dead until power-cycle.

**LSM6DSOX wake-up registers** (`lsm6dsox_sensor.c`) — the hardware
liftoff detector's register values follow the datasheet but have not
been confirmed on this board. Bench-test it: a sharp downward jerk
should exceed 3 g and latch INT2 high. If it never fires, suspect
`SLOPE_FDS` (high-pass vs slope filter) or the threshold scaling first.
This now matters more than it did: INT2 is the *only* way into flight
mode if the radio is down, so an untested wake-up block is an untested
fallback.

**The wake-up threshold in STANDBY.** `WAKE_UP_THS` is scaled against the
accelerometer's full scale, which is ±16 g in both modes — so the 3 g
threshold does not move when the ODR drops to 12.5 Hz. What *does* move
is the sampling: at 12.5 Hz the required 2-sample duration is ~160 ms
rather than ~19 ms. A real launch clears that easily; a short sharp knock
on the bench may not. Bench-test the detector **in standby**, not only at
flight rates.

**Standby rates are a guess at the power budget, not a measurement.**
12.5 Hz / 1.5 Hz / 1 Hz were chosen to be obviously slow while still
proving every sensor is alive. Nobody has measured what the board
actually draws in either mode. Measure it before you plan a pad session
around it.

**Unused by the firmware, but wired:** `PIN_CTS_XBEE` (GPIO5). CTS
matters now that telemetry is real — the XBee raises it when its transmit
buffer fills, and a host that ignores it drops bytes silently.

`PIN_RST_XBEE` (GPIO2) is no longer in this list: it drives the module's
reset in `telemetry_init()` — see the bring-up sequence above.

**`sizeof(StaticTask_t)` / `sizeof(StackType_t)` / `sizeof(portMUX_TYPE)`**
— left blank in the memory spreadsheet; these depend on your
`FreeRTOSConfig.h`. Print them in your own build.

**Stack sizes** — none measured. Run
`uxTaskGetStackHighWaterMark(NULL)` in each task before trimming.

## Battery monitoring

`components/battery_monitor` samples the 6.8k/2.7k divider on GPIO20
(ADC1_CH4) at 10 Hz, averaging 16 raw conversions per sample, and
publishes each one to the router as `STREAM_BATT` — so it lands in
`batt.bin`, separate from every sensor stream.

Separate on purpose. It samples at its own rate, unrelated to any
sensor's, and the question it answers ("was the pack sagging when the
log went strange?") is only answerable if the answer survives whatever
went strange. A brownout that corrupts `imu.bin` should still leave
`batt.bin` intact and explaining why.

Both `battery_v` and the raw `adc_v` at the pin are logged. If the
divider constant turns out wrong, the flight is still recoverable
offline — `battery_v = adc_v / ratio`.

Thresholds (7.2 V warn, 6.6 V critical, sized for 2S) are **logged
only**. Deciding what to do about a low pack is flight logic's job, not
a monitor's.

## Error and event streams

`err.bin` is "what went wrong". Every component reports through
`event_log_error()`, which rate-limits per source (100 ms, after a
3-record burst allowance) so a sensor failing at 104 Hz cannot fill the
queue with 156 copies of one fact. Suppressed errors are not lost —
they are counted, and the next record from that source carries the tally
in `suppressed`. A large number there is the difference between one
glitch and continuous failure.

`event.bin` is "what happened, and when": boot, SD mount, staging
signal, recovery signal, hardware liftoff, software liftoff, apogee,
burn start/end, deploy predicted, mode changes, uplink commands. Never
rate-limited — events are rare by construction and
dropping one would corrupt the timeline that is the whole point.

They are separate files for the same reason they have separate queues:
an error burst must never bury the timeline in exactly the situation
where the timeline matters most.

`event_log` takes its sink as a function pointer rather than calling
`data_router` directly. That is not indirection for its own sake —
`sd_logger` reports its own faults through it, and
`sd_logger → event_log → data_router → sd_logger` is a dependency cycle
the build system rejects. One consequence worth knowing: `sd_logger`
reports queue drops from its *writer task*, never from
`sd_logger_write()`, because reporting from the push path would recurse
without bound on a full queue.

## Blue Raven discrete inputs

Two lines from the Blue Raven altimeter, electrically identical, both
through a 4.7k/3.6k divider:

| Line | Connector | GPIO | Event |
|---|---|---|---|
| Staging | J8 | 22 | `EV_STAGING_SIGNAL` |
| Recovery | J7 | 23 | `EV_RECOVERY_SIGNAL` |

**This board is a tap, not a participant.** Each Blue Raven output is
split two ways — one branch drives the actuation hardware that does the
work, the other comes here. We listen; we never drive these nets. Three
things follow:

- **Never drive these pins.** Back-driving a net that is also wired to
  actuation hardware is the class of mistake that fires something. The
  component's `output_pin` drives a *separate* pin, never this one.
- **The divider is a parallel load** on the actuation net — 8.3 kΩ
  across the signal, about 0.9 mA at 7.4 V. Negligible against anything
  that fires a charge, but not zero.
- **The signal's voltage is set by what actuation needs**, not by what
  suits us. That is the best evidence we have that it sits at battery
  level — which is what the divider assumes — but it is evidence, not a
  measurement.

Worth checking separately: if the actuation branch is inductive (a
relay, a solenoid, any coil), switching it puts a flyback spike on the
shared node. The divider passes 43.4% of whatever appears there, so a
spike above ~7.6 V drives the ESP32-P4 pin past 3.3 V. If there is a
coil on that net, it wants a clamp.

The component is `discrete_in`, named for what it does rather than for
who drives it — the provenance of these lines was restated twice during
development, and a component named after its data source needs renaming
every time that happens. Each channel carries a `source` string instead,
logged at init, where provenance can be corrected without touching a
symbol. It began as `staging.c` ("there is exactly one staging line on
this vehicle"); a second identical line made that false, so it is now a
static pool with one instance per line — the same idiom the sensor
drivers use. A third discrete costs a config struct, not a component.

Each channel's ISR captures the timestamp and its task records the
event with *that* time, not the time the task ran — the value of the
record is knowing when the Blue Raven asserted the line, and a time
taken after scheduling would be measuring FreeRTOS instead.

## Telemetry

Records go into one queue, get packed into a block, and the whole block
goes out as a single XBee packet — the same "buffer, then write once"
trade `sd_logger` makes. One 200-byte packet costs one set of framing
and one over-air transaction; forty 5-byte packets cost forty.

A block flushes on **200 bytes** or **500 ms**, whichever comes first.
The timeout is what stops a slow stream being held hostage: a 1 Hz GPS
fix behind a size-only rule would wait for the IMU to fill the block,
and on a quiet link it might wait forever.

**IMU is decimated 1-in-4 for the radio only.** The card keeps every
sample; the radio carries ~26 Hz. Decimation happens in
`telemetry_write()`, so nothing else in the project is affected.

### Packet format

Little-endian, matching the records. (The XBee's own frame fields
around it are big-endian — Digi's convention, and it stops at the API
layer.)

```
 0  uint8   magic      0xA5
 1  uint8   ver_flags  high nibble version, low nibble flags (bit0 = retransmit)
 2  uint16  seq        packet sequence, wraps at 65535
 4  uint8   rssi       -dBm magnitude at the vehicle, 0 = unknown
 5  uint8   count      records in this packet
 6  uint16  len        payload bytes following the header
 8  records            [uint8 stream][record bytes] x count
    uint16  crc        CRC16-CCITT over everything above
```

Records carry a 1-byte stream tag and **no length** — the length is
implied by the stream via `sensor_record_size()`. That saves a byte per
record, and the version nibble is the guard: bump it if any record's
size changes, or an old decoder reads new data as plausible garbage.

Overhead is 28 bytes per packet (18 XBee + 8 header + 2 CRC) plus 1 byte
per record. A full 200-byte block is ~12% overhead.

### Two integrity layers

The XBee's own 8-bit frame checksum catches UART-level damage; our
CRC16 catches anything that got through the radio intact but corrupt.
Both are exercised by `tools/decode_telemetry.py`, which reports each
separately — a run of CRC failures with a clean checksum count means a
marginal link rather than a wiring problem.

### Retry

Sequence numbers for ground-side accounting, plus bounded retry on a
confirmed TX failure. `TELEMETRY_RETRY_SLOTS` (4) packets can be
in flight un-acked, each with `TELEMETRY_MAX_RETRIES` (1) re-send.
Bounded on purpose: an unbounded queue of un-acked packets on a dead
link would stall everything behind packets nobody will hear, and on a
rocket the next fresh packet is worth more than a perfect record of an
old one. When every slot is full the **oldest** is abandoned.

### What the RSSI field means

It is read from the module's `ATDB`, which reports the strength of the
last packet **the vehicle received**. It describes the *uplink*, and
only means anything if the ground station transmits periodically. It is
**not** a measure of how well the ground station hears the rocket —
that can only be measured at the ground station.

### Before it will work

The module must already be in **API mode (AP=1)** at a matching baud.
That is deliberately not done in firmware: switching an XBee into API
mode means `+++`, a guard-time wait, `ATAP1`, `ATWR`, and if any of it
half-succeeds you have a radio in an unknown state at an unknown baud.
Configure it once on the bench with XCTU.

`telemetry_init()` now **verifies** rather than assumes. In order: bring
the UART up so TX (the module's DIN) idles high; pulse the module's
`/RESET` (GPIO2, wired since PCB v1 and unused until now); send `ATVR`
and wait for the response. Only if nothing answers does it send the
bootloader escape sequence and probe again.

The order is the point. Digi's bootloader is entered by a serial **break**
on DIN across a reset — DIN held low longer than a character time while
the module boots. An ESP32 pad is an input until firmware configures it,
so between board power-on and `uart_set_pin()` DIN is held only by R10's
10k pull-up. Resetting the module *after* the UART is driving DIN high
gives it a clean boot, which is why the reset comes second and not first.

`diag.radio_verified` carries the result and `STATUS_FLAG_TELEM_OK` now
reports it — that flag used to mean "the handle is non-NULL", which is
true whenever the UART opened and says nothing about whether a radio is
on the other end. A module left in transparent mode, or absent, fails the
probe, logs `RADIO NOT VERIFIED`, and **still boots**: a vehicle that
refuses to fly because its downlink is deaf has traded a degraded flight
for no flight, and the card is the record that matters.

`diag.bootloader_escapes` is the number worth watching. Non-zero means
the module *was* in its bootloader at power-on despite the clean reset,
which points at DIN rather than at firmware.

If your module's bootloader wants a different key, `XBEE_ESCAPE_BYTES` in
`telemetry.h` is the one line to change. Nothing else depends on it —
success is judged by whether `ATVR` answers afterwards, never by parsing
the banner.

Set `dest_addr` to the ground station's 64-bit address once known.
Broadcast gets no per-packet delivery ack on DigiMesh, so the retry path
has nothing to trigger on, `packets_acked` stays 0, and `last_rssi` only
becomes meaningful once the ground station transmits (ATDB reports the
strength of the last packet the *vehicle* received).

**CTS is still not honoured.** GPIO5 is wired to the XBee's CTS but not
handed to the UART driver, so a burst that fills the module's buffer
drops bytes silently. Enabling hardware flow control is the fix.

### Seeing it on the ground

```
python tools/decode_telemetry.py --port COM5          # live, Ctrl-C to stop
python tools/decode_telemetry.py --port COM5 --raw flight.bin
```

Live mode prints a vehicle summary that refreshes in place — packets,
lost, CRC failures, mode, altitude AGL, battery, RSSI, SD and telemetry
drop counts, and how long since the last packet — scrolls flight events
as they arrive, and writes the same CSVs the file mode does on Ctrl-C.
`--raw` also appends every byte the radio produced, so a session can be
re-decoded later without re-flying.

Both radios must match on **baud (115200)**, **API mode (AP=1)**,
**network ID (`ID`)** and **preamble ID (`HP`)**. If nothing decodes, the
tool prints that checklist in likelihood order.

## Flight logic is a second opinion, not a trigger

This board fires nothing. The Blue Raven fires every charge, and each of
its outputs is split two ways — one branch to the actuation hardware, one
through a divider to J7/J8 on this board. The ESP is a listener on a line
somebody else drives.

So `flight_logic` computes what it *would* have done, from this board's
own barometer and IMU, and writes that to `event.bin` with its own
timestamps. After a flight you lay the two timelines side by side. When
they agree you have gained confidence; when they disagree you have found
something, and you found it on the ground.

There are no pin defines in the component, no `gpio_pin` in
`deploy_channel_t`, and no call in the file that can drive a level. That
is not an omission waiting to be filled in — if a board revision gains
real pyro channels, the actuation wants its own component with its own
arming interlocks and continuity checks, so the thing that decides and
the thing that fires can be reviewed, and can fail, independently. The
record it writes is `EV_DEPLOY_PREDICTED` (renamed from
`EV_DEPLOY_FIRED`, same numeric value, so old logs still decode) because
a record named "fired" on hardware that cannot fire is a trap for
whoever reads the log next.

### The loop is paced by the IMU

`flight_logic_task` blocks on `lsm6dsox_wait_for_sample()` and takes whatever
barometric sample is current. Tilt is computed by integrating angular rate, and
an integrator is only as good as its sampling — pacing on the 50 Hz barometer
meant point-sampling a 104 Hz gyro stream at a rate that was neither
synchronous with it nor an integer division of it. Some gyro samples were read
twice, others never read at all, and the rotation during a skipped sample was
attributed to a neighbouring rate. Reading every gyro sample removes that class
of error outright.

**The barometer became the oversampled input, and that had to be handled.** At
104 Hz against a 50 Hz barometer, half the ticks carry an altitude identical to
the previous tick. A single shared Δt would divide zero altitude change by the
IMU interval on those ticks and the full change by the same interval on the
others — a velocity estimate alternating between `0` and `2×` the truth, which
looks like violent oscillation and averages to the right answer. `VEL1`, `VEL2`
and `BVEL1` would all have become unusable.

So the estimator keeps **two timelines**:

| Advances | Estimators | Interval |
|---|---|---|
| every tick | gyro bias, tilt, burn count, liftoff | IMU Δt (10 ms flight, 80 ms standby) |
| only when `baro.seq` changes | altitude, velocity, apogee, `PINC` | baro Δt (20 ms flight, 667 ms standby) |

Freshness is decided by sequence number, never by comparing altitudes — two
consecutive readings of the same altitude are ordinary, especially on the pad.

**Every dwell is a duration, not a sample count.** `LIFTOFF_SUSTAIN_MS` 60,
`BURN_ENTER_MS` 60, `BURN_EXIT_MS` 100, each with a 2-sample floor so one
outlier cannot satisfy a dwell. The IMU runs 12.5 Hz in standby and 104 Hz in
flight; a sample count would have meant anything from 29 ms to 240 ms depending
on a mode this component cannot see.

### The config, and what every event means

Channels are configured by a JSON blob in NVS, uploaded by the host app,
in the format the Blue Raven config editor already produces — four
channels (`Apo`, `Main`, `3rd`, `4th`), each with a `primary` and a
`secondary` set of enabled events, a `thresholds` map, `delay_ms`,
`latch` and `manual_arm`. A channel triggers when **every** event in
either set is true, and stays true for `delay_ms`.

The `1`/`2` suffix is a comparison direction, not two quantities: **1 is
below the threshold, 2 is above it.** `AGL1`/`AGL2` and `VEL1`/`VEL2`
already worked that way and the newly implemented events follow it.

| Event | True when | Units |
|---|---|---|
| `AGL1` / `AGL2` | altitude above the pad below / above threshold | feet |
| `FANG1` / `FANG2` | tilt from vertical below / above threshold | degrees |
| `FUTANG` | *projected* tilt at `delay_ms` from now, below threshold | degrees |
| `TVAL1` / `TVAL2` | time since liftoff below / above threshold | seconds |
| `VEL1` / `VEL2` | \|vertical speed\| below / above threshold | ft/s |
| `BVEL1` | signed vertical speed below threshold (descending) | ft/s |
| `BURNCOUNT` | motor burns started ≥ threshold | count |
| `LIFTOFF`, `APOGEE`, `PINC` | that detector has latched | — |
| `APOFIRED` … `R4FIRED` | that channel triggered ≥ 1.5 s ago | — |

`FUTANG` is the one worth reading twice. It is not the current tilt — it
is where the nose is projected to be pointing `delay_ms` from now,
extrapolated from the current tilt rate. The horizon is the channel's own
delay because that is when the charge would actually go off. It is
written as a *below* test, since that is the direction you would AND into
a fire condition ("still close enough to vertical"); flip it in
`event_is_true()` if your config means the opposite.

`APOFIRED` and friends refer to **this board's** channels reaching their
conditions, not the Blue Raven's real charges. The board does see the
Blue Raven fire — that arrives on J7/J8 and lands in `event.bin` as
`EV_STAGING_SIGNAL` / `EV_RECOVERY_SIGNAL` — but wiring those in would
make this timeline a mirror of the Blue Raven's instead of an independent
check on it.

### How tilt is computed

A direction fixed in the world does not stay fixed in the vehicle's own
frame once the vehicle rotates. Track "up, as it was on the pad" in body
coordinates and the angle between it and the airframe is the tilt — no
magnetometer, no GPS.

A vector `u` constant in the inertial frame evolves in body coordinates
as `du/dt = -(ω × u)`. Each tick the code rotates `u` by the small angle
the vehicle just turned through and renormalises.

**The reference axis is measured, not assumed.** The averaged
accelerometer vector on a stationary pad *is* the airframe's up-axis in
body coordinates. So the code never needs to know whether the board's
+Z, −X or something diagonal points at the sky, and a board mounted
sideways in the next revision needs no change. The host test mounts the
simulated board on its side specifically to prove this.

**Gyro bias is estimated only from a live gyro.** In STANDBY the LSM6DSOX's
gyro is powered down and the driver reports exact zeros — which sail through
the at-rest gate and would have the estimator converge on them. So
`update_gyro_liveness()` keys on `flight_mode_is_fast()` and wipes the bias,
the sample count and `tilt_valid` whenever the gyro comes up: nothing learned
from a switched-off sensor survives. From there the estimator runs until
liftoff, feeding only on samples that look at-rest so a hand-carry to the pad
is not averaged in, and freezes at liftoff, after which the vehicle is never
still again.

**Liftoff will not trust an unconverged bias.** The freeze is accompanied by a
sanity check: below 50 live samples, tilt is marked invalid for the flight, the
refusal is logged at ERROR and recorded in `err.bin`, and the latch can never
re-arm because the sample count freezes too. Refusing to report an attitude is
the correct output when you do not have one. **Arm at least two seconds before
launch** if you want FANG or FUTANG to work — a launch straight out of STANDBY
on the IMU wake-up gets altitude, velocity and burns, but no tilt.

**A tilt event with no attitude estimate reads false, not true.** These
sit in an AND-set, so false blocks the channel. A tilt check that
silently passes because the gyro never came up would turn a safety
condition into a no-op exactly when it is the only thing that would have
caught the problem.

### How burns are counted

A burn is sustained acceleration **along the airframe axis**, not raw
vector magnitude — magnitude cannot tell thrust from a hard lateral jolt,
and it reads a healthy 1 g while the vehicle sits on the pad.

Entering a burn takes more acceleration (2.5 g) than leaving one (1.2 g),
so chatter around a single threshold cannot count one motor twice — the
same hysteresis a Schmitt trigger uses, for the same reason.

**The count increments at ignition, not burnout**, which is what makes
`BURNCOUNT >= 2` mean "the sustainer has lit". Burnout is recorded
separately as `EV_BURN_END`, so a burn's duration is recoverable from the
log. This is the vehicle's own opinion from the accelerometer alone; it
deliberately does not consult the Blue Raven's staging line, because
keeping the two independent is what lets a review notice they disagreed.

## Flight mode and standby

The vehicle powers on in **STANDBY** and stays there. On the pad that is
most of the day: sensors sampling slowly, the card filling slowly, the
radio mostly quiet. Full flight rates start when something says the
vehicle is about to fly.

```
   power on
       │
       ▼
   STANDBY ──── ARM (uplink) ────► ARMED
       │                             │
       │                       liftoff seen
       │                             ▼
       └──── IMU wake-up ────────► FLIGHT  ◄── DISARM is REFUSED here
                                     │
                              landing detected
                                     ▼
   STANDBY ◄──── DISARM ─────────  LANDED
```

`flight_mode_is_fast()` is the single predicate every driver keys on —
true for ARMED, FLIGHT and LANDED. Adding a state cannot silently leave a
sensor slow, because there is nowhere else the decision is made.

### What changes between the two rates

| | Standby | Flight |
|---|---|---|
| LSM6DSOX accel | 12.5 Hz | 104 Hz |
| LSM6DSOX gyro | powered down | 104 Hz |
| BMP388 | 1.5 Hz, OSR ×2 | 50 Hz, OSR ×8 |
| Battery | 1 Hz | 10 Hz |
| Telemetry block timeout | 5 s | 500 ms |
| Telemetry IMU decimation | 1 in 32 | 1 in 4 |

The gyro is *powered down* in standby rather than slowed. A powered-down
gyro's output registers hold whatever the last conversion left there, so
the driver writes explicit zeros instead of reading stale numbers that
look like a slow drift.

### Two ways in, and why there are two

**Uplink** — an ARM command from `blue_raven_homebrew` on the laptop,
over the XBee. This is the intended path: it is deliberate, it is
timestamped in `event.bin`, and it can be undone.

**IMU INT2** — the LSM6DSOX's own wake-up block, described under
*Hardware liftoff detect* below. This is the fallback for the radio being
down, out of range, or simply not switched on. It does not go through
ARMED: if the accelerometer says the vehicle is accelerating, the vehicle
is flying whether or not anybody armed it, so it goes straight to FLIGHT
and disarm locks immediately.

The two paths share nothing. The uplink path needs the radio, the UART,
the parser and the ground station. The IMU path needs the chip to have
power and a wire to GPIO15. That is the point.

`flight_mode_note_liftoff()` is called from `lsm6dsox_task`, not from the
ISR, because it ends in a ring-buffer push — the same reason
`discrete_in` captures its timestamp in the ISR and records it from the
task.

### Landing does not slow anything down

LANDED unlocks the disarm command and announces itself in `event.bin`.
It does not touch a single sample rate. Dropping to standby automatically
would start thinning the record at the exact moment somebody is still
deciding whether the vehicle landed or just went quiet.

Nothing calls `flight_mode_note_landed()` yet — `flight_logic`'s landing
detection is still on the *Known incomplete* list. Until it does, the
vehicle stays in FLIGHT after touchdown, which is the safe direction to
be wrong in: it keeps recording.

## Status stream

`status_task` publishes one 32-byte `status_record_t` per second to
`status.bin` and the downlink — mode, mode source, battery volts, free
heap, SD queue drops, telemetry counters, and a `STATUS_FLAG_*` bitfield.

It is the answer to "is the vehicle alive and what does it think it is
doing", in one record small enough to send at 1 Hz for an entire pad
session. A `PING` uplink makes it emit one immediately rather than
waiting for the tick, which is how you prove the link works in both
directions in about two seconds.

## Uplink commands

Ten bytes, little-endian, carried as the payload of an XBee 0x10 Transmit
Request and arriving here inside a 0x90 Receive Packet frame.

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | magic `0x5A` |
| 1 | 1 | high nibble version (1), low nibble opcode |
| 2 | 2 | `seq`, uint16, **strictly increasing** |
| 4 | 4 | `arg`, uint32, command-specific |
| 8 | 2 | CRC16-CCITT over bytes 0..7 |

| Opcode | Name | Effect |
|---|---|---|
| 0x0 | NOP | accepted, no effect — proves the link end to end |
| 0x1 | ARM | STANDBY → ARMED |
| 0x2 | DISARM | → STANDBY; **refused in FLIGHT** |
| 0x3 | PING | emit a status record now |
| 0x4 | MARK | write `arg` into `event.bin` as an operator marker |

Three things about this that are easy to get wrong:

**The magic is `0x5A`, the downlink's `0xA5` reversed.** A downlink packet
looped back by a repeater, a test harness or the ground station's own echo
can therefore never parse as a command.

**`seq` must increase, and it is persisted on the laptop.** The XBee
retransmits on its own, so the same command legitimately arrives twice;
without the sequence check one ARM press would be logged as two. A ground
station that restarts its counter at 1 will be ignored until the vehicle
reboots, which is why `tools/uplink_command.py` keeps the counter in a
file. "Newer" means the unsigned distance is in the lower half of the
16-bit space, so the counter wraps cleanly — at the cost of a window: a
jump of more than 32767 in one step reads as a jump backwards and is
discarded.

**The CRC is not security.** It rejects corruption, not an attacker. The
only command that can lose data is DISARM, and `flight_mode` refuses that
outright while the vehicle is flying. The state machine, not the packet
format, is what protects the flight record.

Commands are not acknowledged. Confirmation comes from the status stream,
and every rejection increments its own counter (`cmds_bad_crc`,
`cmds_stale`, `cmds_malformed`, `cmds_refused`) so a command that did
nothing is distinguishable afterwards from one that never arrived.

## Hardware liftoff detect

The LSM6DSOX's own wake-up block runs INT2 (GPIO15) as a liftoff
detector: gravity is filtered out, what remains is compared against a
3 g threshold, and the event must persist 2 consecutive samples
(~19 ms at 104 Hz) before it latches.

It is deliberately *not* tuned to agree with `flight_logic.c`'s software
detector (20 m/s², ~2 g). Two independent detectors that are tuned to
match are one detector with extra steps; the value is in them being able
to disagree. The software path needs SPI, the router, and the scheduler
all healthy. The hardware path needs the chip to have power.

The event latches (`LIR=1`) and nothing clears it, so INT2 stays high
for the rest of the flight — probe-able on the bench, and no re-entry
from vibration. Because it is a latch and not an edge,
`lsm6dsox_task` can call `flight_mode_note_liftoff()` every tick without
consequence; the state machine is idempotent by design for exactly this.

## Known incomplete

- `discrete_in` — records the event; no decision logic hangs off it yet.
- **Tilt is dead-reckoned and nothing corrects it.** Gyro bias is
  estimated before liftoff and frozen, which removes the constant term;
  what remains is random walk with no second reference during flight.
  Fine for the first minute, which is what `FANG`/`FUTANG` are asked
  about. It is not an attitude filter and should not be trusted as one
  on a long coast.
- **No coning compensation.** Sample-rate aliasing is gone now that every gyro
  sample is read, but a point rate still cannot represent rotation *within* one
  10 ms interval. Accumulating Δθ in the IMU driver between reads is the next
  step if a future airframe spins hard.
- **A negative `delay_ms` in the config silently disables a channel forever.**
  `cJSON`'s `valueint` is taken with no range check and cast to `uint32_t` at
  the comparison, so `-1` becomes 4294967295. Clamp at parse time.
- **The IMU is now the single point of failure for all eighteen events.** If it
  stops publishing, the task times out at 200 ms and the component goes silent.
  A barometer failure now degrades only the altitude and velocity events.
- **Burn detection thresholds are unflown.** 2.5 g to enter a burn and
  1.2 g to leave it are reasonable for a solid motor and have been
  checked only against the simulated flight in `test/host/`. Look at
  `imu.bin` after the first flight and set them from real thrust data.
- **Nothing calls `flight_mode_note_landed()`.** `flight_logic` has no
  landing detection yet, so LANDED is unreachable in flight and the
  vehicle stays in FLIGHT after touchdown — disarm stays locked and the
  card keeps filling. Safe direction to be wrong in, but it means the
  LANDED branch of the state machine has only ever been exercised by the
  host test.
- `sd_logger` `diag_lock` is declared but never taken. With three sensor
  tasks now calling `sd_logger_write()` concurrently, `records_queued`
  and `queue_full_drops` can lose increments. Diagnostics only — never
  affects logged sensor data.

## Design decisions worth knowing

**Overwrite-oldest, not reject-new.** A full queue discards its oldest
unwritten record. Keeps buffered data bounded-stale rather than
arbitrarily stale.

**512-byte write buffers.** Keeps each write within one SD sector. Does
*not* guarantee aligned flushes — flush sizes (494/494/504 typical)
rarely divide evenly into 512. Padding to force alignment was
considered and rejected: GPS would become ~93% zeros for a benefit
smaller than what the queues already absorb.

**Sensor drivers now depend on `data_router`.** They used to be
SD-agnostic, with separate bridge tasks doing the push. Moving the push
into the driver tasks removed a redundant lock-and-copy round trip, at
the cost of no longer being able to compile a sensor driver standalone.
Deliberate trade-off.

## tools/

`decode_logs.py` — converts `.bin` logs to CSV. Verifies the magic
bytes, checks `record_size` against its own expectations (catches
firmware/decoder drift), and CRC-checks every record.

`decode_telemetry.py` — decodes downlink packets, from a capture file or
live from the ground radio with `--port`. See *Seeing it on the ground*
above.

`uplink_command.py` — the ground station half of the command protocol.
Import it from `blue_raven_homebrew`, or run it directly:

```
python tools/uplink_command.py --port COM5 arm
python tools/uplink_command.py --port COM5 mark --arg 3
python tools/uplink_command.py --selftest        # no hardware needed
```

It persists the sequence counter to `tools/.uplink_seq`, so restarting
the ground station does not get you ignored.

## test/host/

```
./test/host/run_host_tests.sh
```

No ESP-IDF, no hardware. The ESP-IDF surface the code touches is stubbed
with *runnable* stubs, so the real XBee frame parser, the real CRC, the
real sequence logic, the real event log and the real `flight_mode` state
machine all execute. Only the UART and the scheduler are fake.

Two of the five stages are the ones worth having.

The **cross-check** generates command packets with the Python
ground-station tool and replays them through the C firmware path,
asserting the vehicle's own verdict for each — that is what proves the
two implementations agree rather than each being self-consistent.

The **flight_logic** stage flies a simulated two-stage flight through the
real estimator across eleven phases — 63 checks — and prints a truth
table for all 18 events at boost, coast and descent, so the polarity of
every threshold can be checked by eye before it is ever loaded onto a
rocket.

Four properties of that simulator are deliberate, and each targets a way
this code fails while still producing plausible numbers:

- the board is mounted **on its side** (airframe axis on body +X), so an
  axis that is assumed rather than measured makes every tilt number wrong;
- the gyro carries a constant bias in **every live sample**, including
  after liftoff, so a bias subtraction that quietly stops working looks
  like drift;
- it reports **exact zeros** while the mode is STANDBY, as the driver does
  when the sensor is powered down — this is what makes the bias regression
  reachable at all;
- it runs **two independent clocks**, so collapsing the estimator's two
  timelines back into one fails the velocity check.

The three phases that matter: 1–2 (30 s of standby with the gyro off, then
arm) assert tilt is *not* valid until the gyro has been live; 4 asserts the
velocity estimate is steady rather than alternating `0 / 2×`; 11 launches
straight out of STANDBY with no arm and asserts tilt is **refused** rather
than reported, while altitude and burn counting keep working. Reverting the
liveness gate fails three checks, so these have teeth rather than merely
describing the intended behaviour.

The **sd_logger** stage stubs the SDMMC layer into no-ops, points the mount
point at a real temp directory, and lets the real code open real files with
real `fopen`/`fwrite` — then reads the bytes back off disk and compares them
against what was pushed. Nothing about the record path is simulated.

It exists because each stream's ring buffer is sized to `sizeof(that stream's
record)`, which makes element size a per-buffer contract nothing re-checks at
runtime. If an element size and a record size ever disagree, push copies the
wrong number of bytes and the failure is *silent*: every record in that stream
lands on the card subtly wrong and you find out after the flight. Two negative
controls confirm the test bites — shrinking one buffer's element by a single
byte, and misfiling one stream into another's file, each fail two checks.
