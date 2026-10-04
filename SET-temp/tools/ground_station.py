#!/usr/bin/env python3
"""
ground_station.py -- the laptop end of the Osiris downlink and uplink.

    python tools/ground_station.py                                  # GUI, pick the port in the window
    python tools/ground_station.py --port COM5                      # GUI, real ground XBee
    python tools/ground_station.py --port socket://127.0.0.1:9750   # GUI, via xbee_link_emulator.py
    python tools/ground_station.py --port COM5 --headless           # terminal only
    python tools/ground_station.py --replay sessions/<dir>/raw.bin --headless

It always speaks real XBee API mode 1 to whatever the port string opens.
A COM port is a real ground radio; socket://... is the link emulator. Nothing
else differs, so moving from the emulator to a second radio changes one string.

Single source of truth
----------------------
Record layouts come from decode_logs.py (STREAMS) and the uplink from
uplink_command.py (build_command, SeqCounter). Nothing here restates either,
so the SD decoder, the command tool and this program cannot drift apart. The
sequence counter file (tools/.uplink_seq) is shared with uplink_command.py.

Downlink packet (telemetry.h)
-----------------------------
    0 magic 0xA5 | 1 ver<<4|flags | 2 seq u16 | 4 rssi | 5 count | 6 len u16
    8 records: [u8 stream][record] x count, sizes implied by stream
    8+len crc u16, CRC16-CCITT over bytes 0 .. 8+len-1

The records carry no lengths, so a record whose size differs between the
firmware and decode_logs.py would decode as plausible garbage. The check that
catches it: the implied sizes must add up to `len` exactly and consume exactly
`count` records. A packet that fails that is rejected as a size mismatch,
never half-decoded.

Every session writes to sessions/<timestamp>/: one CSV per stream (the
decode_logs.py columns plus rx_time) and raw.bin, every byte the radio
produced, which --replay reads back.
"""

import argparse
import collections
import csv
import json
import math
import subprocess
import tempfile
import os
import queue
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode_logs as dl                                    # noqa: E402
import uplink_command as up                                 # noqa: E402
from xbee_api import BROADCAST, FrameParser, build_frame    # noqa: E402

PKT_MAGIC = 0xA5
PKT_VERSION = 1
PKT_HDR = "<BBHBBH"
PKT_HDR_SIZE = 8
FLAG_RETRANSMIT = 0x01

RECORD_SIZES = {sid: struct.calcsize(fmt) for sid, (_, fmt, _) in dl.STREAMS.items()}
STREAM_NAMES = {sid: name for sid, (name, _, _) in dl.STREAMS.items()}
OPCODE_NAMES = {v: k.upper() for k, v in up.COMMANDS.items()}

# STATUS_FLAG_* bit positions, from sensor_records.h. Kept in step by hand,
# like decode_logs.py's enum tables. An unnamed bit is always shown as hex,
# never hidden, so a firmware that adds a bit shows up as "0x..." here
# rather than silently disappearing.
STATUS_FLAG_SD_OK = 0x01

# Must match CONFIG_FC_SD_AUTO_EJECT_S in main/Kconfig.projbuild. The
# vehicle does not downlink its own timeout, so this is a second copy of
# one number and the usual hand-synced-table discipline applies: change
# one, change the other. It only drives a countdown display -- getting it
# wrong shows a wrong estimate, it does not change what the vehicle does.
# 0 here means "the timer is disabled", and no countdown is shown.
SD_AUTO_EJECT_S = 6 * 60 * 60
STATUS_FLAG_NAMES = {
    STATUS_FLAG_SD_OK: "SD_OK",
    0x02: "TELEM_OK",
    0x04: "GPS_FIX",
    0x08: "LIFTOFF_HW",
    0x10: "LIFTOFF_SW",
    0x20: "APOGEE",
    0x40: "LANDED",
    0x80: "DISARM_OK",
}

# Must match CFG_REJECT_* in sensor_records.h. Each reason wants a
# different response, which is why the firmware does not collapse them
# into one "it failed" code.
CFG_REJECT_NAMES = {
    1: "chunks were lost",
    2: "the bytes arrived corrupted",
    3: "the config is too large",
    4: "the vehicle could not write NVS",
}
CFG_REJECT_DETAIL = {
    1: "first missing chunk",
    2: "crc computed",
    3: "size",
    4: "esp_err",
}

# ======================================================================
#  Deployment config schema
#
#  EVENT_KEYS and CHANNEL_KEYS must match EVENT_KEYS[] and
#  CHANNEL_NAMES[] in components/flight_logic/flight_logic.c EXACTLY,
#  including order and spelling -- the vehicle looks these up as JSON
#  object keys, and a key it does not recognise is silently ignored
#  rather than rejected, which is the worst possible failure for a
#  deployment condition. Same hand-synced-table discipline as
#  decode_logs.py's enum tables.
#
#  The units and comparison directions come from the block comment in
#  flight_logic.h. The "1"/"2" suffix is a DIRECTION, not a different
#  quantity: 1 is below the threshold, 2 is above it.
# ======================================================================
EVENT_KEYS = ["AGL1", "AGL2", "FANG1", "FANG2", "FUTANG",
              "TVAL1", "TVAL2", "VEL1", "VEL2", "BVEL1", "BURNCOUNT",
              "LIFTOFF", "APOGEE", "PINC",
              "APOFIRED", "MAINFIRED", "R3FIRED", "R4FIRED"]

CHANNEL_KEYS = ["Apo", "Main", "3rd", "4th"]

# key -> (what it tests, unit). A unit of None means the event takes no
# threshold at all, and the panel greys that field out rather than
# offering a number the vehicle will never read.
EVENT_MEANING = {
    "AGL1":      ("altitude above pad  <", "ft"),
    "AGL2":      ("altitude above pad  >", "ft"),
    "FANG1":     ("tilt from vertical  <", "deg"),
    "FANG2":     ("tilt from vertical  >", "deg"),
    "FUTANG":    ("PROJECTED tilt at fire time  <", "deg"),
    "TVAL1":     ("time since liftoff  <", "s"),
    "TVAL2":     ("time since liftoff  >", "s"),
    "VEL1":      ("|vertical speed|  <", "ft/s"),
    "VEL2":      ("|vertical speed|  >", "ft/s"),
    "BVEL1":     ("signed vertical speed  <", "ft/s"),
    "BURNCOUNT": ("motor burns started  >=", "count"),
    "LIFTOFF":   ("liftoff detected", None),
    "APOGEE":    ("apogee detected", None),
    "PINC":      ("pressure increasing (descending)", None),
    "APOFIRED":  ("this board's Apo channel fired >=1.5 s ago", None),
    "MAINFIRED": ("this board's Main channel fired >=1.5 s ago", None),
    "R3FIRED":   ("this board's 3rd channel fired >=1.5 s ago", None),
    "R4FIRED":   ("this board's 4th channel fired >=1.5 s ago", None),
}

# ======================================================================
#  Plot palette
#
#  Categorical slots 1-3, validated as a set for all-pairs separation on a
#  light surface: worst CVD delta-E 9.2, worst normal-vision 24.0. Used in
#  this FIXED ORDER for X/Y/Z on every plot -- X is always blue, Y always
#  orange, Z always aqua, on accel and gyro alike. Colour follows the axis,
#  never the position in a list, so a plot with one trace hidden does not
#  repaint the others.
#
#  Aqua sits at 2.7:1 against white, under the 3:1 bar, so every series
#  carries a direct label at the end of its trace as well as a legend
#  entry. Identity is never colour alone.
# ======================================================================
SERIES_COLORS = ("#2a78d6", "#eb6834", "#1baf7a")      # X, Y, Z
AXIS_NAMES = ("X", "Y", "Z")

INK_PRIMARY   = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED     = "#8a8984"
GRID          = "#e8e8e6"
PLOT_SURFACE  = "#fcfcfb"

G0 = 9.80665          # m/s^2 per g; the IMU logs m/s^2, rocketry reads g

# ======================================================================
#  Plot event markers
#
#  Four GROUPS, separated by dash pattern rather than by colour. The
#  traces already own the categorical palette, and a marker that
#  competed with them for hue would make the plot harder to read, not
#  easier. Dash also survives greyscale printing and every flavour of
#  colour blindness, which hue does not.
#
#  The whole point of BR and FC being separate groups: this board's
#  timeline is a SECOND OPINION on the Blue Raven's, and the gap between
#  a "BR" line and the matching "FC" line is the thing you came to look
#  at.
# ======================================================================
MARKER_GROUPS = {
    "mode": {"dash": None,          "color": "#6b6a66"},   # vehicle state
    "br":   {"dash": (5, 3),        "color": "#8a8984"},   # Blue Raven said so
    "fc":   {"dash": (2, 3),        "color": "#8a8984"},   # this board said so
    "est":  {"dash": (6, 2, 2, 2),  "color": "#a4a39e"},   # estimator call
    # Operator MARKs. Darker than the rest because it is the one group
    # somebody on the ground put there on purpose, and tight dots so it
    # cannot be mistaken for the BR/FC dashes it will often sit beside.
    "op":   {"dash": (1, 3),        "color": "#3f3e3a"},   # operator MARK
}

# Kept terse on purpose. A rotated label is as tall as it is long, and
# these arrive in clusters -- liftoff HW and SW are ~70 ms apart, and the
# FC and BR calls for one deploy are a few hundred ms apart. Long labels
# turn exactly the interesting moments into an unreadable thicket.
MODE_SOURCE_SHORT = {0: "", 1: "up", 2: "IMU", 3: "lgc", 4: "boot", 5: "test"}
MODE_SHORT = {"STANDBY": "SBY", "ARMED": "ARM", "FLIGHT": "FLT", "LANDED": "LND"}


def classify_event(rec):
    """(label, group) for an event worth a line on the plots, else None.

    Everything else the vehicle logs still reaches event.bin and the
    message log -- this decides only what earns a vertical rule across
    somebody's altitude trace."""
    typ, ch, detail = rec["type"], rec["channel"], rec["detail"]

    if typ == "MODE_CHANGED":
        name = dl.MODES[ch] if 0 <= ch < len(dl.MODES) else f"mode{ch}"
        if name == "STANDBY" and detail != 5:
            return None            # returning to standby is not a flight event
                                   # -- except in a TEST build, where the
                                   # scripted cycle is the thing being watched
        short = MODE_SHORT.get(name, name)
        src = MODE_SOURCE_SHORT.get(detail, "")
        # The source is kept because it is the interesting half: FLIGHT
        # reached by the IMU wake-up block rather than by an uplink ARM
        # means the radio path was not what started the flight.
        return (f"{short}/{src}" if src else short, "mode")
    if typ == "STAGING_SIGNAL":
        return ("BR stg", "br")
    if typ == "RECOVERY_SIGNAL":
        return ("BR rec", "br")
    if typ == "DEPLOY_PREDICTED":
        name = CHANNEL_KEYS[ch] if 0 <= ch < len(CHANNEL_KEYS) else f"ch{ch}"
        return (f"FC {name}", "fc")
    if typ == "APOGEE":
        return ("apogee", "est")
    if typ == "LIFTOFF_HW":
        return ("LO hw", "est")
    if typ == "LIFTOFF_SW":
        return ("LO sw", "est")
    # MARK is logged as UPLINK_COMMAND with channel = the MARK opcode and
    # detail = the number the operator typed. Other accepted commands use
    # the same event type with detail = seq, and stay off the plots.
    if typ == "UPLINK_COMMAND" and ch == up.MARK:
        return (f"M{detail}", "op")
    # LANDED duplicates the MODE_CHANGED into LANDED; one line, not two.
    return None


DELIVERY_STATUS = {0x00: "delivered", 0x01: "MAC ACK failure", 0x02: "LBT failure",
                   0x21: "network ACK failure", 0x25: "route not found",
                   0x74: "payload too large"}


# ==========================================================================
# Packet decoding (pure functions, no state)
# ==========================================================================

class PacketError(Exception):
    def __init__(self, kind, msg):
        super().__init__(msg)
        self.kind = kind


def decode_record(stream_id, raw):
    name, fmt, columns = dl.STREAMS[stream_id]
    row = list(struct.unpack(fmt, raw))
    for col, table in dl.ENUM_COLUMNS.get(stream_id, {}).items():
        i = columns.index(col)
        row[i] = dl._name(table, row[i])
    return dict(zip(columns, row))


def decode_packet(p: bytes):
    if len(p) < PKT_HDR_SIZE + 2:
        raise PacketError("malformed", f"{len(p)} bytes is shorter than a header")
    magic, verflags, seq, rssi, count, length = struct.unpack_from(PKT_HDR, p)
    if magic != PKT_MAGIC:
        raise PacketError("malformed", f"magic 0x{magic:02X}, expected 0x{PKT_MAGIC:02X}")
    if verflags >> 4 != PKT_VERSION:
        raise PacketError("malformed", f"packet version {verflags >> 4}, this decoder "
                          f"knows {PKT_VERSION} -- firmware is newer than the ground station")
    if PKT_HDR_SIZE + length + 2 != len(p):
        raise PacketError("malformed", f"header says {length} payload bytes, "
                          f"packet has {len(p) - PKT_HDR_SIZE - 2}")
    body = PKT_HDR_SIZE + length
    if dl.crc16_ccitt(p[:body]) != struct.unpack_from("<H", p, body)[0]:
        raise PacketError("crc", f"CRC failed (seq field reads {seq})")

    records, i = [], PKT_HDR_SIZE
    while i < body and len(records) < count:
        sid = p[i]
        size = RECORD_SIZES.get(sid)
        if size is None:
            raise PacketError("size", f"seq {seq}: unknown stream tag {sid} "
                              f"-- decode_logs.py is behind the firmware")
        if i + 1 + size > body:
            raise PacketError("size", f"seq {seq}: {STREAM_NAMES[sid]} record overruns "
                              f"the packet -- record size differs from the firmware")
        records.append((sid, decode_record(sid, p[i + 1:i + 1 + size])))
        i += 1 + size
    if i != body or len(records) != count:
        raise PacketError("size", f"seq {seq}: records do not tile the payload "
                          f"({len(records)}/{count} records, {i - PKT_HDR_SIZE}/{length} "
                          f"bytes) -- a record size differs from the firmware")
    return {"seq": seq, "flags": verflags & 0x0F,
            "retransmit": bool(verflags & FLAG_RETRANSMIT),
            "rssi": rssi, "records": records}


# ==========================================================================
# Packet sequence accounting
# ==========================================================================

class SeqTracker:
    """Counts lost and duplicate packets from the 16-bit packet seq.

    A retransmit carries the SAME seq as the original (telemetry.c sets the
    retransmit flag and re-CRCs, but keeps the seq), so a retransmit that
    arrives after its original is a duplicate, and one that arrives after
    the original was lost fills that gap and is subtracted back out of lost.
    """

    RESET_BACKJUMP = 256

    def __init__(self):
        self.last = None
        self.lost = 0
        self.duplicates = 0
        self.resets = 0
        self.missing = collections.OrderedDict()
        self.recent = collections.OrderedDict()

    def _remember(self, seq):
        self.recent[seq] = None
        if len(self.recent) > 1024:
            self.recent.popitem(last=False)

    def accept(self, seq):
        """Returns 'new', 'late', 'dup' or 'reset'."""
        if seq in self.recent:
            self.duplicates += 1
            return "dup"
        if self.last is None:
            self.last = seq
            self._remember(seq)
            return "new"
        fwd = (seq - self.last) & 0xFFFF
        if 0 < fwd < 0x8000:
            gap = fwd - 1
            self.lost += gap
            for k in range(min(gap, 1024)):
                self.missing[(self.last + 1 + k) & 0xFFFF] = None
            while len(self.missing) > 1024:
                self.missing.popitem(last=False)
            self.last = seq
            self._remember(seq)
            return "new"
        if seq in self.missing:
            del self.missing[seq]
            self.lost -= 1
            self._remember(seq)
            return "late"
        if ((self.last - seq) & 0xFFFF) > self.RESET_BACKJUMP:
            # Far behind and never missed: the vehicle's counter restarted.
            self.resets += 1
            self.last = seq
            self.missing.clear()
            self.recent.clear()
            self._remember(seq)
            return "reset"
        self._remember(seq)
        return "late"


# ==========================================================================
# The station: everything except the window
# ==========================================================================

class SimPlayback:
    """A CSV flight being replayed onto the plots at the rate it was flown.

    The rows carry their own timestamps, so the replay is driven by
    those rather than by how often the GUI happens to tick: each tick
    advances a simulated clock by the wall time that has passed (times
    a speed multiplier) and releases every row that clock has reached.
    A slow tick therefore releases several rows at once instead of
    falling behind, and a tick that never came -- a dragged window, a
    machine that slept -- is clamped rather than fast-forwarding the
    whole flight into a single frame.

    It owns its own buffers. Mixing a simulation into the live history
    would produce a plot that is neither, and clearing the live history
    to make room would throw away telemetry that is still arriving."""

    MAX_STEP_S = 1.0        # the longest wall-clock gap a tick will honour

    KIND_ALT, KIND_IMU, KIND_EVENT = 0, 1, 2

    def __init__(self, name, alt_rows, imu_rows, sim_events, speed=1.0,
                 summary=None):
        self.name = name
        self.speed = float(speed) if speed else 1.0
        self.paused = False
        self.summary = list(summary or [])      # emitted when playback ends
        self.summary_done = False
        self.live_packets = 0                   # real packets that arrived mid-replay
        self.live_records = 0

        # AGL, not MSL: the CSV is an altitude above sea level and the
        # plots are above the pad, so the lowest sample becomes zero --
        # the same thing flight_logic does with its pad reference.
        pad = min((a for _t, a in alt_rows), default=0.0)
        sched = [(t, self.KIND_ALT, (t, a - pad)) for t, a in alt_rows]
        sched += [(row[0], self.KIND_IMU, tuple(row)) for row in imu_rows]
        for t, ev_name, ch, detail in sim_events:
            marked = classify_event({"type": ev_name, "channel": ch,
                                     "detail": detail})
            if marked:
                sched.append((t, self.KIND_EVENT, (t, marked[0], marked[1])))
        # By time, then by kind, so a deploy marker is released after the
        # sample that caused it rather than a frame ahead of it.
        sched.sort(key=lambda r: (r[0], r[1]))
        self.sched = sched
        self.i = 0
        self.t0 = sched[0][0] if sched else 0.0
        self.duration = (sched[-1][0] - self.t0) if sched else 0.0
        self.play_t = 0.0                       # seconds into the flight, as shown
        self.last_wall = None

        self.agl = collections.deque(maxlen=6000)
        self.imu = collections.deque(maxlen=20000)
        # A flight-data CSV carries altitude and IMU, never a position,
        # so the ground track has nothing to show during a replay. An
        # empty deque says that honestly; borrowing the live track would
        # draw the vehicle's real position next to a simulated altitude.
        self.track = collections.deque(maxlen=1)
        self.events = collections.deque(maxlen=400)

    @property
    def done(self):
        return self.i >= len(self.sched)

    def tick(self, now):
        """Advance to wall-clock `now`; return how many rows were released."""
        if self.last_wall is None:
            self.last_wall = now
        dt = now - self.last_wall
        self.last_wall = now
        if self.paused or self.done or dt <= 0.0:
            return 0
        self.play_t += min(dt, self.MAX_STEP_S) * self.speed
        cutoff = self.t0 + self.play_t
        released = 0
        while self.i < len(self.sched) and self.sched[self.i][0] <= cutoff:
            _t, kind, payload = self.sched[self.i]
            if kind == self.KIND_ALT:
                self.agl.append(payload)
            elif kind == self.KIND_IMU:
                self.imu.append(payload)
            else:
                self.events.append(payload)
            self.i += 1
            released += 1
        return released

    def progress(self):
        """(seconds shown, total seconds, fraction 0..1)."""
        shown = min(self.play_t, self.duration)
        frac = 1.0 if self.duration <= 0 else max(0.0, min(1.0, shown / self.duration))
        return (shown, self.duration, frac)


class GroundStation:
    """Owns the link, the decoded vehicle state and the session files.

    Threading: a reader thread only pushes raw bytes into a queue. All
    parsing and state changes happen in poll(), called from whichever loop
    owns the station (the GUI's timer or the headless loop), so nothing
    needs a lock."""

    def __init__(self, out_dir=None, dest=BROADCAST, capture=True, seq_path=None):
        self.dest = dest
        self.ser = None
        self.port_name = None
        self.inbox = queue.Queue()
        self.reader = None
        self.parser = FrameParser()
        self.seq = SeqTracker()
        self.seq_counter = up.SeqCounter(seq_path)
        self.messages = collections.deque()
        self.stats = collections.Counter()
        self.frame_id = 0
        # Config uploads go out through a PACED queue rather than a burst
        # of writes. Twenty-odd frames handed to pyserial back to back
        # arrive at the ground radio far faster than it can put them on
        # the air, and nothing here honours CTS -- the same flow-control
        # gap that is still open on the vehicle side. One frame per poll
        # (50 ms in the GUI) empties a whole config in about a second and
        # cannot overrun anything.
        self.tx_queue = collections.deque()
        self.upload = None                            # dict while one is in flight
        self.config_status = None                     # last verdict from the vehicle
        self.pending_tx = {}                          # frame id -> command info
        self.awaiting_ack = collections.deque(maxlen=32)
        self.at_fids = {}

        self.latest = {}                              # stream name -> record dict
        self.mode = None
        self.flags = None
        self.sd_mounted = None                        # None until the first status
        self.vehicle_rssi = None                      # uplink, as heard by the vehicle
        self.ground_rssi = None                       # downlink, as heard here
        self.last_packet_rx = None
        self.connected_at = None
        self.last_db_poll = 0.0
        self.agl_offset = None                        # baro altitude_m - status AGL
        self.agl_history = collections.deque(maxlen=6000)   # (vehicle s, AGL m)
        # (vehicle s, ax, ay, az in g, gx, gy, gz in deg/s). IMU is
        # decimated for the downlink -- 1 in 4 in flight, 1 in 32 in
        # standby -- so this is ~26 Hz airborne, not the 104 Hz on the
        # card. The card is the record; this is the view.
        self.imu_history = collections.deque(maxlen=20000)
        self.gps_track = collections.deque(maxlen=6000)     # (vehicle s, east m, north m)
        # (vehicle s, short label, group) for the plot markers. Bounded:
        # a chattering discrete input can produce these faster than any
        # plot can show them, and an unbounded list would be the same
        # denial of service the vehicle-side filter exists to stop.
        self.events = collections.deque(maxlen=400)
        self.pad_fix = None                                 # (lat, lon) of the first valid fix
        # A replay in progress, or None. The four buffers above are the
        # LIVE ones and a replay never touches them: telemetry keeps
        # being decoded, written to this session's files and accumulated
        # underneath, and Finish puts it back on screen with nothing
        # missing. That only stays true because every view reads the
        # plot_*() accessors below rather than the buffers directly.
        self.sim = None

        self.out_dir = out_dir
        self.capture = capture
        self._csv = {}
        self._raw = None

    # ---------------------------------------------------------------- log
    def note(self, text, level="info", key=None):
        """Queue a line for the log.

        `key` is what decides whether two lines are "the same problem".
        It exists because the obvious test -- identical text -- is wrong
        here: a repeating vehicle error carries a rising suppressed
        count and a varying detail field, so no two lines of a flood are
        ever byte-identical, and matching on text would collapse
        nothing. Callers that can fail repeatedly pass a stable key
        naming the FAULT rather than the message. Anything without one
        falls back to its own text, which is right for the one-off
        messages that make up the rest of the log."""
        self.messages.append((time.time(), level, text, key if key is not None else text))

    def drain_messages(self):
        out = list(self.messages)
        self.messages.clear()
        return out

    # ---------------------------------------------------------- session files
    def _session_dir(self):
        if self.out_dir is None:
            self.out_dir = os.path.join("sessions", time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.out_dir, exist_ok=True)
        return self.out_dir

    def _write_csv(self, sid, rec, rx_time):
        name, _, columns = dl.STREAMS[sid]
        if sid not in self._csv:
            f = open(os.path.join(self._session_dir(), f"{name}.csv"), "w", newline="")
            w = csv.writer(f)
            w.writerow(columns + ["rx_time"])
            self._csv[sid] = (f, w)
        f, w = self._csv[sid]
        w.writerow([rec[c] for c in columns] + [f"{rx_time:.3f}"])

    def _capture_raw(self, chunk):
        if not self.capture:
            return
        if self._raw is None:
            self._raw = open(os.path.join(self._session_dir(), "raw.bin"), "ab")
        self._raw.write(chunk)

    def flush_files(self):
        for f, _ in self._csv.values():
            f.flush()
        if self._raw:
            self._raw.flush()

    def close_files(self):
        for f, _ in self._csv.values():
            f.close()
        self._csv.clear()
        if self._raw:
            self._raw.close()
            self._raw = None

    # ------------------------------------------------------------ connection
    def connect(self, port, baud=115200):
        import serial
        self.disconnect()
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0.05)
        self.port_name = port
        self.connected_at = time.time()
        self.parser = FrameParser()
        self.reader = threading.Thread(target=self._read_loop, args=(self.ser,), daemon=True)
        self.reader.start()
        self.note(f"connected to {port} @ {baud}")

    def disconnect(self):
        ser, self.ser = self.ser, None
        if ser:
            try:
                ser.close()
            except Exception:
                pass
            self.note(f"disconnected from {self.port_name}")
        self.flush_files()

    def _read_loop(self, ser):
        while ser is self.ser:
            try:
                chunk = ser.read(max(1, ser.in_waiting))
            except Exception as e:
                if ser is self.ser:
                    self.inbox.put(("error", str(e)))
                return
            if chunk:
                self.inbox.put(("data", chunk))

    @property
    def connected(self):
        return self.ser is not None

    def _next_fid(self):
        self.frame_id = self.frame_id % 255 + 1
        return self.frame_id

    def _write(self, frame):
        if not self.ser:
            raise RuntimeError("not connected")
        self.ser.write(frame)

    # ---------------------------------------------------------------- uplink
    def send_command(self, name, arg=0):
        opcode = up.COMMANDS[name]
        seq = self.seq_counter.next()
        payload = up.build_command(opcode, seq, arg)
        fid = self._next_fid()
        self._write(up.xbee_tx_request(payload, self.dest, fid))
        info = {"name": name.upper(), "opcode": opcode, "seq": seq, "arg": arg,
                "sent": time.time()}
        self.pending_tx[fid] = info
        self.awaiting_ack.append(info)
        self.stats["cmds_sent"] += 1
        extra = f" arg={arg}" if name == "mark" else ""
        self.note(f"sent {info['name']} seq={seq}{extra}", "cmd")
        return seq

    # ------------------------------------------------------------- config
    def send_config(self, text):
        """Queue a deployment config for upload. Returns (chunks, crc).

        Nothing is written to the port here; the packets are queued and
        drained one per poll(). The vehicle answers with CONFIG_STORED or
        CONFIG_REJECTED once the commit lands -- see _on_event."""
        packets, crc, chunks = up.config_packets(text, self.seq_counter.next)
        for i, payload in enumerate(packets):
            self.tx_queue.append(up.xbee_tx_request(payload, self.dest, self._next_fid()))
            del i
        self.upload = {"sent": 0, "total": len(packets), "chunks": chunks,
                       "crc": crc, "bytes": len(text.encode("utf-8")),
                       "started": time.time(), "verdict": None}
        self.config_status = None
        self.note(f"uploading config: {self.upload['bytes']} bytes, "
                  f"{chunks} chunks, crc 0x{crc:04X}", "cmd")
        return chunks, crc

    def cancel_upload(self):
        self.tx_queue.clear()
        if self.upload and self.upload["verdict"] is None:
            self.upload["verdict"] = "cancelled"
            self.note("config upload cancelled", "warn")

    def upload_progress(self):
        """(sent, total) while an upload is in flight, else None."""
        if not self.upload or self.upload["sent"] >= self.upload["total"]:
            return None
        return self.upload["sent"], self.upload["total"]

    def _pump_tx(self):
        if not self.tx_queue or not self.ser:
            return
        frame = self.tx_queue.popleft()
        try:
            self._write(frame)
        except Exception as e:
            self.tx_queue.clear()
            self.note(f"config upload failed to send: {e}", "error")
            if self.upload:
                self.upload["verdict"] = "send failed"
            return
        if self.upload:
            self.upload["sent"] += 1

    def allowed_commands(self):
        """Which buttons make sense for the last mode the vehicle reported.
        The vehicle enforces all of this itself; this only avoids offering
        a button that is certain to do nothing."""
        ok = {"ping", "mark"}
        if self.mode in (None, "STANDBY"):
            ok.add("arm")
        if self.mode in ("ARMED", "LANDED"):
            ok.add("disarm")
        # The vehicle accepts an eject only in STANDBY and LANDED. Offered
        # while the mode is still unknown for the same reason ARM is: the
        # button being live is not a claim that it will work, and a greyed
        # button before the first status record would hide a control the
        # operator may need on a link that is only sending one way.
        # Already-unmounted is a different matter -- SD_OK clear means the
        # card is gone until a power cycle, so there is nothing left to
        # eject and the button stops pretending otherwise.
        if self.mode in (None, "STANDBY", "LANDED") and self.sd_mounted is not False:
            ok.add("sd_eject")
        # Applying a stored config is gated exactly like the eject.
        if self.mode in (None, "STANDBY", "LANDED"):
            ok.add("config_apply")
        return ok if self.connected else set()

    def _poll_ground_rssi(self):
        fid = self._next_fid()
        self.at_fids[fid] = "DB"
        self._write(build_frame(bytes([0x08, fid]) + b"DB"))

    # ------------------------------------------------------------------ poll
    def poll(self):
        while True:
            try:
                kind, item = self.inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "error":
                self.note(f"port error: {item}", "error")
                self.disconnect()
                continue
            self.feed(item)
        self._pump_tx()
        # A replay advances on the GUI tick, not on packet arrival, so it
        # plays at the right rate on a silent link -- which is the usual
        # case when somebody is simulating.
        self.sim_tick()
        # ATDB on the ground radio measures the downlink. Only worth asking
        # once something has been heard.
        now = time.time()
        if self.connected and self.last_packet_rx and now - self.last_db_poll > 1.0:
            self.last_db_poll = now
            try:
                self._poll_ground_rssi()
            except Exception as e:
                # Polled once a second, so a broken port floods without this.
                self.note(f"ATDB poll failed: {e}", "error", key="atdb-poll")

    def feed(self, chunk, rx_time=None):
        rx_time = rx_time if rx_time is not None else time.time()
        self._capture_raw(chunk)
        for kind, data in self.parser.feed(chunk):
            if kind == "frame":
                self._on_frame(data, rx_time)
            elif kind == "junk":
                self.stats["junk_bytes"] += len(data)
            else:
                self.stats["bad_frames"] += 1

    def _on_frame(self, f, rx_time):
        t = f[0]
        if t == 0x90 and len(f) >= 12:
            self._on_packet(f[12:], rx_time)
        elif t == 0x8B and len(f) >= 7:
            self._on_tx_status(f[1], f[5])
        elif t == 0x88 and len(f) >= 5:
            cmd = f[2:4].decode("ascii", "replace")
            self.at_fids.pop(f[1], None)
            if cmd == "DB" and f[4] == 0 and len(f) >= 6:
                self.ground_rssi = f[5]
        elif t == 0x8A and len(f) >= 2:
            self.note(f"ground radio modem status 0x{f[1]:02X}")
        else:
            self.stats["other_frames"] += 1

    def _on_tx_status(self, fid, status):
        info = self.pending_tx.pop(fid, None)
        if info is None:
            return
        what = f"{info['name']} seq={info['seq']}"
        if status != 0:
            self.note(f"{what}: radio reports NOT delivered "
                      f"({DELIVERY_STATUS.get(status, f'0x{status:02X}')})", "error")
        elif self.dest == BROADCAST:
            self.note(f"{what}: transmitted (broadcast: no delivery confirmation "
                      f"-- watch for the vehicle's UPLINK_COMMAND event)")
        else:
            self.note(f"{what}: delivered to the vehicle radio")

    def _on_packet(self, payload, rx_time):
        try:
            pkt = decode_packet(payload)
        except PacketError as e:
            self.stats[f"pkt_{e.kind}"] += 1
            # A CRC failure is ordinary radio life; a malformed or size
            # error means firmware and decoder disagree, which is not.
            if e.kind != "crc" or self.stats["pkt_crc"] <= 5:
                # Keyed by kind: a firmware/decoder size disagreement
                # repeats on every single packet until someone fixes it.
                self.note(str(e), "error" if e.kind != "crc" else "warn",
                          key=f"pkt-{e.kind}")
            return

        verdict = self.seq.accept(pkt["seq"])
        if pkt["retransmit"]:
            self.stats["retransmits"] += 1
        if verdict == "dup":
            return
        if verdict == "reset":
            self.note(f"packet seq jumped back to {pkt['seq']} -- vehicle rebooted?", "warn")

        if self.sim is not None:
            # Live data has arrived while a replay is on screen. The
            # replay is NOT interrupted -- you asked to watch it, and a
            # packet is not a reason to take it away mid-flight. The
            # packet is decoded and written to this session's files
            # exactly as always, and its samples accumulate in the live
            # buffers underneath. Saying so loudly is the whole safety
            # argument: the plots are showing a file, the vehicle is
            # talking, and both facts have to be visible at once.
            self.sim.live_packets += 1
            self.note(f"LIVE telemetry is arriving while the simulation of "
                      f"{self.sim.name} plays. It is being decoded and logged "
                      f"normally; the plots will show it when you press Finish.",
                      "warn", key="live-during-sim")

        self.stats["packets"] += 1
        self.last_packet_rx = rx_time
        self.vehicle_rssi = pkt["rssi"] or None
        for sid, rec in pkt["records"]:
            self.stats["records"] += 1
            self._on_record(sid, rec, rx_time)

    def _on_record(self, sid, rec, rx_time):
        name = STREAM_NAMES[sid]
        self.latest[name] = rec
        # Unconditional, and above everything else in this method: a
        # replay on the plots must never cost the operator a record on
        # disk. The card is the vehicle's log; this is the ground's, and
        # a simulation is a thing being watched, not a mode the station
        # enters.
        self._write_csv(sid, rec, rx_time)
        if self.sim is not None:
            self.sim.live_records += 1

        if sid == dl.STREAM_IMU:
            t = rec["timestamp_us"] / 1e6
            self.imu_history.append((t,
                                     rec["ax"] / G0, rec["ay"] / G0, rec["az"] / G0,
                                     rec["gx"], rec["gy"], rec["gz"]))
        elif sid == dl.STREAM_GPS:
            self._on_gps(rec)

        if sid == dl.STREAM_BARO and self.agl_offset is not None:
            self.agl_history.append((rec["timestamp_us"] / 1e6,
                                     rec["altitude_m"] - self.agl_offset))
        elif sid == dl.STREAM_STATUS:
            if rec["mode"] != self.mode:
                if self.mode is not None:
                    self.note(f"mode {self.mode} -> {rec['mode']}", "event")
                self.mode = rec["mode"]
            self.flags = rec["flags"]
            # The card going away is the confirmation an EJECT SD gets, and
            # it is worth a line in the log either way -- an unmount nobody
            # commanded means the card failed or was pulled live.
            mounted = bool(rec["flags"] & STATUS_FLAG_SD_OK)
            if self.sd_mounted is not None and mounted != self.sd_mounted:
                self.note("SD card MOUNTED" if mounted else
                          "SD card UNMOUNTED -- safe to remove. The vehicle will "
                          "not log again until it is power-cycled.",
                          "event" if not mounted else "warn")
            self.sd_mounted = mounted
            # flight_logic owns the pad reference. Assuming its AGL is the
            # barometric altitude minus that reference, the latest baro
            # sample recovers the offset, and every baro record after that
            # can be plotted as AGL at full downlink rate instead of 1 Hz.
            baro = self.latest.get("baro")
            if baro:
                self.agl_offset = baro["altitude_m"] - rec["altitude_agl_m"]
            else:
                self.agl_history.append((rec["timestamp_us"] / 1e6, rec["altitude_agl_m"]))
        elif sid == dl.STREAM_EVENT:
            self._on_event(rec)
        elif sid == dl.STREAM_ERR:
            sup = f" (+{rec['suppressed']} suppressed)" if rec["suppressed"] else ""
            # The fault is the source/code/esp_err triple. detail and the
            # suppressed count are per-occurrence and deliberately left
            # out of the key, or a sensor failing continuously would
            # produce a new line per sample -- which is the flood.
            self.note(f"vehicle ERR {rec['source']} {rec['code']} esp_err={rec['esp_err']} "
                      f"detail={rec['detail']}{sup}", "error",
                      key=f"err-{rec['source']}-{rec['code']}-{rec['esp_err']}")

    def _on_event(self, rec):
        typ = rec["type"]

        marked = classify_event(rec)
        if marked:
            label, group = marked
            self.events.append((rec["timestamp_us"] / 1e6, label, group))

        text = f"vehicle EVENT {typ} ch={rec['channel']} detail={rec['detail']}"
        if typ == "UPLINK_COMMAND":
            op = rec["channel"]
            # telemetry.c logs detail=seq for accepted commands, detail=arg for MARK
            for info in list(self.awaiting_ack):
                if info["opcode"] == op and rec["detail"] == (
                        info["arg"] if op == up.MARK else info["seq"]):
                    self.awaiting_ack.remove(info)
                    text = (f"vehicle ACCEPTED {info['name']} seq={info['seq']} "
                            f"({time.time() - info['sent']:.1f} s after sending)")
                    break
            else:
                text = (f"vehicle EVENT UPLINK_COMMAND {OPCODE_NAMES.get(op, op)} "
                        f"detail={rec['detail']}")
        elif typ == "UPLINK_REJECTED":
            text += "  <- the vehicle refused a command"
        elif typ == "CONFIG_RX":
            # One per chunk. Keyed so the log shows a single line whose
            # count climbs, rather than twenty lines of progress.
            self.note(f"config upload: chunk {rec['channel']} of {rec['detail']} received",
                      "event", key="config-rx")
            return
        elif typ == "CONFIG_STORED":
            self.config_status = ("stored", rec["detail"])
            if self.upload:
                self.upload["verdict"] = "stored"
            self.note(f"vehicle STORED the config ({rec['detail']} bytes) in NVS. "
                      f"It takes effect on reboot, or now if you press Apply.", "event")
            return
        elif typ == "CONFIG_REJECTED":
            why = CFG_REJECT_NAMES.get(rec["channel"], f"reason {rec['channel']}")
            detail = CFG_REJECT_DETAIL.get(rec["channel"], "detail")
            self.config_status = ("rejected", why)
            if self.upload:
                self.upload["verdict"] = f"rejected: {why}"
            self.note(f"vehicle REJECTED the config -- {why} ({detail}={rec['detail']}). "
                      f"Nothing was stored; send it again.", "error",
                      key=f"config-rejected-{rec['channel']}")
            return
        elif typ == "CONFIG_APPLIED":
            self.config_status = ("applied", rec["detail"])
            self.note("vehicle APPLIED the stored config -- it is live now", "event")
            return
        self.note(text, "event")

    # ------------------------------------------------------------- summaries
    # ------------------------------------------------------------ GPS
    def _on_gps(self, rec):
        """Track the vehicle's position relative to the PAD, not the globe.

        A raw latitude and longitude tell you nothing at a glance -- six
        decimal places that all look the same. What an operator actually
        needs is 'how far, which way, from where it left', so the first
        valid fix becomes the origin and everything after it is metres
        east and north of that."""
        if not rec["fix_quality"]:
            return                      # a position with no fix is not a position
        lat, lon = rec["latitude"], rec["longitude"]
        if self.pad_fix is None:
            self.pad_fix = (lat, lon)
            self.note(f"GPS pad reference set: {lat:.6f}, {lon:.6f}", "event")
        self.gps_track.append((rec["timestamp_us"] / 1e6, *self.gps_offset_m(lat, lon)))

    def gps_offset_m(self, lat, lon):
        """(east, north) metres from the pad.

        Equirectangular about the pad latitude. Good to well under a
        metre over the few kilometres a recovery walk covers, which is
        the only range this is ever asked about."""
        if self.pad_fix is None:
            return (0.0, 0.0)
        lat0, lon0 = self.pad_fix
        R = 6371000.0
        east  = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
        north = math.radians(lat - lat0) * R
        return (east, north)

    def gps_from_pad(self):
        """(distance_m, bearing_deg) of the LAST fix from the pad, or None.

        Deliberately the last fix rather than the current one: after the
        vehicle is on the ground and the radio has stopped, this is the
        number you walk out with, and it must survive the link going
        away."""
        if not self.gps_track:
            return None
        _, east, north = self.gps_track[-1]
        dist = math.hypot(east, north)
        bearing = math.degrees(math.atan2(east, north)) % 360.0
        return (dist, bearing)

    # ------------------------------------------------------- simulation
    @property
    def sim_name(self):
        """The CSV being replayed, or None. What every view checks."""
        return self.sim.name if self.sim else None

    def start_simulation(self, name, alt_rows, imu_rows, sim_events,
                         speed=1.0, summary=None):
        """Begin replaying a CSV flight onto the plots, at its own rate.

        The plots start empty and fill the way live telemetry fills
        them. A simulation that appeared complete in one frame would
        show you the answer without ever showing you the flight, and
        the answer is not the interesting part: what matters is WHEN
        each channel called it, and what the altitude and acceleration
        were on screen at that moment.

        Live telemetry is not interrupted. It keeps being decoded and
        written to this session's files underneath the replay, and
        comes back onto the plots -- including everything that arrived
        while the replay played -- when it is finished."""
        self.sim = SimPlayback(name, alt_rows, imu_rows, sim_events,
                               speed=speed, summary=summary)
        s = self.sim
        self.note(f"simulation {name}: replaying {len(s.sched)} rows over "
                  f"{s.duration:.1f} s of flight at {s.speed:g}x. Live telemetry "
                  f"keeps logging to this session's files while it plays.", "event")
        return s

    def sim_tick(self, now=None):
        """Advance a replay. Called from poll(), so it runs whether or not
        the plots window happens to be open."""
        if self.sim is None:
            return 0
        s = self.sim
        was_done = s.done
        released = s.tick(now if now is not None else time.monotonic())
        if s.done and not was_done:
            self.note(f"simulation {s.name}: playback complete. The plots are "
                      f"frozen on the finished flight -- press Finish to return "
                      f"them to live telemetry.", "event")
            # The verdict is held back until now on purpose: printing
            # "Main fires at 34.0 s" before the replay starts tells you
            # the ending of the thing you sat down to watch.
            if not s.summary_done:
                s.summary_done = True
                for text, level in s.summary:
                    self.note(text, level)
        return released

    def finish_simulation(self):
        """End a replay and give the plots back to live telemetry."""
        if self.sim is None:
            return None
        s, self.sim = self.sim, None
        if not s.summary_done:
            # Finished early. The verdict is still owed.
            s.summary_done = True
            for text, level in s.summary:
                self.note(text, level)
        extra = ""
        if s.live_records:
            extra = (f" {s.live_records} live records arrived while it played; "
                     f"they were logged throughout and are on the plots now.")
        self.note(f"simulation {s.name} finished -- the plots are live again.{extra}",
                  "event")
        return s

    # Kept as the old name: ending a replay and discarding it are the
    # same operation, since the live history was never overwritten.
    clear_simulation = finish_simulation

    # --------------------------------------------------- what the views read
    # Every plot asks for its data here rather than reaching into the
    # buffers, which is the single place a replay is substituted for
    # live telemetry. Anything that read a buffer directly would keep
    # drawing live data underneath a banner saying SIMULATION.
    def plot_agl(self):
        return self.sim.agl if self.sim else self.agl_history

    def plot_imu(self):
        return self.sim.imu if self.sim else self.imu_history

    def plot_track(self):
        return self.sim.track if self.sim else self.gps_track

    def plot_events(self):
        return self.sim.events if self.sim else self.events

    def clear_plots(self):
        """Wipe the LIVE plot history -- altitude, IMU, track, event markers.

        What this is for: the plots use the vehicle's own clock
        (timestamp_us, microseconds since ITS boot). Reset or power-cycle
        the flight computer and that clock starts again from zero, so the
        new samples land on top of the old ones instead of after them.

        Only the on-screen buffers go. Every record is already in this
        session's CSVs and raw.bin, the GPS pad origin is kept (use
        Re-zero pad for that), and a replay in progress is untouched.
        """
        self.agl_history.clear()
        self.imu_history.clear()
        self.gps_track.clear()
        self.events.clear()
        self.note("plots cleared (session files untouched)")

    def reset_pad_fix(self):
        """Re-zero the ground track on the next valid fix."""
        self.pad_fix = None
        self.gps_track.clear()
        self.note("GPS pad reference cleared -- next fix becomes the origin", "warn")

    def sd_auto_eject_in(self):
        """Seconds until the vehicle unmounts its own card, or None.

        Derived from the uptime in the status record rather than from
        when this program started, so it stays right across a ground
        station restart and is not fooled by a vehicle that was already
        powered when we connected.

        Returns a negative number once the deadline has passed, which
        happens whenever the vehicle is ARMED or in FLIGHT -- the
        vehicle defers the unmount rather than enforcing it, so "overdue"
        is a normal state and the caller renders it as deferred."""
        st = self.latest.get("status")
        if not SD_AUTO_EJECT_S or not st or self.sd_mounted is not True:
            return None
        # uptime_s is a uint16 and wraps at 65535 s (18 h 12 m). The
        # VEHICLE's timer does not -- it works off a 64-bit timer, so a
        # long timeout still fires correctly -- but this countdown cannot
        # be derived from a field that has wrapped, so it declines to
        # guess rather than showing a number that resets every 18 hours.
        if SD_AUTO_EJECT_S >= 0xFFFF:
            return None
        return SD_AUTO_EJECT_S - st["uptime_s"]

    def link_age(self):
        return None if self.last_packet_rx is None else time.time() - self.last_packet_rx

    def flag_text(self):
        if self.flags is None:
            return "--"
        names = [n for bit, n in STATUS_FLAG_NAMES.items() if self.flags & bit]
        unnamed = self.flags & ~sum(STATUS_FLAG_NAMES)
        if unnamed:
            names.append(f"0x{unnamed:04X}")
        return " ".join(names) or "none"

    def summary_line(self):
        s, st = self.stats, self.latest.get("status", {})
        batt = self.latest.get("batt", {})
        age = self.link_age()
        bad = s["pkt_crc"] + s["pkt_malformed"] + s["pkt_size"]
        return (f"mode {self.mode or '--'}  "
                f"AGL {st.get('altitude_agl_m', float('nan')):.1f} m  "
                f"batt {batt.get('battery_v', float('nan')):.2f} V  |  "
                f"pkts {s['packets']}  lost {self.seq.lost}  bad {bad}  dup {self.seq.duplicates}  "
                f"rssi dn/up {fmt_rssi(self.ground_rssi)}/{fmt_rssi(self.vehicle_rssi)}  "
                f"last {'--' if age is None else f'{age:.1f} s'}")


def fmt_rssi(v):
    return "--" if not v else f"-{v} dBm"


# --------------------------------------------------------------------------
# Link signal bars
#
# A phone-style 0..4 bar meter beside each RSSI readout. The radio reports
# RSSI as a positive magnitude (ATDB 0x53 = -83 dBm), so smaller is
# stronger. The 900HP at 200 kb/s stops hearing anything around -100 dBm,
# so a packet that was heard at all earns one bar; zero bars means no
# reading.
# --------------------------------------------------------------------------
RSSI_BAR_THRESHOLDS = (90, 80, 70)      # |dBm| at or below this: 2, 3, 4 bars
SIGNAL_BAR_COLORS = {0: "#b00020", 1: "#b00020", 2: "#a05a00", 3: "#38761d", 4: "#38761d"}
SIGNAL_BAR_EMPTY = "#d0d0d0"
SIGNAL_BAR_STALE = "#9a9a9a"


def rssi_bars(rssi):
    """Number of bars (0-4) for an RSSI magnitude in -dBm, or None."""
    if not rssi:
        return 0
    return 1 + sum(1 for t in RSSI_BAR_THRESHOLDS if rssi <= t)




# ==========================================================================
# Deployment config: model, validation, and the editor window
# ==========================================================================

DEFAULT_DELAY_MS = 1500          # what flight_logic uses when the key is absent


def blank_config():
    """The config the vehicle falls back to when NVS is empty.

    Every event disabled on every channel. flight_logic treats an empty
    condition set as NEVER satisfied, so this is the safe state, not a
    blank slate that happens to do nothing."""
    return {"channels": {name: {"primary": {}, "secondary": {}, "thresholds": {},
                                "latch": False, "delay_ms": DEFAULT_DELAY_MS,
                                "manual_arm": False}
                         for name in CHANNEL_KEYS}}


def validate_config(cfg):
    """Return a list of human-readable problems; empty means it is sendable.

    Checked here rather than left to the vehicle because the vehicle
    cannot report most of these: cJSON silently ignores a key it does
    not recognise, so a misspelled event name uploads cleanly, stores
    cleanly, and simply never fires. That is the failure this function
    exists to prevent."""
    problems = []
    if not isinstance(cfg, dict):
        return ["the config must be a JSON object"]
    channels = cfg.get("channels")
    if not isinstance(channels, dict):
        return ["missing a 'channels' object -- the vehicle would fall back "
                "to safe defaults and ignore the whole file"]

    for name in channels:
        if name not in CHANNEL_KEYS:
            problems.append(f"unknown channel '{name}' (expected one of "
                            f"{', '.join(CHANNEL_KEYS)}) -- it would be ignored")
    for name in CHANNEL_KEYS:
        ch = channels.get(name)
        if ch is None:
            continue
        if not isinstance(ch, dict):
            problems.append(f"{name}: must be an object")
            continue
        for setname in ("primary", "secondary", "thresholds"):
            sub = ch.get(setname)
            if sub is None:
                continue
            if not isinstance(sub, dict):
                problems.append(f"{name}.{setname}: must be an object")
                continue
            for key in sub:
                if key not in EVENT_KEYS:
                    problems.append(f"{name}.{setname}: unknown event '{key}' "
                                    f"-- the vehicle would ignore it silently")
            if setname == "thresholds":
                for key, v in sub.items():
                    if not isinstance(v, (int, float)) or isinstance(v, bool):
                        problems.append(f"{name}.thresholds.{key}: "
                                        f"must be a number, got {v!r}")
            else:
                for key, v in sub.items():
                    if not isinstance(v, bool):
                        problems.append(f"{name}.{setname}.{key}: "
                                        f"must be true or false, got {v!r}")
        if "delay_ms" in ch and (not isinstance(ch["delay_ms"], int)
                                 or isinstance(ch["delay_ms"], bool)
                                 or ch["delay_ms"] < 0):
            problems.append(f"{name}.delay_ms: must be a whole number of "
                            f"milliseconds, got {ch['delay_ms']!r}")
        for flag in ("latch", "manual_arm"):
            if flag in ch and not isinstance(ch[flag], bool):
                problems.append(f"{name}.{flag}: must be true or false")

        # An enabled event with no threshold is the quiet mistake: it
        # reads as configured and compares against 0.
        thresholds = ch.get("thresholds") or {}
        for setname in ("primary", "secondary"):
            for key, on in (ch.get(setname) or {}).items():
                if on and key in EVENT_MEANING and EVENT_MEANING[key][1] \
                        and key not in thresholds:
                    problems.append(f"{name}.{setname}.{key} is enabled but has "
                                    f"no threshold -- it would compare against 0")
    return problems


def config_to_text(cfg):
    """Canonical JSON for sending and for the raw view.

    Sorted keys and a compact separator: sorted so two configs that
    differ only in key order produce the same bytes and therefore the
    same transfer CRC, and compact because every byte is a byte over
    the radio."""
    return json.dumps(cfg, indent=2, sort_keys=True)


def open_config_panel(parent, gs, mono, send_command, on_simulated=None):
    """The flight parameters editor.

    A Toplevel rather than another pane in the main window: the main
    window is a live instrument you watch during a flight, and this is a
    thing you sit and edit on the pad. Mixing them would mean the
    altitude plot competing for space with 72 checkboxes."""
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    win = tk.Toplevel(parent)
    win.title("Flight parameters")
    win.geometry("900x700")

    state = {"path": None, "syncing": False}

    # -- per-channel widget variables --------------------------------------
    vars_ = {}
    for name in CHANNEL_KEYS:
        vars_[name] = {
            "primary": {k: tk.BooleanVar(value=False) for k in EVENT_KEYS},
            "secondary": {k: tk.BooleanVar(value=False) for k in EVENT_KEYS},
            "thresholds": {k: tk.StringVar(value="") for k in EVENT_KEYS},
            "latch": tk.BooleanVar(value=False),
            "delay_ms": tk.StringVar(value=str(DEFAULT_DELAY_MS)),
            "manual_arm": tk.BooleanVar(value=False),
        }

    nb = ttk.Notebook(win)
    nb.pack(fill="both", expand=True, padx=6, pady=6)

    def build_channel_tab(name):
        outer = ttk.Frame(nb)
        head = ttk.Frame(outer, padding=(6, 6, 6, 2))
        head.pack(fill="x")
        ttk.Checkbutton(head, text="latch", variable=vars_[name]["latch"]).pack(side="left")
        ttk.Label(head, text="   delay_ms").pack(side="left")
        ttk.Entry(head, textvariable=vars_[name]["delay_ms"], width=8).pack(side="left", padx=4)
        ttk.Checkbutton(head, text="manual_arm",
                        variable=vars_[name]["manual_arm"]).pack(side="left", padx=(12, 0))
        ttk.Label(head, text="a channel fires when its PRIMARY set is fully true, "
                             "or its SECONDARY set is", foreground="#666").pack(side="right")

        # A canvas because 18 rows plus headers do not fit a short window,
        # and a config you have to resize the window to finish editing is
        # a config someone will finish editing badly.
        canvas = tk.Canvas(outer, highlightthickness=0)
        sb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        grid = ttk.Frame(canvas)
        grid.bind("<Configure>",
                  lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=grid, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True, padx=(6, 0), pady=4)

        hdr = ("event", "pri", "sec", "threshold", "", "tests")
        for c, t in enumerate(hdr):
            ttk.Label(grid, text=t, font=("Helvetica", 9, "bold")).grid(
                row=0, column=c, sticky="w", padx=4, pady=(0, 4))

        for r, key in enumerate(EVENT_KEYS, start=1):
            meaning, unit = EVENT_MEANING[key]
            ttk.Label(grid, text=key, font=mono).grid(row=r, column=0, sticky="w", padx=4)
            ttk.Checkbutton(grid, variable=vars_[name]["primary"][key]).grid(
                row=r, column=1, padx=4)
            ttk.Checkbutton(grid, variable=vars_[name]["secondary"][key]).grid(
                row=r, column=2, padx=4)
            e = ttk.Entry(grid, textvariable=vars_[name]["thresholds"][key],
                          width=10, font=mono)
            e.grid(row=r, column=3, padx=4)
            if unit is None:
                # No threshold is read for this event, so offering a box
                # would invite someone to type a number that does nothing.
                e.state(["disabled"])
            ttk.Label(grid, text=unit or "", foreground="#666").grid(
                row=r, column=4, sticky="w")
            ttk.Label(grid, text=meaning, foreground="#444").grid(
                row=r, column=5, sticky="w", padx=(8, 4))
        return outer

    for name in CHANNEL_KEYS:
        nb.add(build_channel_tab(name), text=name)

    raw_frame = ttk.Frame(nb)
    raw = tk.Text(raw_frame, font=mono, wrap="none", undo=True)
    raw_sb = ttk.Scrollbar(raw_frame, command=raw.yview)
    raw.configure(yscrollcommand=raw_sb.set)
    raw_sb.pack(side="right", fill="y")
    raw.pack(fill="both", expand=True, padx=6, pady=6)
    nb.add(raw_frame, text="Raw JSON")
    RAW_TAB = len(CHANNEL_KEYS)

    # -- model <-> widgets --------------------------------------------------
    def grid_to_cfg():
        cfg = {"channels": {}}
        for name in CHANNEL_KEYS:
            v = vars_[name]
            ch = {"primary": {}, "secondary": {}, "thresholds": {}}
            for setname in ("primary", "secondary"):
                for key in EVENT_KEYS:
                    if v[setname][key].get():
                        ch[setname][key] = True
            for key in EVENT_KEYS:
                txt = v["thresholds"][key].get().strip()
                if txt:
                    try:
                        ch["thresholds"][key] = float(txt)
                    except ValueError:
                        # Kept as the raw string so validate_config can
                        # name it, instead of vanishing on the way out.
                        ch["thresholds"][key] = txt
            ch["latch"] = v["latch"].get()
            delay = v["delay_ms"].get().strip()
            try:
                ch["delay_ms"] = int(delay)
            except ValueError:
                ch["delay_ms"] = delay
            ch["manual_arm"] = v["manual_arm"].get()
            cfg["channels"][name] = ch
        return cfg

    def cfg_to_grid(cfg):
        channels = (cfg or {}).get("channels") or {}
        for name in CHANNEL_KEYS:
            ch = channels.get(name) or {}
            v = vars_[name]
            for setname in ("primary", "secondary"):
                sub = ch.get(setname) or {}
                for key in EVENT_KEYS:
                    v[setname][key].set(bool(sub.get(key)))
            th = ch.get("thresholds") or {}
            for key in EVENT_KEYS:
                if key in th:
                    val = th[key]
                    v["thresholds"][key].set(
                        f"{val:g}" if isinstance(val, (int, float))
                        and not isinstance(val, bool) else str(val))
                else:
                    v["thresholds"][key].set("")
            v["latch"].set(bool(ch.get("latch", False)))
            v["delay_ms"].set(str(ch.get("delay_ms", DEFAULT_DELAY_MS)))
            v["manual_arm"].set(bool(ch.get("manual_arm", False)))

    def set_raw(cfg):
        raw.delete("1.0", "end")
        raw.insert("1.0", config_to_text(cfg))

    def read_raw():
        return json.loads(raw.get("1.0", "end"))

    # The two views are kept in step at the moment of switching, not on
    # every keystroke: live two-way binding of a free-text JSON editor
    # means every half-typed character is a parse error.
    def on_tab_changed(_event=None):
        if state["syncing"] or not state.get("ready"):
            return
        state["syncing"] = True
        try:
            idx = nb.index(nb.select())
            if idx == RAW_TAB:
                # `bounce` means we are arriving back here because the JSON
                # would not parse. Regenerating the text from the grid now
                # would overwrite the very edit the user has to fix --
                # which is worse than the typo.
                if not state.pop("bounce", False):
                    set_raw(grid_to_cfg())
                state["was_raw"] = True
                return
            if state.get("was_raw"):
                try:
                    cfg = read_raw()
                except json.JSONDecodeError as e:
                    # Bounce back and report in the status line rather than
                    # a modal. This handler fires on EVERY tab click, and a
                    # dialog you must dismiss to get back to the text you
                    # were editing is more disruptive than the mistake.
                    state["bounce"] = True
                    nb.select(RAW_TAB)
                    state["was_raw"] = True
                    say(f"JSON will not parse, so nothing was changed: {e}", "#b00020")
                    return
                cfg_to_grid(cfg)
            state["was_raw"] = False
        finally:
            state["syncing"] = False

    nb.bind("<<NotebookTabChanged>>", on_tab_changed)

    def current_cfg():
        """Whichever view is in front is the authority."""
        if nb.index(nb.select()) == RAW_TAB:
            return read_raw()
        return grid_to_cfg()

    # -- buttons ------------------------------------------------------------
    bar = ttk.Frame(win, padding=6)
    bar.pack(fill="x")
    status = ttk.Label(win, text="", padding=(8, 0, 8, 6), foreground="#444")
    status.pack(fill="x")

    def say(text, colour="#444"):
        status.configure(text=text, foreground=colour)

    def do_load():
        path = filedialog.askopenfilename(
            parent=win, title="Load deployment config",
            filetypes=[("JSON", "*.json"), ("All files", "*")])
        if not path:
            return
        try:
            with open(path) as f:
                cfg = json.load(f)
        except Exception as e:
            messagebox.showerror("Load failed", str(e), parent=win)
            return
        problems = validate_config(cfg)
        cfg_to_grid(cfg)
        set_raw(cfg)
        state["path"] = path
        win.title(f"Flight parameters -- {os.path.basename(path)}")
        if problems:
            say(f"loaded with {len(problems)} problem(s) -- press Check", "#b00020")
        else:
            say(f"loaded {os.path.basename(path)}")

    def do_save():
        try:
            cfg = current_cfg()
        except json.JSONDecodeError as e:
            messagebox.showerror("Raw JSON", str(e), parent=win)
            return
        path = filedialog.asksaveasfilename(
            parent=win, title="Save deployment config", defaultextension=".json",
            initialfile=os.path.basename(state["path"] or "config.json"),
            filetypes=[("JSON", "*.json"), ("All files", "*")])
        if not path:
            return
        try:
            with open(path, "w") as f:
                f.write(config_to_text(cfg) + "\n")
        except Exception as e:
            messagebox.showerror("Save failed", str(e), parent=win)
            return
        state["path"] = path
        win.title(f"Flight parameters -- {os.path.basename(path)}")
        say(f"saved to {os.path.basename(path)}")

    def do_check(quiet=False):
        try:
            cfg = current_cfg()
        except json.JSONDecodeError as e:
            if not quiet:
                messagebox.showerror("Raw JSON", str(e), parent=win)
            return None
        problems = validate_config(cfg)
        if problems:
            if not quiet:
                messagebox.showwarning(
                    "Check", "The vehicle would accept this file but not do what "
                             "you mean:\n\n  - " + "\n  - ".join(problems[:20]),
                    parent=win)
            say(f"{len(problems)} problem(s)", "#b00020")
            return None
        n = sum(1 for name in CHANNEL_KEYS
                for setname in ("primary", "secondary")
                for k, on in (cfg["channels"].get(name, {}).get(setname) or {}).items()
                if on)
        say(f"looks good: {len(config_to_text(cfg).encode())} bytes, "
            f"{n} enabled condition(s)", "#38761d")
        return cfg

    def do_send():
        cfg = do_check()
        if cfg is None:
            return
        if not gs.connected:
            messagebox.showerror("Send", "Not connected to a radio.", parent=win)
            return
        text = config_to_text(cfg)
        if not messagebox.askyesno(
                "Send config",
                f"Upload {len(text.encode())} bytes to the vehicle?\n\n"
                f"This stores it in NVS. It does NOT take effect until you press "
                f"Apply, or the vehicle reboots.", parent=win):
            return
        try:
            chunks, crc = gs.send_config(text)
        except Exception as e:
            messagebox.showerror("Send failed", str(e), parent=win)
            return
        say(f"sending {chunks} chunks, crc 0x{crc:04X}...", "#0b5394")

    def do_apply():
        if "config_apply" not in gs.allowed_commands():
            messagebox.showerror(
                "Apply", "The vehicle will only apply a config in STANDBY or "
                         "LANDED. It is currently "
                         f"{gs.mode or 'not reporting a mode'}.", parent=win)
            return
        if not messagebox.askyesno(
                "Apply config",
                "Make the config stored on the vehicle the live one?\n\n"
                "This replaces the running deployment conditions and resets "
                "each channel's arm state.", parent=win):
            return
        send_command("config_apply")
        say("apply sent -- watch for CONFIG_APPLIED in the log", "#0b5394")

    def do_simulate():
        """Run this config against a CSV of flight data."""
        cfg = do_check()
        if cfg is None:
            return
        path = filedialog.askopenfilename(
            parent=win, title="Flight data CSV to simulate",
            filetypes=[("CSV", "*.csv"), ("All files", "*")])
        if not path:
            return
        if gs.connected and gs.latest:
            if not messagebox.askyesno(
                    "Simulate",
                    "The plots are showing LIVE telemetry.\n\n"
                    "The replay takes over the plots while it plays. The "
                    "vehicle's telemetry keeps being decoded and written to "
                    "this session's files the whole time, and returns to the "
                    "plots when you press Finish.\n\nContinue?", parent=win):
                return
        say("simulating...", "#0b5394")
        win.update_idletasks()
        # Two separate try blocks, not one with two except clauses: these
        # are different failures with different fixes, and lumping them
        # together is how "the simulator binary is the wrong
        # architecture" got reported as "Could not read the CSV".
        try:
            events, notes = run_flight_sim(path, config_to_text(cfg))
        except RuntimeError as e:
            messagebox.showerror("Simulator", str(e), parent=win)
            say("simulation failed", "#b00020")
            return
        try:
            alt_rows, imu_rows = load_sim_csv(path)
        except Exception as e:
            messagebox.showerror("Simulate", f"Could not read the CSV:\n\n{e}",
                                 parent=win)
            say("simulation failed", "#b00020")
            return

        base = os.path.basename(path)

        # The verdict is assembled now but handed to the player, which
        # logs it when the replay reaches the end. Announcing "Main
        # fires at 34.0 s" before the playback starts would give away
        # the ending of the thing the operator sat down to watch, and
        # the timing against the altitude trace is the whole point.
        fired = [(t, n, c) for t, n, c, _d in events if n == "DEPLOY_PREDICTED"]
        summary = [(f"simulated {base}: " + "; ".join(notes), "event")]
        if fired:
            summary.append(
                ("channels that would fire: " +
                 ", ".join(f"{CHANNEL_KEYS[c] if c < len(CHANNEL_KEYS) else c}"
                           f" at {t:.2f}s" for t, _n, c in fired), "event"))
        else:
            summary.append(("no channel reached its condition in this flight",
                            "warn"))

        speed = 1.0
        span = alt_rows[-1][0] - alt_rows[0][0] if len(alt_rows) > 1 else 0.0
        gs.start_simulation(base, alt_rows, imu_rows, events,
                            speed=speed, summary=summary)
        say(f"replaying {base} \u2014 {span:.0f} s of flight, in the plots window",
            "#0b5394")
        if on_simulated:
            on_simulated()

    for text, fn in (("Load...", do_load), ("Save...", do_save),
                     ("Check", do_check), ("Simulate on CSV...", do_simulate)):
        ttk.Button(bar, text=text, command=fn).pack(side="left", padx=3)
    ttk.Button(bar, text="Close", command=win.destroy).pack(side="right", padx=3)
    ttk.Button(bar, text="Apply on vehicle", command=do_apply).pack(side="right", padx=3)
    ttk.Button(bar, text="Send to vehicle", command=do_send).pack(side="right", padx=3)

    # -- live upload progress ----------------------------------------------
    def poll_progress():
        if not win.winfo_exists():
            return
        prog = gs.upload_progress()
        if prog:
            sent, total = prog
            say(f"uploading... {sent}/{total} packets", "#0b5394")
        elif gs.config_status:
            kind, detail = gs.config_status
            if kind == "stored":
                say(f"vehicle stored {detail} bytes. Press Apply to make it live.",
                    "#38761d")
            elif kind == "applied":
                say("vehicle applied the stored config -- it is live now", "#38761d")
            else:
                say(f"vehicle rejected it: {detail}", "#b00020")
        win.after(200, poll_progress)

    cfg_to_grid(blank_config())
    set_raw(blank_config())
    state["was_raw"] = False
    # Only now: tab-changed fires while the notebook is being built, before
    # say() and the rest of this closure exist.
    state["ready"] = True
    poll_progress()
    return win




# ==========================================================================
# Offline simulation
#
# Feeds a CSV of flight data through the REAL deployment logic -- the
# compiled flight_logic.c, not a reimplementation of it -- and reports
# what the flight computer would have done with the config you are
# holding.
#
# The temptation was to port deploy_logic_update() to Python. That
# produces a simulator which is wrong the first time anyone edits
# flight_logic.c, and wrong SILENTLY, while somebody is deciding whether
# a deployment config is safe to fly. Shelling out to a binary built
# from the firmware's own source is slower to set up and cannot lie.
# ==========================================================================

SIM_BUILD_HINT = ("Build it once with:\n\n"
                  "    ./test/host/build_sim.sh\n\n"
                  "It needs only a host C compiler -- no ESP-IDF. On Windows "
                  "that means Git Bash, MSYS2/MinGW or WSL.")


WRONG_PLATFORM_HINT = (
    "The simulator next to ground_station.py was built for a DIFFERENT "
    "platform -- most often a Linux binary sitting in a Windows checkout, "
    "left behind by running build_sim.sh under WSL or a mounted share.\n\n"
    "Windows needs flight_sim.exe, built with MinGW-w64 (MSYS2). Git Bash "
    "alone will not do it -- it ships no C compiler.")


def find_flight_sim(here=None, osname=None):
    # `here` and `osname` are injectable so the platform choice can be
    # tested from either platform. This is the bug that shipped a Linux
    # binary into a Windows checkout; it gets a regression test.
    """Path to a simulator binary THIS platform can execute, or None.

    Platform-specific names only. It is tempting to accept either name
    and let os.access(X_OK) sort it out, but that check is useless on
    Windows -- it returns True for any file that exists, including a
    Linux ELF. The binary then fails at exec time with WinError 193,
    far from here, and the error gets attributed to whatever the caller
    happened to be doing at the time.
    """
    here = here or os.path.dirname(os.path.abspath(__file__))
    osname = osname or os.name
    name = "flight_sim.exe" if osname == "nt" else "flight_sim"
    cand = os.path.join(here, name)
    if os.path.isfile(cand) and (osname == "nt" or os.access(cand, os.X_OK)):
        return cand
    from shutil import which
    return which("flight_sim")


def parse_sim_output(text):
    """(events, notes) from the simulator's stdout.

    events: [(t_s, EVENT_NAME, channel, detail)]
    notes:  the "# ..." summary lines, verbatim
    """
    events, notes = [], []
    for line in text.splitlines():
        line = line.rstrip()
        if not line:
            continue
        if line.startswith("#"):
            notes.append(line.lstrip("# ").rstrip())
            continue
        parts = line.split("\t")
        if len(parts) != 4:
            continue
        try:
            events.append((float(parts[0]), parts[1],
                           int(parts[2]), int(parts[3])))
        except ValueError:
            continue
    return events, notes


def run_flight_sim(csv_path, config_text, timeout=60):
    """Run the simulator. Returns (events, notes). Raises RuntimeError."""
    exe = find_flight_sim()
    if exe is None:
        raise RuntimeError("The flight simulator has not been built yet.\n\n"
                           + SIM_BUILD_HINT)
    tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False,
                                      encoding="utf-8")
    try:
        tmp.write(config_text)
        tmp.close()
        proc = subprocess.run([exe, csv_path, tmp.name], capture_output=True,
                              text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"The simulator did not finish within {timeout}s.")
    except OSError as e:
        # Raised as RuntimeError like every other failure in here, so the
        # caller cannot mistake "the binary would not run" for "the CSV
        # would not parse" -- which is exactly what happened when this
        # escaped as a bare OSError.
        if getattr(e, "winerror", None) == 193:
            raise RuntimeError(f"{exe}\n\n{e}\n\n{WRONG_PLATFORM_HINT}")
        raise RuntimeError(f"Could not run the simulator at {exe}:\n\n{e}\n\n"
                           + SIM_BUILD_HINT)
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass

    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.strip() or
                           f"simulator exited {proc.returncode}")
    return parse_sim_output(proc.stdout)


def load_sim_csv(path):
    """The CSV's own columns, for plotting alongside the simulated events.

    Parsed here as well as in the simulator on purpose: the simulator
    consumes the file and reports events, and the plots need the traces.
    Both read the same columns, and a file the simulator rejects never
    reaches this."""
    alt, imu = [], []
    with open(path, newline="", encoding="utf-8-sig") as f:
        rdr = csv.DictReader(f)
        cols = {(c or "").strip(): (c or "") for c in (rdr.fieldnames or [])}

        def g(row, name, default=0.0):
            key = cols.get(name)
            if key is None:
                return default
            try:
                return float((row.get(key) or "").strip())
            except (TypeError, ValueError):
                return default

        have_m = "altitude_m" in cols
        have_g = "ax_g" in cols
        for row in rdr:
            if "time_s" not in cols:
                break
            try:
                t = float((row.get(cols["time_s"]) or "").strip())
            except (TypeError, ValueError):
                continue
            a = g(row, "altitude_m") if have_m else g(row, "altitude_ft") * 0.3048
            alt.append((t, a))
            if have_g:
                ax, ay, az = g(row, "ax_g"), g(row, "ay_g"), g(row, "az_g")
            else:
                ax = g(row, "ax") / G0
                ay = g(row, "ay") / G0
                az = g(row, "az") / G0
            imu.append((t, ax, ay, az, g(row, "gx"), g(row, "gy"), g(row, "gz")))
    return alt, imu


# ==========================================================================
# Plotting
#
# Hand-drawn on a tk.Canvas rather than embedding matplotlib: these redraw
# several times a second against live telemetry, and this program's only
# dependency is pyserial. One primitive draws every plot -- four
# copy-pasted routines would drift apart the first time one was fixed.
# ==========================================================================

def _fmt_tick(v, span):
    """A tick label with the precision the RANGE justifies, not the float.

    "%.6g" on an axis spanning 160 m produces -6.65041, which is six
    digits of noise dressed as precision. The span decides: a 1500 m
    altitude axis wants whole metres, a 2 g axis wants two decimals."""
    if span >= 500:
        return f"{v:.0f}"
    if span >= 50:
        return f"{v:.0f}"
    if span >= 5:
        return f"{v:.1f}"
    if span >= 0.5:
        return f"{v:.2f}"
    return f"{v:.3g}"


def draw_xy_plot(canvas, title, series, x_label, y_label,
                 equal_aspect=False, hover=None, empty_text="waiting for data",
                 compass=False, origin=None, markers=(), point_markers=()):
    """Draw `series` on `canvas`.

    series: [{"name": str, "color": hex, "points": [(x, y), ...]}]

    equal_aspect forces one unit of x to occupy the same pixels as one
    unit of y. That is not cosmetic for the ground track: without it a
    drift of 200 m east and 40 m north is drawn as a square, and the
    shape of the track -- which is the entire reason to plot it rather
    than read a number -- is a lie.

    hover: (x, y) in DATA units, or None. Draws a crosshair and reads the
    nearest sample of each series out beside it.

    compass: label the plot edges N / E / S / W. For the ground track,
    where the axes are signed distances, this is what stops "-250 east"
    needing to be read as "250 west" in your head while you are trying
    to work out which way to walk.

    markers: [(x, label, group)] drawn as vertical rules across a time
    axis -- mode changes, Blue Raven signals, this board's own deploy
    calls. Annotations, not data: recessive grey, separated by dash
    pattern, and always behind the traces.

    point_markers: [(x, y, label)] for an axis where a vertical rule
    would be meaningless. The ground track's x is metres east, so "when
    the chute came out" is a POSITION on the track, not a line across
    it.

    origin: (x, y, label) to mark, or None. The ground track's pad is
    always at 0,0 and always within range -- the first fix defines the
    origin -- but "within range" is not the same as findable, especially
    on a westward drift where it ends up hard against the right edge.

    Returns True if a trace was actually drawn. Callers that add their
    own annotations must check it -- a canvas is too small to draw on
    for the first frame or two after its window opens, and an annotation
    added regardless lands on an otherwise blank plot.
    """
    canvas.delete("all")
    w, h = canvas.winfo_width(), canvas.winfo_height()
    if w < 60 or h < 60:
        return False
    pad_l, pad_r, pad_t, pad_b = 56, 26, 30, 30

    canvas.create_text(10, 15, anchor="w", text=title,
                       fill=INK_PRIMARY, font=("Helvetica", 10, "bold"))

    drawn = [sr for sr in series if len(sr["points"]) >= 2]
    if not drawn:
        canvas.create_text(w / 2, h / 2, text=empty_text, fill=INK_MUTED)
        return False

    xs = [p[0] for sr in drawn for p in sr["points"]]
    ys = [p[1] for sr in drawn for p in sr["points"]]
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = min(ys), max(ys)
    if x_hi - x_lo < 1e-9:
        x_lo, x_hi = x_lo - 1, x_hi + 1
    if y_hi - y_lo < 1e-9:
        y_lo, y_hi = y_lo - 1, y_hi + 1
    # A little headroom so a trace never rides the frame.
    y_pad = (y_hi - y_lo) * 0.08
    y_lo, y_hi = y_lo - y_pad, y_hi + y_pad

    plot_w = w - pad_l - pad_r
    plot_h = h - pad_t - pad_b

    if equal_aspect:
        # Grow the SHORTER axis's range until the scales match, keeping
        # each range centred. Never shrink one, which would clip data.
        sx = plot_w / (x_hi - x_lo)
        sy = plot_h / (y_hi - y_lo)
        scale = min(sx, sy)
        cx, cy = (x_lo + x_hi) / 2, (y_lo + y_hi) / 2
        half_x = plot_w / (2 * scale)
        half_y = plot_h / (2 * scale)
        x_lo, x_hi = cx - half_x, cx + half_x
        y_lo, y_hi = cy - half_y, cy + half_y

    def to_px(x, y):
        return (pad_l + (x - x_lo) / (x_hi - x_lo) * plot_w,
                h - pad_b - (y - y_lo) / (y_hi - y_lo) * plot_h)

    # --- recessive grid and labels -------------------------------------
    y_span = y_hi - y_lo
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        gy = y_lo + frac * y_span
        _, py = to_px(x_lo, gy)
        canvas.create_line(pad_l, py, w - pad_r, py, fill=GRID)
        canvas.create_text(pad_l - 6, py, anchor="e", text=_fmt_tick(gy, y_span),
                           fill=INK_MUTED, font=("Helvetica", 8))
    # Zero line, when zero is in view and is not already the frame. Accel
    # and gyro are signed and a sign change is the interesting part.
    if y_lo < 0 < y_hi:
        _, pz = to_px(x_lo, 0.0)
        canvas.create_line(pad_l, pz, w - pad_r, pz, fill="#cfcfcb", dash=(3, 3))

    x_span = x_hi - x_lo
    canvas.create_text(pad_l, h - 9, anchor="w", text=_fmt_tick(x_lo, x_span),
                       fill=INK_MUTED, font=("Helvetica", 8))
    canvas.create_text(w - pad_r, h - 9, anchor="e", text=_fmt_tick(x_hi, x_span),
                       fill=INK_MUTED, font=("Helvetica", 8))
    canvas.create_text((pad_l + w - pad_r) / 2, h - 9, text=x_label,
                       fill=INK_SECONDARY, font=("Helvetica", 8))
    canvas.create_text(9, pad_t + plot_h / 2, text=y_label, angle=90,
                       fill=INK_SECONDARY, font=("Helvetica", 8))

    # --- origin marker, beneath the traces so it never hides data ------
    if origin is not None:
        ox, oy, olabel = origin
        if x_lo <= ox <= x_hi and y_lo <= oy <= y_hi:
            px, py = to_px(ox, oy)
            canvas.create_line(px - 7, py, px + 7, py, fill=INK_SECONDARY)
            canvas.create_line(px, py - 7, px, py + 7, fill=INK_SECONDARY)
            canvas.create_oval(px - 4, py - 4, px + 4, py + 4,
                               outline=INK_SECONDARY, width=1)
            # Flip the label inward near the right edge. A westward
            # drift puts the pad hard against it, which is exactly the
            # case where you most want to see where the pad is.
            if px + 9 + 6 * len(olabel) > w - 2:
                canvas.create_text(px - 9, py + 9, anchor="ne", text=olabel,
                                   fill=INK_SECONDARY, font=("Helvetica", 8))
            else:
                canvas.create_text(px + 9, py + 9, anchor="nw", text=olabel,
                                   fill=INK_SECONDARY, font=("Helvetica", 8))

    # --- compass, so a signed axis does not need mental arithmetic -----
    if compass:
        cx = (pad_l + w - pad_r) / 2
        cy = (pad_t + h - pad_b) / 2
        for text, x, y, anchor in (("N", cx, pad_t + 2, "n"),
                                   ("S", cx, h - pad_b - 2, "s"),
                                   ("E", w - pad_r - 3, cy, "e"),
                                   ("W", pad_l + 3, cy, "w")):
            canvas.create_text(x, y, anchor=anchor, text=text,
                               fill="#c2c2bd", font=("Helvetica", 11, "bold"))

    # --- event rules, beneath the traces --------------------------------
    if markers:
        # Labels are rotated so a dozen of them can sit along one axis,
        # and staggered downward when two events land within a few
        # pixels of each other -- which they do exactly when the flight
        # is most interesting (liftoff HW and SW, BR and FC deploys).
        placed = []   # label x positions already taken
        for mx, label, group in sorted(markers, key=lambda m: m[0]):
            if not (x_lo <= mx <= x_hi):
                continue
            px, _ = to_px(mx, y_lo)
            style = MARKER_GROUPS.get(group, MARKER_GROUPS["est"])
            kw = {"fill": style["color"]}
            if style["dash"]:
                kw["dash"] = style["dash"]
            canvas.create_line(px, pad_t, px, h - pad_b, **kw)

            # A rotated label is tall and thin -- about 15px wide -- so
            # crowding is an X problem, and the fix is to spread the
            # labels sideways rather than stack them downward. The
            # events that cluster are the ones worth reading: liftoff HW
            # and SW ~70 ms apart, the FC and BR calls for one deploy a
            # few hundred ms apart. On a 46 s axis those land within a
            # few pixels of each other, and no amount of vertical
            # stacking separates two 15px-wide columns 5px apart.
            lx = px
            while any(abs(lx - qx) < 15 for qx in placed):
                lx += 15
            if lx > w - pad_r - 4:
                continue                  # out of room: rule only, no label
            placed.append(lx)
            if lx != px:
                # Leader back to the rule this label belongs to, so a
                # nudged label is never read against the wrong line.
                canvas.create_line(px, pad_t + 2, lx - 4, pad_t + 2,
                                   fill="#cfcfcb")
            # anchor="ne" with angle=90 hangs the text DOWNWARD from the
            # anchor; "nw" sends it up, which is how these ended up
            # written across the plot titles. Measured, not guessed.
            canvas.create_text(lx - 2, pad_t + 2, anchor="ne", text=label,
                               angle=90, fill=INK_SECONDARY,
                               font=("Helvetica", 7))

    # --- the traces ----------------------------------------------------
    for sr in drawn:
        coords = [c for p in sr["points"] for c in to_px(*p)]
        canvas.create_line(*coords, fill=sr["color"], width=2,
                           capstyle="round", joinstyle="round")

    # --- identity: legend AND a direct label on each trace -------------
    # Both, not either: aqua is under 3:1 on this surface, so colour
    # alone cannot be what tells the traces apart. Label text is ink; the
    # colour rides a dot next to it.
    if len(drawn) >= 2:
        # Right-aligned on the title row. Laid out right-to-left so it
        # can never grow leftwards into the title, which is what
        # happened when it started at the left margin.
        rx = w - pad_r
        for sr in reversed(drawn):
            tw = 6 * len(sr["name"])
            canvas.create_text(rx, 15, anchor="e", text=sr["name"],
                               fill=INK_SECONDARY, font=("Helvetica", 8))
            canvas.create_oval(rx - tw - 11, 11, rx - tw - 4, 18,
                               fill=sr["color"], outline="")
            rx -= tw + 11 + 10

        # Direct labels only where there is more than one trace to tell
        # apart. With a single series the title already names it, and an
        # end label just gets clipped against the right margin.
        #
        # Converging traces are the hard case: three gyro axes all
        # settling toward zero put X, Y and Z on the same pixel and the
        # labels become an unreadable blob. So the DOTS stay exactly on
        # the data and only the labels are nudged apart, which keeps the
        # plot honest while making it legible.
        ends = []
        for sr in drawn:
            ex, ey = to_px(*sr["points"][-1])
            ends.append([min(ex, w - pad_r), ey, ey, sr])
        ends.sort(key=lambda e: e[2])
        MIN_GAP = 10
        for i in range(1, len(ends)):
            if ends[i][2] - ends[i - 1][2] < MIN_GAP:
                ends[i][2] = ends[i - 1][2] + MIN_GAP
        # Keep the nudged stack inside the plot.
        overflow = ends[-1][2] - (h - pad_b)
        if overflow > 0:
            for e in ends:
                e[2] -= overflow

        for ex, ey, ly, sr in ends:
            canvas.create_oval(ex - 3, ey - 3, ex + 3, ey + 3,
                               fill=sr["color"], outline=PLOT_SURFACE, width=1)
            # Flip to the inside of the plot when the point is close
            # enough to the edge that the label would run off it.
            if ex + 6 + 6 * len(sr["name"]) > w - 2:
                canvas.create_text(ex - 6, ly, anchor="e", text=sr["name"],
                                   fill=INK_SECONDARY, font=("Helvetica", 8, "bold"))
            else:
                canvas.create_text(ex + 6, ly, anchor="w", text=sr["name"],
                                   fill=INK_SECONDARY, font=("Helvetica", 8, "bold"))
    else:
        # One series: just mark where it currently is.
        sr = drawn[0]
        ex, ey = to_px(*sr["points"][-1])
        ex = min(ex, w - pad_r)
        canvas.create_oval(ex - 3, ey - 3, ex + 3, ey + 3,
                           fill=sr["color"], outline=PLOT_SURFACE, width=1)

    # --- point markers (a place on a track, not a time) ------------------
    # Several events can share one GPS fix -- everything before the
    # vehicle moves is at the pad. The DOTS stay on the data and the
    # labels stack upward, so a cluster reads as a list rather than as
    # one illegible smear.
    placed_pts = []
    for mx, my, label in point_markers:
        if not (x_lo <= mx <= x_hi and y_lo <= my <= y_hi):
            continue
        px, py = to_px(mx, my)
        canvas.create_oval(px - 4, py - 4, px + 4, py + 4,
                           outline=INK_SECONDARY, fill=PLOT_SURFACE, width=1)
        ly = py - 7
        while any(abs(px - qx) < 44 and abs(ly - qy) < 9 for qx, qy in placed_pts):
            ly -= 9
        placed_pts.append((px, ly))
        canvas.create_text(px, ly, anchor="s", text=label,
                           fill=INK_SECONDARY, font=("Helvetica", 7))

    # --- hover crosshair + readout -------------------------------------
    if hover is not None:
        hx, _hy = hover
        if x_lo <= hx <= x_hi:
            px, _ = to_px(hx, y_lo)
            canvas.create_line(px, pad_t, px, h - pad_b, fill="#b9b9b4", dash=(2, 2))
            parts = []
            for sr in drawn:
                # Nearest sample in x -- the traces are sampled, and
                # interpolating would invent a reading that never existed.
                best = min(sr["points"], key=lambda p: abs(p[0] - hx))
                parts.append(f"{sr['name']} {best[1]:.4g}")
            canvas.create_text(pad_l + 4, pad_t + 8, anchor="w",
                               text=f"{_fmt_tick(hx, x_span)}  " + "   ".join(parts),
                               fill=INK_PRIMARY, font=("Helvetica", 8))
    return True


def open_plots_window(parent, gs, mono):
    """Altitude, acceleration, gyro and ground track, 2x2, live."""
    import tkinter as tk
    from tkinter import ttk

    win = tk.Toplevel(parent)
    win.title("Plots")
    win.geometry("1000x720")

    bar = ttk.Frame(win, padding=(8, 6))
    bar.pack(fill="x")
    recovery = ttk.Label(bar, text="GPS: no fix yet", font=("Helvetica", 13, "bold"))
    recovery.pack(side="left")

    # Simulated data and live telemetry look identical on a plot. The
    # only thing stopping somebody reading one as the other is this, so
    # it is loud, it is above the plots rather than beside them, and it
    # carries the playback controls -- there is no way to drive a replay
    # without the banner in front of you.
    sim_wrap = tk.Frame(win)
    sim_bar = tk.Frame(sim_wrap, bg="#b00020")
    sim_bar.pack(fill="x")
    sim_lbl = tk.Label(sim_bar, text="", font=("Helvetica", 11, "bold"),
                       fg="#ffffff", bg="#b00020", pady=4, anchor="w",
                       justify="left")
    sim_lbl.pack(side="left", padx=8)

    def sim_finish():
        gs.finish_simulation()

    def sim_pause():
        if gs.sim:
            gs.sim.paused = not gs.sim.paused

    def sim_restart():
        s = gs.sim
        if not s:
            return
        s.i = 0
        s.play_t = 0.0
        s.last_wall = None
        s.paused = False
        s.agl.clear()
        s.imu.clear()
        s.events.clear()

    fin_btn = tk.Button(sim_bar, text="Finish", command=sim_finish,
                        font=("Helvetica", 10, "bold"))
    fin_btn.pack(side="right", padx=(4, 8), pady=3)
    pause_btn = tk.Button(sim_bar, text="Pause", command=sim_pause, width=7)
    pause_btn.pack(side="right", padx=4, pady=3)
    tk.Button(sim_bar, text="Replay", command=sim_restart).pack(side="right",
                                                                padx=4, pady=3)
    sim_speed = tk.StringVar(value="1x")

    def on_speed(_e=None):
        if gs.sim:
            gs.sim.speed = float(sim_speed.get().rstrip("x"))
    speed_box = ttk.Combobox(sim_bar, textvariable=sim_speed, width=4,
                             state="readonly",
                             values=("0.5x", "1x", "2x", "5x", "10x", "25x"))
    speed_box.pack(side="right", padx=4, pady=3)
    speed_box.bind("<<ComboboxSelected>>", on_speed)
    tk.Label(sim_bar, text="speed", fg="#ffffff", bg="#b00020").pack(
        side="right", padx=(8, 0))

    # A second, differently coloured line -- not more red text on red --
    # because "you are looking at a file" and "the vehicle is talking
    # right now" are two separate things to know.
    sim_live = tk.Label(sim_wrap, text="", font=("Helvetica", 10, "bold"),
                        fg="#3a2a00", bg="#ffd24d", pady=3, anchor="w",
                        justify="left")

    def wrap_live(event):
        # The sentence has to survive a narrow window: a warning whose
        # second half is off the right-hand edge is not a warning. Only
        # set it when it actually changes, or configuring inside a
        # <Configure> handler chases its own tail.
        want = max(200, event.width - 16)
        if int(sim_live.cget("wraplength") or 0) != want:
            sim_live.configure(wraplength=want)
    sim_live.bind("<Configure>", wrap_live)
    ttk.Button(bar, text="Re-zero pad",
               command=gs.reset_pad_fix).pack(side="right")
    ttk.Button(bar, text="Clear plots",
               command=gs.clear_plots).pack(side="right", padx=(0, 6))
    window_var = tk.StringVar(value="120")
    ttk.Label(bar, text="window (s)").pack(side="right", padx=(0, 4))
    ttk.Entry(bar, textvariable=window_var, width=6).pack(side="right", padx=4)

    grid = ttk.Frame(win, padding=4)
    grid.pack(fill="both", expand=True)
    for i in range(2):
        grid.rowconfigure(i, weight=1)
        grid.columnconfigure(i, weight=1)

    canvases = {}
    hover = {}
    for key, (r, c) in (("alt", (0, 0)), ("accel", (0, 1)),
                        ("gyro", (1, 0)), ("track", (1, 1))):
        cv = tk.Canvas(grid, background=PLOT_SURFACE, highlightthickness=1,
                       highlightbackground="#d8d8d4")
        cv.grid(row=r, column=c, sticky="nsew", padx=3, pady=3)
        canvases[key] = cv
        hover[key] = None

        def on_move(event, k=key):
            hover[k] = ("px", event.x)

        def on_leave(_event, k=key):
            hover[k] = None
        cv.bind("<Motion>", on_move)
        cv.bind("<Leave>", on_leave)

    def window_s():
        try:
            v = float(window_var.get())
            return v if v > 0 else 120.0
        except ValueError:
            return 120.0

    def tail(history, idx_pairs):
        """Recent samples as {name: [(t, v)]}, trimmed to the time window."""
        if not history:
            return {}
        t_end = history[-1][0]
        t_start = t_end - window_s()
        out = {name: [] for name, _ in idx_pairs}
        for row in history:
            if row[0] < t_start:
                continue
            for name, i in idx_pairs:
                out[name].append((row[0], row[i]))
        return out

    def hover_x(key, canvas, x_lo, x_hi):
        """Turn a pixel x into a data x, matching draw_xy_plot's geometry."""
        h = hover.get(key)
        if h is None:
            return None
        pad_l, pad_r = 58, 14
        w = canvas.winfo_width()
        span = w - pad_l - pad_r
        if span <= 0:
            return None
        frac = (h[1] - pad_l) / span
        if not 0.0 <= frac <= 1.0:
            return None
        return (x_lo + frac * (x_hi - x_lo), 0.0)

    def redraw():
        if not win.winfo_exists():
            return

        marks = list(gs.plot_events())

        # --- altitude ---
        agl = [(t, v) for t, v in gs.plot_agl()]
        if agl:
            t_end = agl[-1][0]
            agl = [p for p in agl if p[0] >= t_end - window_s()]
        alt_series = [{"name": "AGL", "color": SERIES_COLORS[0], "points": agl}]
        hv = hover_x("alt", canvases["alt"], agl[0][0], agl[-1][0]) if len(agl) >= 2 else None
        draw_xy_plot(canvases["alt"], "Altitude above pad", alt_series,
                     "vehicle time (s)", "m", hover=hv, markers=marks,
                     empty_text="waiting for altitude")

        # --- accel / gyro ---
        imu = gs.plot_imu()
        acc = tail(imu, [(AXIS_NAMES[i], 1 + i) for i in range(3)])
        gyr = tail(imu, [(AXIS_NAMES[i], 4 + i) for i in range(3)])
        for key, data, title, unit in (
                ("accel", acc, "Acceleration", "g"),
                ("gyro", gyr, "Angular rate", "deg/s")):
            series = [{"name": AXIS_NAMES[i], "color": SERIES_COLORS[i],
                       "points": data.get(AXIS_NAMES[i], [])} for i in range(3)]
            pts = series[0]["points"]
            hv = (hover_x(key, canvases[key], pts[0][0], pts[-1][0])
                  if len(pts) >= 2 else None)
            draw_xy_plot(canvases[key], title, series,
                         "vehicle time (s)", unit, hover=hv, markers=marks,
                         empty_text="waiting for IMU")

        # --- ground track ---
        gps = gs.plot_track()
        track = [(e, n) for _t, e, n in gps]
        tr_series = [{"name": "track", "color": SERIES_COLORS[1], "points": track}]
        # An event is a POSITION on a ground track, not a line across it,
        # so each one is placed at the fix nearest it in time. GPS is
        # 1 Hz, so a marker can be up to half a second off along the
        # track -- fine for "roughly where the chute opened", not a
        # number to quote.
        track_marks = []
        if gps:
            for mt, label, _group in marks:
                near = min(gps, key=lambda r: abs(r[0] - mt))
                if abs(near[0] - mt) <= 2.0:
                    track_marks.append((near[1], near[2], label))

        draw_xy_plot(canvases["track"], "Ground track from pad",
                     tr_series, "east (m)  [-ve = west]", "north (m)",
                     equal_aspect=True, compass=True, origin=(0.0, 0.0, "pad"),
                     point_markers=track_marks,
                     empty_text=("a flight-data CSV carries no GPS"
                                 if gs.sim else "waiting for a GPS fix"))

        # --- am I looking at a real flight? ---
        s = gs.sim
        if s:
            shown, total, frac = s.progress()
            if s.done:
                txt = (f"SIMULATION \u2014 {s.name} \u2014 playback complete "
                       f"({total:.1f} s), plots frozen \u2014 press Finish for "
                       f"live telemetry")
            else:
                txt = (f"SIMULATION \u2014 {s.name} \u2014 {shown:.1f} / "
                       f"{total:.1f} s  ({frac * 100:.0f}%)"
                       f"{'  [PAUSED]' if s.paused else ''} \u2014 "
                       f"not live telemetry")
            sim_lbl.configure(text=txt)
            pause_btn.configure(text="Resume" if s.paused else "Pause")
            # The box follows the player, not the other way round: the
            # speed can be set when the replay is started, and a control
            # showing 1x while the flight runs at 10x is a lie about the
            # one number this window exists to make legible.
            want_speed = f"{s.speed:g}x"
            if sim_speed.get() != want_speed:
                sim_speed.set(want_speed)
            fin_btn.configure(text="Finish" if s.done else "Finish early")
            if s.live_packets:
                sim_live.configure(
                    text=f"\u26a0  LIVE TELEMETRY IS ARRIVING \u2014 "
                         f"{s.live_packets} packets, {s.live_records} records "
                         f"decoded and written to this session's files while the "
                         f"replay plays. Press Finish to put the vehicle back on "
                         f"these plots.")
                if not sim_live.winfo_ismapped():
                    sim_live.pack(fill="x")
            elif sim_live.winfo_ismapped():
                sim_live.pack_forget()
            if not sim_wrap.winfo_ismapped():
                sim_wrap.pack(fill="x", before=grid)
        elif sim_wrap.winfo_ismapped():
            sim_wrap.pack_forget()

        # --- the number you walk out with ---
        rb = gs.gps_from_pad()
        if rb is None:
            recovery.configure(text="GPS: no fix yet", foreground=INK_MUTED)
        else:
            dist, bearing = rb
            # Always the LIVE fix, even mid-replay. This is the number
            # somebody walks into the desert with; a simulation is not a
            # reason to stop showing it, so it says whose it is instead.
            recovery.configure(
                text=(("LIVE GPS: " if gs.sim else "")
                      + f"{dist:.0f} m / {dist * 3.28084:.0f} ft"
                        f"  at  {bearing:03.0f}° from pad"),
                foreground=INK_PRIMARY)

        win.title(f"Plots \u2014 SIMULATION: {gs.sim_name}" if gs.sim_name
                  else "Plots")
        win.after(200, redraw)

    redraw()
    return win


# ==========================================================================
# Headless
# ==========================================================================

class TerminalCollapser:
    """Consecutive-duplicate suppression for the terminal modes.

    A terminal cannot go back and edit a line it already printed, so the
    GUI's approach (rewrite the line in place with a count) is not
    available. The honest equivalent is what syslog does: hold the
    repeats back, then state how many there were once the flood ends."""

    def __init__(self):
        self.key = None
        self.repeats = 0

    def _flush(self):
        if self.repeats:
            out = f"        ... last message repeated {self.repeats} more time" \
                  f"{'' if self.repeats == 1 else 's'}"
            self.repeats = 0
            return [out]
        return []

    def __call__(self, msgs):
        out = []
        for ts, level, text, key in msgs:
            if key == self.key:
                self.repeats += 1
                continue
            out += self._flush()
            self.key = key
            out.append(f"[{time.strftime('%H:%M:%S', time.localtime(ts))}] {text}")
        return out

    def finish(self):
        return self._flush()


def run_headless(gs, interval=2.0):
    collapser = TerminalCollapser()
    next_summary = 0.0
    try:
        while True:
            gs.poll()
            for line in collapser(gs.drain_messages()):
                print(line, flush=True)
            now = time.time()
            if now >= next_summary:
                next_summary = now + interval
                print(gs.summary_line(), flush=True)
                gs.flush_files()
            if not gs.connected and gs.inbox.empty():
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    for line in collapser.finish():
        print(line, flush=True)
    gs.disconnect()


def run_replay(gs, path):
    with open(path, "rb") as f:
        data = f.read()
    gs.capture = False
    for i in range(0, len(data), 4096):
        gs.feed(data[i:i + 4096])
    for line in TerminalCollapser()(gs.drain_messages()):
        print(line)
    print(gs.summary_line())
    s = gs.stats
    print(f"records {s['records']}  junk bytes {s['junk_bytes']}  bad frames {s['bad_frames']}  "
          f"CRC fail {s['pkt_crc']}  malformed {s['pkt_malformed']}  size mismatch {s['pkt_size']}")
    gs.close_files()
    if gs.out_dir:
        print(f"CSVs in {gs.out_dir}")


# ==========================================================================
# GUI
# ==========================================================================

def run_gui(gs, port=None, baud=115200, autoclose_ms=None, on_ready=None):
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title("Osiris ground station")
    root.geometry("1100x720")
    mono = ("Consolas", 10) if sys.platform == "win32" else ("DejaVu Sans Mono", 10)

    # -- top bar --------------------------------------------------------
    top = ttk.Frame(root, padding=6)
    top.pack(fill="x")
    ttk.Label(top, text="Port").pack(side="left")
    port_var = tk.StringVar(value=port or "socket://127.0.0.1:9750")
    ttk.Entry(top, textvariable=port_var, width=32).pack(side="left", padx=4)
    conn_btn = ttk.Button(top, text="Connect")
    conn_btn.pack(side="left")
    conn_lbl = ttk.Label(top, text="not connected")
    conn_lbl.pack(side="left", padx=10)
    dest_txt = "broadcast" if gs.dest == BROADCAST else f"{gs.dest:016X}"
    ttk.Label(top, text=f"uplink dest: {dest_txt}").pack(side="right")

    def toggle():
        if gs.connected:
            gs.disconnect()
        else:
            try:
                gs.connect(port_var.get().strip(), baud)
            except Exception as e:
                messagebox.showerror("Connect failed", str(e))
    conn_btn.configure(command=toggle)

    body = ttk.Frame(root, padding=(6, 0))
    body.pack(fill="both", expand=True)
    body.columnconfigure(1, weight=1)
    body.rowconfigure(0, weight=1)

    def panel(parent, title, rows):
        lf = ttk.LabelFrame(parent, text=title, padding=6)
        lf.pack(fill="x", padx=4, pady=4)
        vals = {}
        for i, r in enumerate(rows):
            ttk.Label(lf, text=r).grid(row=i, column=0, sticky="w")
            v = ttk.Label(lf, text="--", font=mono, width=22, anchor="w")
            v.grid(row=i, column=1, sticky="w", padx=(8, 0))
            vals[r] = v
        return lf, vals

    left = ttk.Frame(body)
    left.grid(row=0, column=0, sticky="ns")
    mode_lbl = tk.Label(left, text="--", font=("Helvetica", 22, "bold"), width=10)
    mode_lbl.pack(fill="x", pady=(4, 2))
    _, veh = panel(left, "Vehicle", [
        "flags", "uptime", "battery", "AGL", "baro alt (MSL)", "GPS", "sats / fix",
        "accel |a|", "from pad", "SD card", "SD drops", "telem drops"])

    plot = tk.Canvas(body, background="white", highlightthickness=1,
                     highlightbackground="#bbb")
    plot.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

    right = ttk.Frame(body)
    right.grid(row=0, column=2, sticky="ns")
    _, link = panel(right, "Link", ["last packet", "packets", "lost", "bad CRC",
                                    "decode errors", "duplicates", "retransmits",
                                    "RSSI downlink", "RSSI uplink"])

    # Signal bars sit in the same cell row as each RSSI readout, in a
    # third column, so the dBm text stays where it always was.
    try:
        _bar_bg = ttk.Style().lookup("TLabelframe", "background") or root.cget("background")
    except tk.TclError:
        _bar_bg = root.cget("background")

    def make_bars(lbl):
        """A bar meter canvas beside a panel label; returns its draw function."""
        cv = tk.Canvas(lbl.master, width=34, height=18, highlightthickness=0,
                       background=_bar_bg)
        cv.grid(row=int(lbl.grid_info()["row"]), column=2, sticky="e", padx=(4, 0))

        def draw(n, stale):
            """Four rising bars, the first n filled. Grey when the link is stale."""
            cv.delete("all")
            bw, h = 6, 18
            fill = SIGNAL_BAR_STALE if stale else SIGNAL_BAR_COLORS[n]
            for i in range(4):
                x0 = 1 + i * (bw + 2)
                top = h - 1 - (i + 1) * (h - 2) // 4
                cv.create_rectangle(x0, top, x0 + bw, h - 1, width=0,
                                    fill=fill if i < n else SIGNAL_BAR_EMPTY)
            if n == 0 and not stale:
                # Phone-style "no service" cross over the empty meter.
                cv.create_line(23, 2, 32, 11, fill=SIGNAL_BAR_COLORS[0], width=2)
                cv.create_line(32, 2, 23, 11, fill=SIGNAL_BAR_COLORS[0], width=2)
        return draw

    draw_dn_bars = make_bars(link["RSSI downlink"])
    draw_up_bars = make_bars(link["RSSI uplink"])

    cmds = ttk.LabelFrame(right, text="Commands", padding=6)
    cmds.pack(fill="x", pady=(8, 0))
    mark_var = tk.StringVar(value="1")
    buttons = {}

    def send(name):
        try:
            arg = int(mark_var.get(), 0) if name == "mark" else 0
        except ValueError:
            messagebox.showerror("MARK", "argument must be an integer")
            return
        if name == "arm" and not messagebox.askyesno(
                "ARM", "Send ARM? Sensors, SD and downlink go to flight rates."):
            return
        if name == "disarm" and not messagebox.askyesno("DISARM", "Send DISARM?"):
            return
        if name == "sd_eject" and not messagebox.askyesno(
                "EJECT SD",
                "Unmount the SD card?\n\n"
                "The vehicle will flush and close every log file and unmount "
                "the card. This is ONE WAY: it will not log again until it is "
                "power-cycled.\n\n"
                "Wait for SD_OK to clear in the flags before pulling the card.",
                icon="warning", default="no"):
            return
        try:
            gs.send_command(name, arg)
        except Exception as e:
            messagebox.showerror("Send failed", str(e))

    # No NOP button: PING proves the link just as well and also gets a
    # status record back. NOP is still in the uplink protocol
    # (uplink_command.py) -- only the button is gone.
    for i, name in enumerate(["arm", "disarm"]):
        b = ttk.Button(cmds, text=name.upper(), command=lambda n=name: send(n))
        b.grid(row=0, column=i, sticky="ew", padx=2, pady=2)
        buttons[name] = b
    buttons["ping"] = ttk.Button(cmds, text="PING", command=lambda: send("ping"))
    buttons["ping"].grid(row=1, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
    ttk.Entry(cmds, textvariable=mark_var, width=8).grid(row=2, column=0, padx=2, pady=2)
    buttons["mark"] = ttk.Button(cmds, text="MARK", command=lambda: send("mark"))
    buttons["mark"].grid(row=2, column=1, sticky="ew", padx=2, pady=2)

    # Full width and on its own row, away from the flight controls above it.
    # An eject cannot be undone from here, so it should not sit in the grid
    # where a mis-aimed click lands on it.
    ttk.Separator(cmds, orient="horizontal").grid(
        row=3, column=0, columnspan=2, sticky="ew", pady=(8, 4))
    buttons["sd_eject"] = ttk.Button(cmds, text="EJECT SD",
                                     command=lambda: send("sd_eject"))
    buttons["sd_eject"].grid(row=4, column=0, columnspan=2, sticky="ew",
                             padx=2, pady=2)
    ttk.Label(cmds, text="Confirmation comes from the vehicle's\n"
              "UPLINK_COMMAND event in the log. For EJECT SD,\n"
              "wait for the SD_OK flag to go dark.",
              foreground="#666").grid(row=5, column=0, columnspan=2, pady=(6, 0))

    cfgf = ttk.LabelFrame(right, text="Deployment config", padding=6)
    cfgf.pack(fill="x", pady=(8, 0))
    cfg_win = [None]

    def open_params():
        # One window, reused. Two editors open on the same config is a
        # good way to send the one you were not looking at.
        if cfg_win[0] is not None and cfg_win[0].winfo_exists():
            cfg_win[0].lift()
            cfg_win[0].focus_force()
            return
        cfg_win[0] = open_config_panel(root, gs, mono, send, on_simulated=open_plots)

    plots_win = [None]

    def open_plots():
        if plots_win[0] is not None and plots_win[0].winfo_exists():
            plots_win[0].lift()
            plots_win[0].focus_force()
            return
        plots_win[0] = open_plots_window(root, gs, mono)

    ttk.Button(cfgf, text="Flight parameters...", command=open_params).pack(fill="x")
    ttk.Button(cfgf, text="Plots (accel / gyro / GPS)...",
               command=open_plots).pack(fill="x", pady=(4, 0))
    cfg_lbl = ttk.Label(cfgf, text="--", font=mono, foreground="#666")
    cfg_lbl.pack(fill="x", pady=(4, 0))

    # A replay can outlive the window it was started from -- close the
    # plots and the main altitude strip is still showing a file. There
    # has to be a way back to live telemetry from here, or the only
    # remedy is restarting the program.
    sim_row = tk.Frame(cfgf, bg="#b00020")
    sim_row_lbl = tk.Label(sim_row, text="", font=mono, fg="#ffffff",
                           bg="#b00020", anchor="w")
    sim_row_lbl.pack(side="left", padx=6, pady=2)
    tk.Button(sim_row, text="Finish",
              command=lambda: gs.finish_simulation()).pack(side="right",
                                                           padx=4, pady=2)

    logf = ttk.Frame(root, padding=6)
    logf.pack(fill="both")

    # The log follows the newest line ONLY while it is already showing the
    # newest line. Scroll up and it holds still; messages keep arriving and
    # land below the viewport instead of dragging it down.
    #
    # This label is always packed, never hidden, and only its text changes.
    # A label that appeared and disappeared would resize the log under the
    # pointer at the exact moment somebody is reading it, which is the
    # thing being fixed.
    follow_lbl = tk.Label(logf, text="", font=mono, anchor="w",
                          foreground="#a05a00")
    follow_lbl.pack(fill="x")

    log = tk.Text(logf, height=12, font=mono, state="disabled", wrap="none")
    sb = ttk.Scrollbar(logf, command=log.yview)
    log.configure(yscrollcommand=sb.set)
    sb.pack(side="right", fill="y")
    log.pack(fill="both", expand=True)
    for tag, color in (("error", "#b00020"), ("warn", "#a05a00"), ("event", "#0b5394"),
                       ("cmd", "#38761d"), ("info", "#333")):
        log.tag_configure(tag, foreground=color)

    MODE_COLORS = {"STANDBY": "#666", "ARMED": "#c77c00", "FLIGHT": "#b00020",
                   "LANDED": "#38761d"}

    def draw_plot():
        plot.delete("all")
        w, h = plot.winfo_width(), plot.winfo_height()
        pad_l, pad_r, pad_t, pad_b = 60, 12, 24, 28
        plot.create_text(pad_l, 12, anchor="w",
                         text=(f"SIMULATION: {gs.sim_name} \u2014 altitude above "
                               f"pad (m) vs flight time (s)" if gs.sim_name else
                               "Altitude AGL (m) vs vehicle time (s)"),
                         fill="#b00020" if gs.sim_name else INK_PRIMARY)
        pts = list(gs.plot_agl())
        if len(pts) < 2:
            plot.create_text(w / 2, h / 2, text="waiting for altitude", fill="#999")
            return
        t_end = pts[-1][0]
        pts = [p for p in pts if p[0] >= t_end - 120]
        t0 = pts[0][0]
        ys = [p[1] for p in pts]
        y_lo, y_hi = min(ys + [0.0]), max(ys + [1.0])
        span_t = max(t_end - t0, 1e-3)
        span_y = y_hi - y_lo

        def xy(t, y):
            return (pad_l + (t - t0) / span_t * (w - pad_l - pad_r),
                    h - pad_b - (y - y_lo) / span_y * (h - pad_t - pad_b))

        for frac in (0, 0.5, 1):
            y = y_lo + frac * span_y
            _, py = xy(t0, y)
            plot.create_line(pad_l, py, w - pad_r, py, fill="#eee")
            plot.create_text(pad_l - 6, py, anchor="e", text=f"{y:.0f}")
        plot.create_text(pad_l, h - 10, anchor="w", text=f"{t0:.0f}")
        plot.create_text(w - pad_r, h - 10, anchor="e", text=f"{t_end:.0f}")
        coords = [c for p in pts for c in xy(*p)]
        plot.create_line(*coords, fill="#0b5394", width=2)
        plot.create_text(w - pad_r, 12, anchor="e",
                         text=f"{ys[-1]:.1f} m  ({ys[-1] * 3.28084:.0f} ft)",
                         font=("Helvetica", 11, "bold"))

    def fnum(rec, key, fmt):
        return "--" if not rec or key not in rec else format(rec[key], fmt)

    def refresh():
        st, baro = gs.latest.get("status"), gs.latest.get("baro")
        gps, imu = gs.latest.get("gps"), gs.latest.get("imu")
        mode_lbl.configure(text=gs.mode or "--", fg=MODE_COLORS.get(gs.mode, "#333"))
        veh["flags"].configure(text=gs.flag_text())
        veh["uptime"].configure(text=fnum(st, "uptime_s", "d") + " s" if st else "--")
        # batt stream, not status: status no longer carries battery_v.
        batt = gs.latest.get("batt")
        veh["battery"].configure(text=fnum(batt, "battery_v", ".2f") + " V" if batt else "--")
        veh["AGL"].configure(text=(f"{st['altitude_agl_m']:.1f} m / "
                                   f"{st['altitude_agl_m'] * 3.28084:.0f} ft") if st else "--")
        veh["baro alt (MSL)"].configure(text=fnum(baro, "altitude_m", ".1f") + " m" if baro else "--")
        veh["GPS"].configure(text=f"{gps['latitude']:.6f}, {gps['longitude']:.6f}" if gps else "--")
        veh["sats / fix"].configure(text=f"{gps['satellites']} / {gps['fix_quality']}" if gps else "--")
        if imu:
            a = (imu["ax"] ** 2 + imu["ay"] ** 2 + imu["az"] ** 2) ** 0.5
            veh["accel |a|"].configure(text=f"{a:.2f}")
        rb = gs.gps_from_pad()
        veh["from pad"].configure(
            text="--" if rb is None
            else f"{rb[0] * 3.28084:.0f} ft at {rb[1]:03.0f}\u00b0")

        # The one field an operator stares at after pressing EJECT SD, so it
        # says the state in words rather than leaving them to read a bit out
        # of the flags line.
        if gs.sd_mounted is None:
            sd_text, sd_fg = "--", "black"
        elif not gs.sd_mounted:
            sd_text, sd_fg = "UNMOUNTED", "#b00020"
        else:
            left = gs.sd_auto_eject_in()
            if left is None:
                sd_text, sd_fg = "mounted", "black"
            elif left < 0:
                # Past the deadline and still mounted: the vehicle is
                # ARMED or flying and is holding the unmount back.
                sd_text, sd_fg = "mounted (auto-eject deferred)", "#a05a00"
            else:
                sd_text = f"mounted (auto {left // 3600:d}:{left % 3600 // 60:02d})"
                sd_fg = "#a05a00" if left < 600 else "black"
        veh["SD card"].configure(text=sd_text, foreground=sd_fg)
        veh["SD drops"].configure(text=fnum(st, "sd_drops", "d"))
        veh["telem drops"].configure(text=fnum(st, "telem_drops", "d"))

        age = gs.link_age()
        link["last packet"].configure(
            text="--" if age is None else f"{age:.1f} s ago",
            foreground="#b00020" if age is not None and age > 6 else "black")
        s = gs.stats
        link["packets"].configure(text=str(s["packets"]))
        link["lost"].configure(text=str(gs.seq.lost))
        link["bad CRC"].configure(text=str(s["pkt_crc"]))
        link["decode errors"].configure(text=str(s["pkt_malformed"] + s["pkt_size"]),
                                        foreground="#b00020" if s["pkt_malformed"] + s["pkt_size"] else "black")
        link["duplicates"].configure(text=str(gs.seq.duplicates))
        link["retransmits"].configure(text=str(s["retransmits"]))
        link["RSSI downlink"].configure(text=fmt_rssi(gs.ground_rssi))
        link["RSSI uplink"].configure(text=fmt_rssi(gs.vehicle_rssi))
        # Both readings describe the last packet each end heard, so they go
        # grey together when the downlink stops arriving.
        _stale = age is None or age > 6
        draw_dn_bars(rssi_bars(gs.ground_rssi), stale=_stale)
        draw_up_bars(rssi_bars(gs.vehicle_rssi), stale=_stale)

        prog = gs.upload_progress()
        if prog:
            cfg_lbl.configure(text=f"uploading {prog[0]}/{prog[1]}", foreground="#0b5394")
        elif gs.config_status:
            kind, detail = gs.config_status
            cfg_lbl.configure(
                text={"stored": f"stored ({detail} B)", "applied": "applied"}.get(
                    kind, "REJECTED"),
                foreground="#b00020" if kind == "rejected" else "#38761d")
        else:
            cfg_lbl.configure(text="--", foreground="#666")

        if gs.sim:
            shown, total, _frac = gs.sim.progress()
            sim_row_lbl.configure(
                text=(f"SIM {gs.sim.name}  "
                      + ("frozen at end" if gs.sim.done
                         else f"{shown:.0f}/{total:.0f}s")
                      + ("  +LIVE" if gs.sim.live_packets else "")))
            if not sim_row.winfo_ismapped():
                sim_row.pack(fill="x", pady=(4, 0))
        elif sim_row.winfo_ismapped():
            sim_row.pack_forget()

        allowed = gs.allowed_commands()
        for name, b in buttons.items():
            b.state(["!disabled"] if name in allowed else ["disabled"])
        conn_btn.configure(text="Disconnect" if gs.connected else "Connect")
        conn_lbl.configure(text=f"connected: {gs.port_name}" if gs.connected else "not connected")
        draw_plot()

    tick_n = [0]
    pending = [0]          # lines that arrived while scrolled back
    pending_bad = [False]  # ...and whether any of them was an error
    shown = [None]         # what the label currently says, to avoid redrawing it

    def log_at_bottom():
        """True when the last line is visible.

        yview() gives (first, last) as fractions of the whole document.
        The comparison is against 0.9995 rather than 1.0 because a
        partially visible last line leaves it a hair under."""
        return log.yview()[1] > 0.9995

    def update_follow_label():
        n = pending[0]
        if not n:
            follow_lbl.configure(text="", foreground="#a05a00")
            return
        follow_lbl.configure(
            text=f"  paused: {n} new line{'' if n == 1 else 's'} below"
                 f"{' -- INCLUDING ERRORS' if pending_bad[0] else ''}."
                 f"  Click here to jump to the latest.",
            foreground="#b00020" if pending_bad[0] else "#a05a00")

    def jump_to_latest(_event=None):
        log.see("end")
        pending[0] = 0
        pending_bad[0] = False
        update_follow_label()
        shown[0] = 0

    follow_lbl.bind("<Button-1>", jump_to_latest)

    # ------------------------------------------------------------------
    #  Collapsing a repeating fault onto one line.
    #
    #  A sensor failing continuously emits one ERR per sample. Printed
    #  one per line that is not a log, it is a denial of service on the
    #  operator's attention -- the flood scrolls away the mode change or
    #  the event that explains it.
    #
    #  So a line whose key has been seen RECENTLY is rewritten in place
    #  with a count and the time it was last seen, instead of a new line
    #  being added. Recently means within COLLAPSE_WINDOW lines: an
    #  error that stopped an hour ago and started again is genuinely new
    #  information and gets its own line, rather than having its count
    #  silently ticked up somewhere far out of sight.
    #
    #  Rewriting is safe because this widget is only ever appended to.
    #  A stored line number therefore stays valid -- nothing is inserted
    #  above it to shift it -- and replacing a line's text changes no
    #  line count, so it cannot move the viewport of somebody who has
    #  scrolled back.
    # ------------------------------------------------------------------
    COLLAPSE_WINDOW = 40
    collapsed = {}      # key -> [line number, count, first ts, base text]

    def log_last_line():
        """Line number of the last REAL line, or 0 when empty.

        tk.Text always carries a trailing newline, so "end-1c" lands on
        the line AFTER the last one with text in it -- after two
        appended lines it reads 3.0, not 2.something. Subtracting one
        here means every caller below can just say "the last line" and
        be right, instead of each of them rediscovering this."""
        return int(log.index("end-1c").split(".")[0]) - 1

    def render_collapsed(entry, ts):
        ln, n, first, base = entry
        return (f"[{time.strftime('%H:%M:%S', time.localtime(first))}] {base}"
                f"   (x{n}, last {time.strftime('%H:%M:%S', time.localtime(ts))})")

    def tick():
        gs.poll()
        msgs = gs.drain_messages()
        if msgs:
            # Asked BEFORE the insert. Afterwards the tail has moved past
            # the viewport and the answer is always "no", which would make
            # the log stop following for everyone, permanently.
            following = log_at_bottom()
            log.configure(state="normal")
            added = 0
            added_bad = False
            for ts, level, text, key in msgs:
                entry = collapsed.get(key)
                if entry is not None and log_last_line() - entry[0] <= COLLAPSE_WINDOW:
                    entry[1] += 1
                    ln = entry[0]
                    log.delete(f"{ln}.0", f"{ln}.end")
                    log.insert(f"{ln}.0", render_collapsed(entry, ts), level)
                    continue
                log.insert("end",
                           f"[{time.strftime('%H:%M:%S', time.localtime(ts))}] {text}\n",
                           level)
                collapsed[key] = [log_last_line(), 1, ts, text]
                added += 1
                added_bad = added_bad or level == "error"
            log.configure(state="disabled")
            if following:
                log.see("end")
            elif added:
                # Only genuinely new lines push the viewport further away;
                # a rewritten line does not move anything.
                pending[0] += added
                pending_bad[0] = pending_bad[0] or added_bad
        # Scrolling back down by hand resumes following, so this is checked
        # every tick and not only when a message happens to arrive.
        if pending[0] and log_at_bottom():
            pending[0] = 0
            pending_bad[0] = False
        if shown[0] != pending[0]:
            shown[0] = pending[0]
            update_follow_label()
        tick_n[0] += 1
        if tick_n[0] % 5 == 0:
            refresh()
        if tick_n[0] % 40 == 0:
            gs.flush_files()
        root.after(50, tick)

    def on_close():
        gs.disconnect()
        gs.close_files()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    if port:
        root.after(100, toggle)
    root.after(50, tick)
    if on_ready:                         # test hook
        root.after(max(0, (autoclose_ms or 1000) - 300), lambda: on_ready(root))
    if autoclose_ms:                     # test hook
        root.after(autoclose_ms, on_close)
    root.mainloop()


# ==========================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(description="Osiris ground station",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--port", help="COM5, /dev/ttyUSB0, or socket://127.0.0.1:9750 for the emulator")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dest", type=lambda s: int(s, 16), default=BROADCAST,
                    help="vehicle radio's 64-bit address in hex for uplink (default broadcast)")
    ap.add_argument("--out", help="session directory (default sessions/<timestamp>)")
    ap.add_argument("--no-capture", action="store_true", help="do not write raw.bin")
    ap.add_argument("--headless", action="store_true", help="terminal only, no window")
    ap.add_argument("--replay", help="decode a raw.bin capture instead of a live port")
    args = ap.parse_args(argv)

    gs = GroundStation(out_dir=args.out, dest=args.dest, capture=not args.no_capture)
    if args.replay:
        run_replay(gs, args.replay)
        return 0
    if args.headless:
        if not args.port:
            ap.error("--headless needs --port")
        gs.connect(args.port, args.baud)
        run_headless(gs)
        gs.close_files()
        return 0
    run_gui(gs, args.port, args.baud)
    return 0


if __name__ == "__main__":
    sys.exit(main())
