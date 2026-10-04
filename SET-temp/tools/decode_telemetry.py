#!/usr/bin/env python3
"""
Decode an XBee API byte stream into per-stream CSVs -- from a file, or
live from the ground station's radio.

    python3 decode_telemetry.py capture.bin           # -> telem_baro.csv, ...
    python3 decode_telemetry.py capture.bin --stdout
    python3 decode_telemetry.py --port COM5           # live, Ctrl-C to stop
    python3 decode_telemetry.py --port /dev/ttyUSB0 --raw flight.bin

Live mode prints a one-line vehicle summary that refreshes in place, and
on Ctrl-C writes the same CSVs the file mode does. --raw also appends
every byte the radio produced to a file, so a session can be re-decoded
later without re-flying.

The input is whatever a serial port produced, raw. Both capture points
work: 0x90 Receive Packet frames from the ground station's radio, and
0x10 Transmit Request frames captured on the vehicle side -- the same
telemetry packet sits inside each, just at a different offset.

It does not need to start on a frame boundary and it does not need to be
clean -- the parser resyncs on 0x7E and drops anything whose checksum or
CRC fails, counting both.

Two independent integrity layers, deliberately:

  * the XBee's own 8-bit frame checksum, which catches UART-level damage
  * our CRC16-CCITT over the packet, which catches anything that got
    through the radio intact but corrupt

A packet failing CRC is reported, not silently skipped -- a run of them
means the link is marginal, which is worth knowing.
"""

import csv
import struct
import sys
import time
from collections import defaultdict

# Kept in step with sensor_records.h and telemetry.h by hand.
STREAMS = {
    0: ("baro",  "<IQfff",     ["seq", "timestamp_us", "pressure_pa",
                                "temperature_c", "altitude_m"]),
    1: ("imu",   "<IQffffff",  ["seq", "timestamp_us", "ax", "ay", "az",
                                "gx", "gy", "gz"]),
    2: ("gps",   "<IQddfBB",   ["seq", "timestamp_us", "latitude", "longitude",
                                "altitude_m", "fix_quality", "satellites"]),
    3: ("batt",  "<IQff",      ["seq", "timestamp_us", "battery_v", "adc_v"]),
    4: ("err",   "<IQBBhII",   ["seq", "timestamp_us", "source", "code",
                                "esp_err", "detail", "suppressed"]),
    5: ("event", "<IQBBHI",    ["seq", "timestamp_us", "type", "channel",
                                "flags", "detail"]),
    6: ("status", "<IQBBHfII", ["seq", "timestamp_us", "mode", "flags",
                                "uptime_s", "altitude_agl_m",
                                "sd_drops", "telem_drops"]),
}
RECORD_SIZE = {k: struct.calcsize(v[1]) for k, v in STREAMS.items()}

ERR_SOURCES = ["NONE", "BMP388", "LSM6DSOX", "GPS", "BATTERY", "SD",
               "TELEMETRY", "FLIGHT_LOGIC", "DISCRETE_IN", "ROUTER", "SYSTEM"]
ERR_CODES = ["NONE", "INIT_FAILED", "READ_FAILED", "WRITE_FAILED",
             "IRQ_TIMEOUT", "QUEUE_FULL", "PARSE_FAILED", "MOUNT_FAILED",
             "TASK_FAILED", "CONFIG_INVALID", "OVERFLOW", "CALIBRATION"]
EVENT_TYPES = ["NONE", "BOOT", "SD_MOUNTED", "STAGING_SIGNAL", "RECOVERY_SIGNAL",
               "LIFTOFF_HW", "LIFTOFF_SW", "APOGEE", "DEPLOY_ARMED", "DEPLOY_PREDICTED",
               "MODE_CHANGED", "LANDED", "UPLINK_COMMAND", "UPLINK_REJECTED",
               "BURN_START", "BURN_END", "SD_UNMOUNTED",
               "CONFIG_RX", "CONFIG_STORED", "CONFIG_REJECTED", "CONFIG_APPLIED"]
MODES = ["STANDBY", "ARMED", "FLIGHT", "LANDED"]
MODE_SRC = ["NONE", "UPLINK", "IMU_WAKEUP", "FLIGHT_LOGIC", "BOOT", "TEST"]
ENUM_COLUMNS = {4: {"source": ERR_SOURCES, "code": ERR_CODES},
                5: {"type": EVENT_TYPES},
                6: {"mode": MODES}}

PKT_MAGIC = 0xA5
PKT_VERSION = 1
PKT_HDR = 8

XBEE_RX_PACKET = 0x90     # ground-side capture: what the receiver got
XBEE_TX_REQUEST = 0x10    # vehicle-side capture: what the rocket sent
XBEE_TX_STATUS = 0x8B

# Where the telemetry packet starts inside each frame type.
PAYLOAD_OFFSET = {XBEE_RX_PACKET: 12, XBEE_TX_REQUEST: 14}


def _name(table, v):
    return table[v] if 0 <= v < len(table) else "?%d" % v


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class ApiFrameParser:
    """Incremental XBee API frame parser.

    Live capture and file capture share this one implementation on
    purpose: a second scanner written for the serial path would be a
    second thing to get wrong, and the two would drift.

    feed() takes any number of bytes and yields whole, checksum-valid
    frames. It resyncs on 0x7E and holds a partial frame across calls,
    which is what a serial read needs -- reads land mid-frame constantly.
    """

    def __init__(self):
        self.buf = bytearray()
        self.stats = {"frames": 0, "bad_checksum": 0, "resyncs": 0}

    def feed(self, data):
        self.buf.extend(data)
        b = self.buf
        while True:
            start = b.find(0x7E)
            if start < 0:
                self.stats["resyncs"] += len(b)
                del b[:]
                return
            if start:
                self.stats["resyncs"] += start
                del b[:start]
            if len(b) < 3:
                return                       # need the length field
            length = (b[1] << 8) | b[2]
            if length == 0 or length > 0xFFF:
                self.stats["resyncs"] += 1
                del b[0]
                continue
            if len(b) < 3 + length + 1:
                return                       # frame still arriving
            body = bytes(b[3:3 + length])
            chk = b[3 + length]
            if (0xFF - (sum(body) & 0xFF)) & 0xFF != chk:
                self.stats["bad_checksum"] += 1
                del b[0]                     # not a frame here; resync past it
                continue
            self.stats["frames"] += 1
            del b[:3 + length + 1]
            yield body[0], body


def iter_api_frames(blob):
    """Yield (frame_type, frame_bytes) for every checksum-valid frame."""
    p = ApiFrameParser()
    out = list(p.feed(blob))
    iter_api_frames.stats = p.stats
    return out


def parse_packet(pkt):
    """Returns (header_dict, [(stream, values)]) or None if it fails CRC."""
    if len(pkt) < PKT_HDR + 2 or pkt[0] != PKT_MAGIC:
        return None
    ver = pkt[1] >> 4
    flags = pkt[1] & 0x0F
    seq, rssi, count = struct.unpack_from("<HBB", pkt, 2)
    length = struct.unpack_from("<H", pkt, 6)[0]
    if PKT_HDR + length + 2 > len(pkt):
        return None
    want = struct.unpack_from("<H", pkt, PKT_HDR + length)[0]
    if crc16_ccitt(pkt[:PKT_HDR + length]) != want:
        return None

    hdr = {"version": ver, "retransmit": bool(flags & 0x01), "seq": seq,
           "rssi_dbm": -rssi if rssi else None, "count": count}
    out = []
    off = PKT_HDR
    end = PKT_HDR + length
    while off < end:
        sid = pkt[off]
        off += 1
        size = RECORD_SIZE.get(sid)
        if size is None or off + size > end:
            break                      # unknown stream: rest of block is unreadable
        vals = list(struct.unpack_from(STREAMS[sid][1], pkt, off))
        cols = STREAMS[sid][2]
        for col, table in ENUM_COLUMNS.get(sid, {}).items():
            vals[cols.index(col)] = _name(table, vals[cols.index(col)])
        out.append((sid, vals))
        off += size
    return hdr, out



# ===================================================================
#  Live mode
# ===================================================================

def _fmt_age(seconds):
    if seconds is None:
        return "--"
    return "%.1fs" % seconds


class LiveState:
    """Everything the one-line summary shows, updated as packets land."""

    def __init__(self):
        self.packets = 0
        self.bad_crc = 0
        self.retransmits = 0
        self.seqs = set()
        self.last_seq = None
        self.rssi = None
        self.last_packet_t = None
        self.mode = "?"
        self.alt = None
        self.batt = None
        self.sd_drops = None
        self.telem_drops = None
        self.flags = None
        self.events = []

    def note_packet(self, hdr, recs):
        self.packets += 1
        self.last_packet_t = time.time()
        self.seqs.add(hdr["seq"])
        self.last_seq = hdr["seq"]
        if hdr["rssi_dbm"] is not None:
            self.rssi = hdr["rssi_dbm"]
        if hdr["retransmit"]:
            self.retransmits += 1
        for sid, vals in recs:
            cols = STREAMS[sid][2]
            d = dict(zip(cols, vals))
            if sid == 6:                       # status
                self.mode = d.get("mode", "?")
                self.alt = d.get("altitude_agl_m")
                self.sd_drops = d.get("sd_drops")
                self.telem_drops = d.get("telem_drops")
                self.flags = d.get("flags")
            elif sid == 3:                     # batt -- the only source of battery_v
                self.batt = d.get("battery_v")
            elif sid == 5:                     # event -- the flight timeline
                self.events.append((d.get("timestamp_us"), d.get("type"),
                                    d.get("channel"), d.get("detail")))

    def missing(self):
        if not self.seqs:
            return 0
        lo, hi = min(self.seqs), max(self.seqs)
        return (hi - lo + 1) - len(self.seqs)

    def line(self):
        age = None
        if self.last_packet_t is not None:
            age = time.time() - self.last_packet_t
        stale = "" if age is None or age < 3.0 else "  *** NO DATA ***"
        return ("pkts %-6d lost %-4d crc %-3d | %-7s alt %-8s batt %-6s "
                "rssi %-7s drops sd/tx %s/%s | last %s%s" % (
                    self.packets, self.missing(), self.bad_crc, self.mode,
                    ("%.0fm" % self.alt) if self.alt is not None else "--",
                    ("%.2fV" % self.batt) if self.batt is not None else "--",
                    ("-%ddBm" % -self.rssi) if self.rssi is not None else "--",
                    self.sd_drops if self.sd_drops is not None else "-",
                    self.telem_drops if self.telem_drops is not None else "-",
                    _fmt_age(age), stale))


def run_live(port, baud, rows, raw_path=None, quiet=False):
    try:
        import serial                          # pyserial
    except ImportError:
        print("pyserial is not installed: pip install pyserial", file=sys.stderr)
        return 2

    parser = ApiFrameParser()
    st = LiveState()
    raw = open(raw_path, "ab") if raw_path else None

    print("listening on %s @ %d baud -- Ctrl-C to stop and write CSVs" % (port, baud))
    print("(the radio must be in API mode AP=1 on the same network ID and "
          "preamble as the vehicle)")
    last_draw = 0.0
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            while True:
                data = ser.read(4096) or ser.read(1)
                if data:
                    if raw:
                        raw.write(data)
                        raw.flush()
                    for ftype, frame in parser.feed(data):
                        off = PAYLOAD_OFFSET.get(ftype)
                        if off is None or len(frame) <= off:
                            continue
                        parsed = parse_packet(frame[off:])
                        if parsed is None:
                            st.bad_crc += 1
                            continue
                        hdr, recs = parsed
                        st.note_packet(hdr, recs)
                        for sid, vals in recs:
                            rows[sid].append([hdr["seq"], hdr["rssi_dbm"]] + vals)
                        if not quiet:
                            for sid, vals in recs:
                                if sid == 5:   # events scroll; they are the timeline
                                    cols = STREAMS[sid][2]
                                    d = dict(zip(cols, vals))
                                    sys.stdout.write("\r\033[K  EVENT %-16s ch=%s detail=%s\n"
                                                     % (d.get("type"), d.get("channel"),
                                                        d.get("detail")))
                now = time.time()
                if now - last_draw > 0.25:
                    sys.stdout.write("\r\033[K" + st.line())
                    sys.stdout.flush()
                    last_draw = now
    except KeyboardInterrupt:
        pass
    finally:
        if raw:
            raw.close()

    print("\n")
    print("api frames        : %d (bad checksum %d)" %
          (parser.stats["frames"], parser.stats["bad_checksum"]))
    print("telemetry packets : %d ok, %d failed CRC, %d retransmits" %
          (st.packets, st.bad_crc, st.retransmits))
    if st.seqs:
        print("sequence          : %d..%d, %d received, %d missing" %
              (min(st.seqs), max(st.seqs), len(st.seqs), st.missing()))
    if st.packets == 0:
        print()
        print("Nothing decoded. In order of likelihood:")
        print("  * the ground radio is not in API mode (AP=1)")
        print("  * network ID (ID) or preamble (HP) differ between the radios")
        print("  * baud mismatch -- the vehicle transmits at 115200")
        print("  * the vehicle is not powered, or its radio failed to verify")
        print("    (check its boot log for 'RADIO NOT VERIFIED')")
    if st.events:
        print()
        print("flight events:")
        for ts, typ, ch, det in st.events:
            print("  %-12s %-18s ch=%s detail=%s" % (ts, typ, ch, det))
    return 0


def write_outputs(rows, to_stdout):
    """One CSV per stream. Shared by the file and live paths so a live
    session ends with exactly the artefacts a file decode produces."""
    if not rows:
        print("no records decoded -- nothing to write")
        return
    for sid, data in sorted(rows.items()):
        name, _fmt, cols = STREAMS[sid]
        header = ["pkt_seq", "pkt_rssi_dbm"] + cols
        if to_stdout:
            print("\n== %s (%d records) ==" % (name, len(data)))
            print(",".join(header))
            for r in data:
                print(",".join("" if v is None else str(v) for v in r))
        else:
            out = "telem_%s.csv" % name
            with open(out, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(header)
                w.writerows(data)
            print("wrote %-18s %d records" % (out, len(data)))


def _opt(argv, name, default=None):
    """--name value, or --name=value."""
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(name + "="):
            return a.split("=", 1)[1]
    return default


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    to_stdout = "--stdout" in argv
    quiet = "--quiet" in argv
    port = _opt(argv, "--port")
    baud = int(_opt(argv, "--baud", "115200"))
    raw_path = _opt(argv, "--raw")

    rows = defaultdict(list)

    if port:
        rc = run_live(port, baud, rows, raw_path=raw_path, quiet=quiet)
        if rc == 0:
            write_outputs(rows, to_stdout)
        return rc

    # Values consumed by the options above are not input files.
    consumed = set()
    for name in ("--port", "--baud", "--raw"):
        for i, a in enumerate(argv):
            if a == name and i + 1 < len(argv):
                consumed.add(i + 1)
    paths = [a for i, a in enumerate(argv)
             if not a.startswith("--") and i not in consumed]
    if not paths:
        print("no input file given (and no --port)", file=sys.stderr)
        return 1

    packets = good = bad_crc = retransmits = 0
    seqs = []

    for path in paths:
        blob = open(path, "rb").read()
        for ftype, frame in iter_api_frames(blob):
            off = PAYLOAD_OFFSET.get(ftype)
            if off is None or len(frame) <= off:
                continue
            # 0x90: type, src64[8], reserved[2], options, data...
            # 0x10: type, frame_id, dest64[8], dest16[2], radius, options, data...
            parsed = parse_packet(frame[off:])
            packets += 1
            if parsed is None:
                bad_crc += 1
                continue
            good += 1
            hdr, recs = parsed
            seqs.append(hdr["seq"])
            if hdr["retransmit"]:
                retransmits += 1
            for sid, vals in recs:
                rows[sid].append([hdr["seq"], hdr["rssi_dbm"]] + vals)

    stats = getattr(iter_api_frames, "stats", {})
    print("api frames        : %d (bad checksum %d)" %
          (stats.get("frames", 0), stats.get("bad_checksum", 0)))
    print("telemetry packets : %d ok, %d failed CRC, %d retransmits" %
          (good, bad_crc, retransmits))

    if seqs:
        lo, hi = min(seqs), max(seqs)
        missing = sorted(set(range(lo, hi + 1)) - set(seqs))
        print("sequence          : %d..%d, %d received, %d missing" %
              (lo, hi, len(set(seqs)), len(missing)))
        if missing:
            head = ", ".join(str(m) for m in missing[:20])
            print("missing seqs      : %s%s" %
                  (head, " ..." if len(missing) > 20 else ""))

    write_outputs(rows, to_stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
