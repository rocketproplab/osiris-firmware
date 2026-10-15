#!/usr/bin/env python3
"""
decode_logs.py -- turn the flight computer's binary logs into CSV.

Usage:
    python3 decode_logs.py baro.bin                 # -> baro.csv
    python3 decode_logs.py baro.bin imu.bin gps.bin batt.bin err.bin event.bin status.bin
    python3 decode_logs.py /path/to/sdcard/*.bin
    python3 decode_logs.py baro.bin --stdout        # print instead of writing

The format is defined in sd_logger.h. Every file starts with a 16-byte
header, then a tight array of fixed-size records with no delimiters.
The header carries record_size, so this script can sanity-check the file
against what it expects instead of trusting the filename.

Struct layouts use '<' (little-endian, no alignment padding) to match
__attribute__((packed)) on the firmware side. If you ever remove packed
from those structs, these format strings stop matching and you'll get
garbage -- the size check below is what catches that.
"""

import argparse
import os
import struct
import sys

MAGIC = b"SETLOG"
HEADER_FORMAT = "<6sHBB6s"          # magic, version, stream_id, record_size, reserved
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_SIZE = 2                        # CRC16-CCITT appended after each record


def crc16_ccitt(data):
    """Must match crc16_ccitt() in sd_logger.c exactly."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

STREAM_BARO = 0
STREAM_IMU = 1
STREAM_GPS = 2
STREAM_BATT = 3
STREAM_ERR = 4
STREAM_EVENT = 5
STREAM_STATUS = 6

# Enum names, kept in step with sensor_records.h by hand. A value with no
# name here decodes as "?<n>" rather than silently reading as a plausible
# neighbour -- if you see those, this table is behind the firmware.
ERR_SOURCES = ["NONE", "BMP388", "LSM6DSOX", "GPS", "BATTERY", "SD",
               "TELEMETRY", "FLIGHT_LOGIC", "DISCRETE_IN", "ROUTER", "SYSTEM"]
ERR_CODES = ["NONE", "INIT_FAILED", "READ_FAILED", "WRITE_FAILED",
             "IRQ_TIMEOUT", "QUEUE_FULL", "PARSE_FAILED", "MOUNT_FAILED",
             "TASK_FAILED", "CONFIG_INVALID", "OVERFLOW", "CALIBRATION"]
EVENT_TYPES = ["NONE", "BOOT", "SD_MOUNTED", "STAGING_SIGNAL", "RECOVERY_SIGNAL",
               "LIFTOFF_HW", "LIFTOFF_SW", "APOGEE", "DEPLOY_ARMED", "DEPLOY_PREDICTED",
               "MODE_CHANGED", "LANDED", "UPLINK_COMMAND", "UPLINK_REJECTED",
               "BURN_START", "BURN_END", "SD_UNMOUNTED",
               "CONFIG_RX", "CONFIG_STORED", "CONFIG_REJECTED", "CONFIG_APPLIED",
               "SENSOR_LOST", "SENSOR_RECOVERED",
               "IMU_FAILOVER", "ALT_SOURCE", "VEL_SOURCE"]
MODES = ["STANDBY", "ARMED", "FLIGHT", "LANDED"]
MODE_SRC = ["NONE", "UPLINK", "IMU_WAKEUP", "FLIGHT_LOGIC", "BOOT", "TEST"]

def _name(table, v):
    return table[v] if 0 <= v < len(table) else "?%d" % v

# stream_id -> (name, struct format, column headers)
STREAMS = {
    STREAM_BARO: (
        "baro",
        "<IQfff",
        ["seq", "timestamp_us", "pressure_pa", "temperature_c", "altitude_m"],
    ),
    STREAM_IMU: (
        "imu",
        "<IQffffff",
        ["seq", "timestamp_us", "ax", "ay", "az", "gx", "gy", "gz"],
    ),
    STREAM_GPS: (
        "gps",
        "<IQddfBB",
        ["seq", "timestamp_us", "latitude", "longitude",
         "altitude_m", "fix_quality", "satellites"],
    ),
    # adc_v is the calibrated voltage at the pin, before the divider is
    # undone. It is stored alongside battery_v so a wrong divider
    # constant can be corrected after the fact instead of costing the
    # flight: battery_v = adc_v / ratio.
    STREAM_BATT: (
        "batt",
        "<IQff",
        ["seq", "timestamp_us", "battery_v", "adc_v"],
    ),
    # suppressed is how many identical errors from this source were
    # rate-limited away since the previous record. A large value is the
    # difference between one glitch and a sensor failing continuously.
    STREAM_ERR: (
        "err",
        "<IQBBhII",
        ["seq", "timestamp_us", "source", "code", "esp_err", "detail",
         "suppressed"],
    ),
    STREAM_EVENT: (
        "event",
        "<IQBBHI",
        ["seq", "timestamp_us", "type", "channel", "flags", "detail"],
    ),
    # mode is flight_mode_t; flags is the STATUS_FLAG_* bitfield, left as
    # an integer so a decoder change cannot silently drop a new bit.
    STREAM_STATUS: (
        "status",
        "<IQBBHfII",
        ["seq", "timestamp_us", "mode", "flags", "uptime_s",
         "altitude_agl_m", "sd_drops", "telem_drops"],
    ),
}

# Older on-card layouts, keyed by (stream_id, record_size). The file
# header records the size the firmware actually wrote, so a log from
# older firmware still decodes instead of tripping the size check.
LEGACY_STREAMS = {
    # log version 1: status carried battery_v (now only in batt.bin)
    (STREAM_STATUS, 32): (
        "status",
        "<IQBBHffII",
        ["seq", "timestamp_us", "mode", "flags", "uptime_s", "battery_v",
         "altitude_agl_m", "sd_drops", "telem_drops"],
    ),
}

# Columns rendered through a name table instead of as raw integers.
ENUM_COLUMNS = {
    STREAM_ERR:   {"source": ERR_SOURCES, "code": ERR_CODES},
    STREAM_EVENT: {"type": EVENT_TYPES},
    STREAM_STATUS: {"mode": MODES},
}


def decode_file(path, to_stdout=False):
    with open(path, "rb") as f:
        raw_header = f.read(HEADER_SIZE)
        if len(raw_header) < HEADER_SIZE:
            print(f"  ERROR: {path} is too short to hold a header", file=sys.stderr)
            return False

        magic, version, stream_id, record_size, _reserved = struct.unpack(
            HEADER_FORMAT, raw_header
        )

        if magic != MAGIC:
            print(f"  ERROR: {path} bad magic {magic!r} (expected {MAGIC!r}) -- "
                  f"not a flight log, or written by different firmware",
                  file=sys.stderr)
            return False

        if stream_id not in STREAMS:
            print(f"  ERROR: {path} unknown stream_id {stream_id}", file=sys.stderr)
            return False

        name, fmt, columns = LEGACY_STREAMS.get((stream_id, record_size),
                                                STREAMS[stream_id])
        expected_size = struct.calcsize(fmt)

        # The header records what the firmware actually wrote. If that
        # disagrees with this script's layout, the firmware and decoder
        # are out of sync -- stop rather than emit plausible nonsense.
        if record_size != expected_size:
            print(f"  ERROR: {path} record_size {record_size} but this script "
                  f"expects {expected_size} for stream '{name}'. Firmware and "
                  f"decoder are out of sync.", file=sys.stderr)
            return False

        body = f.read()

    stride = record_size + CRC_SIZE
    total = len(body) // stride
    leftover = len(body) % stride

    rows = []
    crc_failures = 0
    for i in range(total):
        entry = body[i * stride:(i + 1) * stride]
        payload = entry[:record_size]
        stored_crc = struct.unpack("<H", entry[record_size:])[0]
        if crc16_ccitt(payload) != stored_crc:
            crc_failures += 1
            continue          # corrupt entry -- skip rather than emit bad data
        row = list(struct.unpack(fmt, payload))
        for col, table in ENUM_COLUMNS.get(stream_id, {}).items():
            row[columns.index(col)] = _name(table, row[columns.index(col)])
        rows.append(tuple(row))

    # Sanity signals worth surfacing rather than silently ignoring.
    notes = []
    if crc_failures:
        notes.append(f"{crc_failures} entr(ies) failed CRC and were skipped "
                     f"-- card corruption or interrupted write")
    if leftover:
        notes.append(f"{leftover} trailing bytes ignored (likely power cut "
                     f"mid-record -- expected, not a bug)")
    if rows:
        seqs = [r[0] for r in rows]
        gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)
        if gaps:
            notes.append(f"{gaps} sequence gap(s) -- dropped samples "
                         f"(check queue_full_drops in diagnostics)")

        span_us = rows[-1][1] - rows[0][1]
        if span_us > 0 and len(rows) > 1:
            hz = (len(rows) - 1) / (span_us / 1_000_000)
            notes.append(f"~{hz:.1f} Hz average over {span_us/1_000_000:.1f} s")

    out_lines = [",".join(columns)]
    for row in rows:
        out_lines.append(",".join(repr(v) if isinstance(v, float) else str(v)
                                  for v in row))

    if to_stdout:
        print("\n".join(out_lines))
    else:
        out_path = os.path.splitext(path)[0] + ".csv"
        with open(out_path, "w") as out:
            out.write("\n".join(out_lines) + "\n")
        print(f"  {path} -> {out_path}")

    print(f"    stream={name} version={version} records={total}")
    for note in notes:
        print(f"    note: {note}")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Decode SET flight computer binary logs into CSV."
    )
    parser.add_argument("files", nargs="+", help="one or more .bin log files")
    parser.add_argument("--stdout", action="store_true",
                        help="print CSV to stdout instead of writing .csv files")
    args = parser.parse_args()

    ok = True
    for path in args.files:
        if not os.path.exists(path):
            print(f"  ERROR: {path} not found", file=sys.stderr)
            ok = False
            continue
        if not decode_file(path, to_stdout=args.stdout):
            ok = False

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
