#!/usr/bin/env python3
"""
xbee_link_emulator.py -- stand in for BOTH XBee radios so the ground station
can talk to the flight computer over a plain wire.

Why this exists
---------------
The vehicle writes 0x10 Transmit Request frames into its radio and expects
0x90 Receive Packet frames back. The ground station does the same from the
other side. Wire the two together directly and each gets the wrong frame type.
This program sits in the middle and behaves the way the pair of radios would:

  vehicle  --0x10-->  [emulated vehicle radio] ~~air~~ [emulated ground radio]  --0x90-->  ground
  vehicle  <--0x90--  [emulated vehicle radio] ~~air~~ [emulated ground radio]  <--0x10--  ground

Each emulated radio also answers AT commands (0x08) with 0x88 and reports
0x8B Transmit Status, so the firmware's ATVR bring-up probe passes and the
ground tools' ATDB queries get an answer.

Wiring (SparkFun XBee Explorer, NO XBee in its socket, NO XBee on the board)
---------------------------------------------------------------------------
  Explorer pin 2 (DOUT, into the FTDI)  <- board GPIO3 / J5.4   vehicle TX
  Explorer pin 3 (DIN, out of the FTDI) -> board GPIO4 / J5.3   vehicle RX
  Explorer GND                          -- board GND
  Leave 3V3 unconnected.

Usage
-----
  python tools/xbee_link_emulator.py --vehicle-port COM4

Start this BEFORE resetting the board, or the firmware's ATVR probe goes
unanswered and it falls back to the bootloader escape.

The ground side is a TCP port (default 127.0.0.1:9750), not a COM port -- no
virtual-serial driver needed. Any ground tool built on pyserial can use it
by opening the port with serial.serial_for_url() instead of serial.Serial():

  serial.serial_for_url("socket://127.0.0.1:9750", baudrate=115200, timeout=0.1)

serial_for_url("COM5", ...) behaves exactly like serial.Serial("COM5", ...),
so that one change costs nothing when a real ground radio arrives -- you
change the port string and nothing else.

What is NOT emulated
--------------------
RF behaviour: range, real RSSI (ATDB returns the constant --rssi), timing,
mesh routing. The 0x8A modem-status frame a real radio emits after reset is
not sent (the emulator cannot see the /RESET line). --drop gives crude
random loss for exercising the ground station's lost-packet counting.

API mode 1 only (AP=1, no escaping), which is what the firmware uses.
"""

import argparse
import collections
import random
import socket
import sys
import threading
import time

from xbee_api import BROADCAST, FrameParser, build_frame

# 0x8B delivery status codes (DigiMesh)
DS_SUCCESS = 0x00
DS_NETWORK_ACK_FAILURE = 0x21
DS_ROUTE_NOT_FOUND = 0x25
DS_PAYLOAD_TOO_LARGE = 0x74

# AT commands a real radio accepts that have nothing to report back
AT_NO_VALUE = {"WR", "AC", "FR", "RE"}


# --------------------------------------------------------------------------
# Radio + link logic (pure: bytes in, frames out, no I/O)
# --------------------------------------------------------------------------

class Radio:
    def __init__(self, name, addr64, vr=0x8075, rssi=40, max_payload=256):
        self.name = name
        self.addr64 = addr64
        self.max_payload = max_payload
        self.at = {
            "VR": vr.to_bytes(2, "big"),
            "SH": (addr64 >> 32).to_bytes(4, "big"),
            "SL": (addr64 & 0xFFFFFFFF).to_bytes(4, "big"),
            "MY": b"\xFF\xFE",
            "ID": b"\x7F\xFF",
            "HP": b"\x00",
            "AP": b"\x01",
            "BD": b"\x07",            # 7 = 115200
            "NP": max_payload.to_bytes(2, "big"),
            "DB": bytes([rssi & 0xFF]),
        }


class Link:
    """Two radios and the air between them. from_host(side, frame_data)
    returns [(side, frame_bytes), ...] -- what each host should receive."""

    SIDES = ("vehicle", "ground")

    def __init__(self, vehicle: Radio, ground: Radio, drop=0.0, retries=3,
                 rng=None, log=None):
        self.radios = {"vehicle": vehicle, "ground": ground}
        self.drop = drop
        self.retries = retries
        self.rng = rng or random.Random()
        self.log = log or (lambda msg: None)
        self.stats = collections.Counter()
        self._reported = set()

    @staticmethod
    def other(side):
        return "ground" if side == "vehicle" else "vehicle"

    def _once(self, key, msg):
        if key not in self._reported:
            self._reported.add(key)
            self.log(msg)

    def _dropped(self):
        return self.drop > 0 and self.rng.random() < self.drop

    def from_host(self, side, data: bytes):
        ftype = data[0]
        if ftype in (0x08, 0x09):
            return self._at(side, data)
        if ftype == 0x10:
            return self._tx(side, data)
        self.stats[f"{side}_unsupported"] += 1
        self._once((side, "type", ftype),
                   f"{side}: frame type 0x{ftype:02X} not emulated, ignored")
        return []

    def _at(self, side, data):
        radio = self.radios[side]
        if len(data) < 4:
            self.stats[f"{side}_malformed"] += 1
            self.log(f"{side}: AT frame too short: {data.hex(' ')}")
            return []
        self.stats[f"{side}_at"] += 1
        fid = data[1]
        cmd = data[2:4].decode("ascii", "replace")
        param = data[4:]

        if param:                                    # a set command
            radio.at[cmd] = bytes(param)
            status, value = 0x00, b""
            if cmd == "AP" and param != b"\x01":
                self.log(f"{side}: AP set to {param.hex()} -- this emulator "
                         "only speaks AP=1, frames will still be AP=1")
            self._once((side, "set", cmd), f"{side}: AT{cmd} = {param.hex()}")
        elif cmd in radio.at:
            status, value = 0x00, radio.at[cmd]
            self._once((side, "get", cmd),
                       f"{side}: AT{cmd} -> {value.hex() or '(empty)'}")
        elif cmd in AT_NO_VALUE:
            status, value = 0x00, b""
            self._once((side, "get", cmd), f"{side}: AT{cmd} -> OK")
        else:
            status, value = 0x02, b""                # invalid command
            self._once((side, "unk", cmd),
                       f"{side}: AT{cmd} not emulated, answered ERROR")

        if fid == 0:                                 # frame ID 0: no response
            return []
        return [(side, build_frame(bytes([0x88, fid]) + data[2:4]
                                   + bytes([status]) + value))]

    def _tx(self, side, data):
        radio = self.radios[side]
        if len(data) < 14:
            self.stats[f"{side}_malformed"] += 1
            self.log(f"{side}: 0x10 frame too short: {data.hex(' ')}")
            return []
        self.stats[f"{side}_tx"] += 1
        fid = data[1]
        dest = int.from_bytes(data[2:10], "big")
        rf = data[14:]
        peer_side = self.other(side)
        peer = self.radios[peer_side]

        retries_used = 0
        delivered = False
        if len(rf) > radio.max_payload:
            status = DS_PAYLOAD_TOO_LARGE
            self._once((side, "big"), f"{side}: RF payload {len(rf)} B exceeds "
                       f"{radio.max_payload} B, rejected (0x74)")
        elif dest == BROADCAST:
            status = DS_SUCCESS                      # broadcasts always "succeed"
            delivered = not self._dropped()
        elif dest == peer.addr64:
            for attempt in range(self.retries + 1):
                retries_used = attempt
                if not self._dropped():
                    delivered = True
                    break
            status = DS_SUCCESS if delivered else DS_NETWORK_ACK_FAILURE
        else:
            status = DS_ROUTE_NOT_FOUND
            self._once((side, "dest", dest),
                       f"{side}: unicast to {dest:016X}, no such radio "
                       f"(the {peer_side} radio is {peer.addr64:016X}) -> 0x25")

        out = []
        if delivered:
            opts = 0xC2 if dest == BROADCAST else 0xC1
            rx = (bytes([0x90]) + radio.addr64.to_bytes(8, "big")
                  + b"\xFF\xFE" + bytes([opts]) + rf)
            out.append((peer_side, build_frame(rx)))
            self.stats[f"{side}_delivered"] += 1
        else:
            self.stats[f"{side}_lost"] += 1
        if fid:
            out.append((side, build_frame(bytes(
                [0x8B, fid, 0xFF, 0xFE, retries_used, status, 0x00]))))
        return out


# --------------------------------------------------------------------------
# I/O: vehicle on a serial port, ground on a TCP socket
# --------------------------------------------------------------------------

class Emulator:
    def __init__(self, link: Link, vehicle_port, listen_host="127.0.0.1",
                 listen_port=9750, capture_path=None, log=None, junk_log_limit=10):
        self.link = link
        self.vehicle = vehicle_port
        self.log = log or (lambda msg: None)
        self.lock = threading.Lock()
        self.parsers = {"vehicle": FrameParser(), "ground": FrameParser()}
        self.client = None
        self.running = False
        self.capture = open(capture_path, "ab") if capture_path else None
        self.junk_log_left = junk_log_limit

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((listen_host, listen_port))
        self.server.listen(1)
        self.server.settimeout(0.2)
        self.listen_port = self.server.getsockname()[1]

    # -- output --
    def _send(self, side, frame):
        if side == "vehicle":
            self.vehicle.write(frame)
            return
        if self.capture:
            self.capture.write(frame)
            self.capture.flush()
        c = self.client
        if c is None:
            self.link.stats["ground_no_client"] += 1
            return
        try:
            c.sendall(frame)
        except OSError:
            self._drop_client(c, "send failed")

    # -- input --
    def on_bytes(self, side, chunk):
        with self.lock:
            for kind, payload in self.parsers[side].feed(chunk):
                if kind == "frame":
                    for dst, frame in self.link.from_host(side, payload):
                        self._send(dst, frame)
                elif kind == "junk":
                    self.link.stats[f"{side}_junk_bytes"] += len(payload)
                    if self.junk_log_left > 0:
                        self.junk_log_left -= 1
                        self.log(f"{side}: {len(payload)} non-frame byte(s): "
                                 f"{payload[:32]!r}")
                else:
                    self.link.stats[f"{side}_bad_checksum"] += 1
                    self.log(f"{side}: bad checksum, candidate "
                             f"{payload[:24].hex(' ')}")

    def _vehicle_reader(self):
        while self.running:
            try:
                chunk = self.vehicle.read(max(1, self.vehicle.in_waiting))
            except Exception as e:                   # port unplugged, etc.
                self.log(f"vehicle port error: {e}")
                self.running = False
                return
            if chunk:
                self.on_bytes("vehicle", chunk)

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                old, self.client = self.client, conn
                self.parsers["ground"] = FrameParser()
            if old:
                self._drop_client(old, "replaced by new connection")
            self.log(f"ground client connected from {addr[0]}:{addr[1]}")
            threading.Thread(target=self._client_reader, args=(conn,),
                             daemon=True).start()

    def _client_reader(self, conn):
        while self.running:
            try:
                chunk = conn.recv(4096)
            except OSError:
                chunk = b""
            if not chunk:
                self._drop_client(conn, "disconnected")
                return
            if conn is self.client:
                self.on_bytes("ground", chunk)

    def _drop_client(self, conn, why):
        with self.lock:
            was_current = conn is self.client
            if was_current:
                self.client = None
        try:
            conn.close()
        except OSError:
            pass
        if was_current or why.startswith("replaced"):
            self.log(f"ground client {why}")

    def start(self):
        self.running = True
        for target in (self._vehicle_reader, self._accept_loop):
            threading.Thread(target=target, daemon=True).start()

    def stop(self):
        self.running = False
        try:
            self.server.close()
        except OSError:
            pass
        if self.client:
            self._drop_client(self.client, "closed")
        if self.capture:
            self.capture.close()

    def status_line(self):
        s = self.link.stats
        return (f"veh->gnd {s['vehicle_tx']} ({s['vehicle_lost']} lost)  "
                f"gnd->veh {s['ground_tx']} ({s['ground_lost']} lost)  "
                f"AT veh/gnd {s['vehicle_at']}/{s['ground_at']}  "
                f"bad cksum {s['vehicle_bad_checksum']}/{s['ground_bad_checksum']}  "
                f"junk {s['vehicle_junk_bytes']}/{s['ground_junk_bytes']} B  "
                f"client {'yes' if self.client else 'no'}")


# --------------------------------------------------------------------------

def main(argv=None):
    import serial

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vehicle-port", required=True,
                    help="serial port wired to the flight computer (COM4), or any pyserial URL")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--listen", default="127.0.0.1:9750",
                    help="host:port the ground station connects to (default %(default)s)")
    ap.add_argument("--vehicle-addr", default="0013A20000000001",
                    help="64-bit address of the emulated vehicle radio (hex)")
    ap.add_argument("--ground-addr", default="0013A20000000002",
                    help="64-bit address of the emulated ground radio (hex); must match "
                         "the firmware's dest_addr if that is not broadcast")
    ap.add_argument("--vr", default="8075", help="ATVR firmware version reported (hex)")
    ap.add_argument("--rssi", type=int, default=40, help="ATDB value reported, -dBm")
    ap.add_argument("--max-payload", type=int, default=256,
                    help="max RF payload bytes (check ATNP on a real module)")
    ap.add_argument("--drop", type=float, default=0.0,
                    help="probability each over-the-air attempt is lost, 0..1")
    ap.add_argument("--retries", type=int, default=3,
                    help="unicast retries before 0x21 (like ATRR)")
    ap.add_argument("--seed", type=int, help="random seed for --drop")
    ap.add_argument("--capture", help="append every byte the ground radio outputs to this file")
    ap.add_argument("--status-interval", type=float, default=5.0)
    args = ap.parse_args(argv)

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    host, _, port = args.listen.rpartition(":")
    vr = int(args.vr, 16)
    vehicle = Radio("vehicle", int(args.vehicle_addr, 16), vr, args.rssi, args.max_payload)
    ground = Radio("ground", int(args.ground_addr, 16), vr, args.rssi, args.max_payload)
    link = Link(vehicle, ground, args.drop, args.retries, random.Random(args.seed), log)

    ser = serial.serial_for_url(args.vehicle_port, baudrate=args.baud, timeout=0.1)
    emu = Emulator(link, ser, host or "127.0.0.1", int(port), args.capture, log)
    emu.start()
    log(f"vehicle on {args.vehicle_port} @ {args.baud}; ground tools connect to "
        f"socket://{host or '127.0.0.1'}:{emu.listen_port}")
    log("reset the flight computer now so its ATVR probe is answered")
    try:
        while emu.running:
            time.sleep(args.status_interval)
            log(emu.status_line())
    except KeyboardInterrupt:
        pass
    finally:
        emu.stop()
        ser.close()
        log("final: " + emu.status_line())
    return 0


if __name__ == "__main__":
    sys.exit(main())
