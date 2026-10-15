#!/usr/bin/env python3
"""
Ground station -> vehicle uplink commands, for the SET I Osiris avionics.

This is the laptop half of the protocol implemented in
components/telemetry/telemetry.c.  Import it from blue_raven_homebrew, or
run it directly:

    python tools/uplink_command.py --port COM5 arm
    python tools/uplink_command.py --port COM5 mark --arg 3
    python tools/uplink_command.py --selftest          # no hardware needed
    python tools/uplink_command.py --port COM5 --transparent ping   # ground radio at AP=0

WIRE FORMAT (10 bytes, little-endian)

    offset  size  field
    0       1     magic     0x5A
    1       1     ver_cmd   high nibble = version (1), low nibble = opcode
    2       2     seq       uint16, STRICTLY INCREASING
    4       4     arg       uint32, command-specific, 0 where unused
    8       2     crc       CRC16-CCITT (poly 0x1021, init 0xFFFF) over bytes 0..7

Those ten bytes are the RF payload of an XBee 0x10 Transmit Request. The
vehicle sees them inside a 0x90 Receive Packet frame and hands the payload
straight to telemetry_handle_command().

THE SEQUENCE NUMBER IS THE PART THAT BITES YOU.

The vehicle discards any command whose seq is not newer than the last one it
accepted. That is what stops the XBee's own retransmit from being logged as a
second ARM press. It also means a ground station that restarts its counter at
1 will be ignored until the vehicle reboots. So the counter is persisted to
disk (see SeqCounter) rather than living in memory. If you ever do need to
resynchronise, power-cycle the flight computer -- there is deliberately no
command to reset it, because such a command would be exactly the replay hole
the sequence number exists to close.

WHAT THE VEHICLE DOES WITH EACH OPCODE

    NOP     0x0  accepted, no effect. Use it to prove the link end to end.
    ARM     0x1  STANDBY -> ARMED: sensors, SD and downlink go to flight rates.
                 Ignored in any other state.
    DISARM  0x2  -> STANDBY. REFUSED while the vehicle is in FLIGHT; the
                 refusal is recorded in event.bin as EV_UPLINK_REJECTED.
                 Allowed again once landing is detected.
    PING    0x3  emit one status record immediately instead of waiting for
                 the 1 Hz tick.
    MARK    0x4  write `arg` into event.bin as an operator marker -- for
                 "this is the moment we did X" during a test.
    SD_EJECT 0x5 flush the buffers, close the files and unmount the card so
                 it can be pulled without corrupting the filesystem.
                 ACCEPTED ONLY IN STANDBY AND LANDED. Refused in ARMED and
                 in FLIGHT, and the refusal is recorded as
                 EV_UPLINK_REJECTED with the mode in the detail field.

                 It is ONE WAY. There is no remount command, by the same
                 reasoning that there is no seq reset: the confirmation the
                 ground watches for is STATUS_FLAG_SD_OK going clear, and
                 that bit only means something if it cannot come back on
                 its own. Power-cycle the flight computer to log again.

                 Note what "accepted" means here: the vehicle has queued
                 the unmount, not finished it. The unmount takes about half
                 a second and happens in the status task, which then emits
                 the status record with SD_OK clear. That record, not the
                 acceptance, is the thing that says the card is safe to
                 pull.

A command is not acknowledged. Confirmation comes from the status stream:
STATUS_FLAG_* and the mode field tell you what the vehicle actually did, and
the cmds_* counters tell you how many were rejected and why.
"""

import argparse
import os
import struct
import sys

# ---------------------------------------------------------------- protocol

CMD_MAGIC   = 0x5A          # the downlink's 0xA5 reversed, on purpose:
                            # a looped-back downlink can never parse as this
CMD_VERSION = 1
CMD_SIZE    = 10

NOP, ARM, DISARM, PING, MARK, SD_EJECT, CONFIG_APPLY = (
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6)

COMMANDS = {
    "nop":          NOP,
    "arm":          ARM,
    "disarm":       DISARM,
    "ping":         PING,
    "mark":         MARK,
    "sd_eject":     SD_EJECT,
    "config_apply": CONFIG_APPLY,
}


def crc16_ccitt(data: bytes) -> int:
    """Poly 0x1021, init 0xFFFF, no reflection, no final xor.

    Byte-for-byte the same function as crc16_ccitt() in telemetry.c; the
    host test in test/host/ checks that a vector built here is accepted
    there.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_command(cmd: int, seq: int, arg: int = 0) -> bytes:
    """The 10 payload bytes. Raises on anything the vehicle would reject."""
    if not 0 <= cmd <= 0x0F:
        raise ValueError("opcode must fit in a nibble")
    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq must fit in uint16")
    if not 0 <= arg <= 0xFFFFFFFF:
        raise ValueError("arg must fit in uint32")
    if seq == 0:
        # last_cmd_seq starts at 0 on the vehicle, so seq 0 is never newer.
        raise ValueError("seq 0 is reserved and will always be discarded")
    body = struct.pack("<BBHI", CMD_MAGIC, (CMD_VERSION << 4) | cmd, seq, arg)
    return body + struct.pack("<H", crc16_ccitt(body))


# --------------------------------------------------- config upload packets

CONFIG_MAGIC       = 0x5B      # 0x5A+1; a command can never parse as a chunk
CONFIG_VERSION     = 1
CONFIG_HDR_SIZE    = 13
CONFIG_SUB_CHUNK   = 0
CONFIG_SUB_COMMIT  = 1
CONFIG_CHUNK_BYTES = 96        # must match CONFIG_RX_CHUNK_BYTES on the vehicle
CONFIG_MAX_BYTES   = 4096      # must match CONFIG_RX_MAX_BYTES


def _config_packet(sub, seq, cfg_crc, chunk_idx, chunk_count, total_len, payload):
    """One config packet. See config_receiver.h for the field list."""
    if len(payload) > 0xFF:
        raise ValueError("payload must fit in a uint8 length")
    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq must fit in uint16")
    if seq == 0:
        raise ValueError("seq 0 is reserved and will always be discarded")
    body = struct.pack("<BBHHHHHB", CONFIG_MAGIC,
                       (CONFIG_VERSION << 4) | sub, seq, cfg_crc,
                       chunk_idx, chunk_count, total_len, len(payload)) + payload
    assert len(body) == CONFIG_HDR_SIZE + len(payload)
    return body + struct.pack("<H", crc16_ccitt(body))


def config_packets(text, seq_source):
    """Every packet needed to upload `text`, chunks then the commit.

    `seq_source` is called once per packet and must return the next
    uplink sequence number -- the SAME counter the commands use, so the
    vehicle's strictly-increasing rule drops the XBee's own retransmits
    for a config exactly as it does for an ARM.

    cfg_crc is the CRC of the whole config and doubles as the transfer
    id: re-sending the same text resumes the same transfer, while a
    changed text starts a fresh one and discards whatever partial buffer
    the vehicle was holding.
    """
    data = text.encode("utf-8")
    if not data:
        raise ValueError("config is empty")
    if len(data) >= CONFIG_MAX_BYTES:
        raise ValueError(f"config is {len(data)} bytes; the vehicle accepts "
                         f"under {CONFIG_MAX_BYTES}")

    cfg_crc = crc16_ccitt(data)
    total = len(data)
    chunks = [data[i:i + CONFIG_CHUNK_BYTES]
              for i in range(0, total, CONFIG_CHUNK_BYTES)]

    out = []
    for i, payload in enumerate(chunks):
        out.append(_config_packet(CONFIG_SUB_CHUNK, seq_source(), cfg_crc,
                                  i, len(chunks), total, payload))
    # The commit is what makes the vehicle answer. Without it, a lost
    # final chunk would leave the vehicle silent with a half-filled
    # buffer and the ground with nothing to wait for.
    out.append(_config_packet(CONFIG_SUB_COMMIT, seq_source(), cfg_crc,
                              0, len(chunks), total, b""))
    return out, cfg_crc, len(chunks)


# ------------------------------------------------------------- XBee framing

XBEE_BROADCAST = 0x000000000000FFFF


def xbee_tx_request(payload: bytes, dest: int = XBEE_BROADCAST,
                    frame_id: int = 1) -> bytes:
    """Wrap a payload in an XBee API 0x10 Transmit Request.

    Note the endianness switch: everything inside the payload is
    little-endian because the records are, but the XBee's own frame fields
    are big-endian because that is Digi's convention. The boundary is
    exactly here.
    """
    body = (bytes([0x10, frame_id])
            + dest.to_bytes(8, "big")
            + b"\xFF\xFE"       # reserved
            + bytes([0x00, 0x00])  # broadcast radius, options
            + payload)
    cks = (0xFF - (sum(body) & 0xFF)) & 0xFF
    return b"\x7E" + len(body).to_bytes(2, "big") + body + bytes([cks])


# --------------------------------------------------------- sequence counter

class SeqCounter:
    """Monotonic uint16 that survives restarts of the ground station.

    Stored next to this script by default. Wraps 65535 -> 1 (never 0), which
    the vehicle handles: it treats "newer" as a distance in the lower half of
    the 16-bit space rather than a plain comparison.
    """

    def __init__(self, path=None):
        self.path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         ".uplink_seq")
        self.value = 0
        try:
            with open(self.path) as f:
                self.value = int(f.read().strip()) & 0xFFFF
        except (OSError, ValueError):
            pass

    def next(self) -> int:
        self.value = self.value + 1 if self.value < 0xFFFF else 1
        try:
            with open(self.path, "w") as f:
                f.write(str(self.value))
        except OSError as e:
            print(f"warning: could not persist seq ({e}); "
                  f"a restart may be ignored by the vehicle", file=sys.stderr)
        return self.value


# ---------------------------------------------------------------- self test

_VECTORS = [
    # (cmd, seq, arg, expected bytes) -- the first one is printed by the C
    # host test, so the two implementations are pinned to each other.
    (ARM,    7,  0,          "5A 11 07 00 00 00 00 00 7D 2C"),
]


def selftest() -> int:
    bad = 0
    for cmd, seq, arg, want in _VECTORS:
        got = " ".join(f"{b:02X}" for b in build_command(cmd, seq, arg))
        ok = got == want
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'}  cmd=0x{cmd:X} seq={seq} arg={arg}")
        if not ok:
            print(f"        want {want}\n        got  {got}")

    for name, cmd in COMMANDS.items():
        pkt = build_command(cmd, 1234, 0xABCD1234)
        assert len(pkt) == CMD_SIZE
        assert pkt[0] == CMD_MAGIC and (pkt[1] >> 4) == CMD_VERSION
        assert (pkt[1] & 0x0F) == cmd
        assert struct.unpack("<H", pkt[2:4])[0] == 1234
        assert struct.unpack("<I", pkt[4:8])[0] == 0xABCD1234
        assert crc16_ccitt(pkt[:8]) == struct.unpack("<H", pkt[8:10])[0]
        print(f"  ok    {name:6s} round-trips")

    # ---- config upload packets ----
    body = '{"channels":{"Apo":{"primary":{"AGL1":true}}}}' * 5
    seqs = iter(range(1, 500))
    pkts, crc, nchunks = config_packets(body, lambda: next(seqs))
    assert len(pkts) == nchunks + 1
    print(f"  ok    config of {len(body)} bytes -> {nchunks} chunks + commit")
    for pk in pkts:
        assert pk[0] == CONFIG_MAGIC and (pk[1] >> 4) == CONFIG_VERSION
        assert crc16_ccitt(pk[:-2]) == struct.unpack("<H", pk[-2:])[0]
        assert struct.unpack_from("<H", pk, 4)[0] == crc
    print("  ok    every config packet carries the transfer crc and checks out")
    rebuilt = b"".join(pk[CONFIG_HDR_SIZE:-2] for pk in pkts[:-1])
    assert rebuilt == body.encode()
    print("  ok    chunks reassemble to the original bytes")
    assert (pkts[-1][1] & 0x0F) == CONFIG_SUB_COMMIT
    assert pkts[-1][12] == 0
    print("  ok    the last packet is an empty commit")
    # NOT named `bad` -- that is this function's failure counter, and
    # shadowing it here made a passing self-test report FAILED.
    for bad_seq, why in ((0, "config seq 0"), (0x10000, "config seq too large")):
        try:
            _config_packet(CONFIG_SUB_CHUNK, bad_seq, 0, 0, 1, 1, b"x")
        except ValueError:
            print(f"  ok    {why} rejected before transmission")
        else:
            print(f"  FAIL  {why} was not rejected")
            bad += 1

    frame = xbee_tx_request(build_command(ARM, 1))
    assert frame[0] == 0x7E
    assert int.from_bytes(frame[1:3], "big") == len(frame) - 4
    assert (sum(frame[3:-1]) + frame[-1]) & 0xFF == 0xFF
    print("  ok    XBee 0x10 frame length and checksum")

    for seq in (0, -1, 0x10000):
        try:
            build_command(ARM, seq)
        except ValueError:
            print(f"  ok    seq {seq} rejected before transmission")
        else:
            print(f"  FAIL  seq {seq} was not rejected")
            bad += 1

    print(f"\n{'FAILED' if bad else 'all good'}")
    return 1 if bad else 0


# ------------------------------------------------------- cross-check vectors

def emit_vectors(out=sys.stdout):
    """Vectors for test/host/test_uplink_command.c to replay.

    Format per line: "<expected verdict> <hex payload> <note>". The verdicts
    are the vehicle's own rejection categories, so a disagreement shows up as
    the specific thing the two sides read differently, not just "it failed".
    Order matters -- the sequence-number cases depend on what came before.
    """
    def line(expect, payload, note):
        out.write(f"{expect} {payload.hex().upper()} {note}\n")

    out.write("# generated by tools/uplink_command.py --emit-vectors\n")

    line("accept", build_command(NOP, 1), "python NOP seq=1 accepted")
    line("stale",  build_command(NOP, 1), "python retransmit of seq=1 discarded")
    line("accept", build_command(ARM, 2), "python ARM seq=2 accepted")
    line("stale",  build_command(PING, 1), "python replayed older seq discarded")
    line("accept", build_command(MARK, 3, 0xDEADBEEF), "python MARK carries a 32-bit arg")
    # Mode-gated, so its verdict depends on what came before: ARM above put
    # the vehicle in ARMED, where an eject is refused. Understood but
    # refused is a different counter from stale and from malformed, which
    # is the distinction being checked.
    line("refuse", build_command(SD_EJECT, 4), "python SD_EJECT refused while ARMED")
    line("accept", build_command(DISARM, 5), "python DISARM from ARMED accepted")
    line("accept", build_command(SD_EJECT, 6), "python SD_EJECT accepted in STANDBY")

    bad = bytearray(build_command(PING, 7)); bad[9] ^= 0x01
    line("badcrc", bytes(bad), "python packet with a flipped CRC bit")

    wrongver = bytearray(build_command(PING, 8)); wrongver[1] = (2 << 4) | PING
    line("malformed", bytes(wrongver), "python packet claiming version 2")

    line("malformed", bytes([0xA5, 0x10, 1, 0, 40, 1, 20, 0, 0, 0]),
         "a looped-back DOWNLINK packet is not a command")

    # "Newer" is a distance in the lower half of the 16-bit space, which is
    # what lets the counter roll over. The flip side is a WINDOW: a jump of
    # more than 32767 in one step reads as a jump backwards and is discarded.
    # That is by design -- it is the same arithmetic that rejects a replay --
    # so the ground station must step through the space, not leap across it.
    line("stale",  build_command(PING, 40000), "a jump of >32767 is out of window, discarded")
    line("accept", build_command(PING, 32000), "a large but in-window step is accepted")
    line("accept", build_command(NOP, 64000),  "another in-window step")
    line("accept", build_command(ARM, 1),      "65536 wraps to 1 and is still newer")


def emit_config_vectors(out=sys.stdout):
    """Config-upload vectors for test/host/test_config_receiver.c.

    Same idea as emit_vectors(): the only test that proves the laptop
    and the vehicle agree about the CONFIG format rather than each being
    self-consistent. Verdicts are what the receiver does with the
    packet -- "stored", "rejected", "config" (claimed, no verdict yet)
    or "notconfig" (not ours, falls through to the command parser).
    Order matters; the chunks must precede their commit.
    """
    seq = [1000]

    def nxt():
        seq[0] += 1
        return seq[0]

    def line(expect, payload, note):
        out.write(f"{expect} {payload.hex().upper()} {note}\n")

    out.write("# generated by tools/uplink_command.py --emit-config-vectors\n")

    body = ('{"channels":{"Apo":{"primary":{"APOGEE":true},'
            '"thresholds":{"AGL1":500.0},"latch":true,"delay_ms":1500},'
            '"Main":{"primary":{"AGL1":true},"latch":true,"delay_ms":0},'
            '"3rd":{},"4th":{}}}')
    pkts, _crc, nchunks = config_packets(body, nxt)
    for i, pk in enumerate(pkts[:-1]):
        line("config", pk, f"python config chunk {i + 1}/{nchunks} accepted")
    line("stored", pkts[-1], "python commit stores the reassembled config")

    # A command must NOT be claimed by the config receiver.
    line("notconfig", build_command(ARM, 1), "an ARM command is not a config packet")
    line("notconfig", bytes([0xA5, 0x10, 1, 0, 40, 1, 20, 0, 0, 0]),
         "a looped-back DOWNLINK is not a config packet")

    # A commit for a transfer that never happened must be rejected, not
    # silently ignored -- the ground is waiting for an answer.
    line("rejected", _config_packet(CONFIG_SUB_COMMIT, nxt(), 0xBEEF, 0, 1, 10, b""),
         "a commit with nothing buffered is rejected")

    # Only the last chunk missing: the commit still answers.
    body2 = "x" * (CONFIG_CHUNK_BYTES * 3)
    pkts2, crc2, n2 = config_packets(body2, nxt)
    for i, pk in enumerate(pkts2[:-2]):          # drop the final chunk
        line("config", pk, f"python partial upload, chunk {i + 1}/{n2}")
    line("rejected", pkts2[-1], "the commit names the missing final chunk")


# --------------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="?", choices=sorted(COMMANDS), help="opcode to send")
    ap.add_argument("--port", help="serial port the XBee is on, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--arg", type=lambda s: int(s, 0), default=0,
                    help="uint32 argument (MARK uses it; others ignore it)")
    ap.add_argument("--dest", type=lambda s: int(s, 0), default=XBEE_BROADCAST,
                    help="destination 64-bit address (default: broadcast)")
    ap.add_argument("--seq", type=int, help="override the persisted sequence number")
    ap.add_argument("--dry-run", action="store_true", help="print the bytes, send nothing")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--transparent", action="store_true",
                    help="ground radio is in transparent mode (AP=0): send the 10 bytes "
                         "as they are, no API frame; the radio's DH/DL is the destination")
    ap.add_argument("--emit-vectors", action="store_true",
                    help="write cross-check vectors for the C host test")
    ap.add_argument("--emit-config-vectors", action="store_true",
                    help="write config-upload cross-check vectors")
    args = ap.parse_args()

    if args.emit_vectors:
        emit_vectors()
        return 0
    if args.emit_config_vectors:
        emit_config_vectors()
        return 0
    if args.selftest:
        return selftest()
    if not args.command:
        ap.error("a command is required (or --selftest)")

    seq = args.seq if args.seq is not None else SeqCounter().next()
    payload = build_command(COMMANDS[args.command], seq, args.arg)
    frame = payload if args.transparent else xbee_tx_request(payload, args.dest)

    print(f"{args.command.upper()} seq={seq} arg=0x{args.arg:08X}")
    print(f"  payload : {' '.join(f'{b:02X}' for b in payload)}")
    if args.transparent:
        print("  frame   : none (transparent: the payload is what goes on the wire)")
    else:
        print(f"  frame   : {' '.join(f'{b:02X}' for b in frame)}")

    if args.dry_run:
        return 0
    if not args.port:
        print("no --port given; nothing sent (use --dry-run to silence this)",
              file=sys.stderr)
        return 2

    try:
        import serial            # pyserial
    except ImportError:
        print("pyserial is not installed: pip install pyserial", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        ser.write(frame)
    print("  sent. Watch the status stream to confirm what the vehicle did.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
