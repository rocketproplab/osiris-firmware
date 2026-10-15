"""downlink_stream.py -- find downlink telemetry packets in a byte stream.

With the vehicle radio in API mode, every telemetry packet arrives at the
ground in a 0x90 frame of its own, so a 0x90 payload IS one packet. With
the vehicle radio in TRANSPARENT mode (menuconfig -> XBee telemetry link,
the backup mode) that is no longer guaranteed: the vehicle's module cuts
RF packets wherever its UART input pauses or its buffer fills, so one
telemetry packet can arrive split over two 0x90 frames, or two packets
can share one. And a capture taken on a transparent vehicle's UART has
no frames at all, just packets back to back.

Every telemetry packet is self-delimiting (telemetry.h):

    0 magic 0xA5 | 1 ver<<4|flags | 2 seq u16 | 4 rssi | 5 count | 6 len u16
    8 records ... | 8+len crc u16, CRC16-CCITT over bytes 0 .. 8+len-1

so this finds them by magic, length and CRC. A candidate whose CRC fails
costs one byte and the search resumes at the next, so a real packet that
overlaps a false start is still found. A partial that stops growing for
STALE_S seconds is given up (anything valid inside it is still found).

The firmware does the same thing in the other direction for uplink
packets: components/telemetry/uplink_stream.c.
"""

import collections

PKT_MAGIC = 0xA5
PKT_HDR = 8
MAX_PACKET = 256                       # TELEMETRY_MAX_PACKET
MAX_LEN = MAX_PACKET - PKT_HDR - 2     # largest legal `len` field
STALE_S = 1.0


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def is_whole_packet(p: bytes) -> bool:
    """True if p is shaped like exactly one packet (magic and length).
    Says nothing about the CRC -- the decoder checks and reports that."""
    return (len(p) >= PKT_HDR + 2 and p[0] == PKT_MAGIC
            and PKT_HDR + (p[6] | (p[7] << 8)) + 2 == len(p))


class DownlinkStream:
    """feed(bytes[, now]) -> list of CRC-valid packets, in order.
    expire(now) applies the stale timeout on a quiet link; flush() ends
    a file (everything left is incomplete)."""

    def __init__(self, stale_s=STALE_S):
        self.buf = bytearray()
        self.stale_s = stale_s
        self.last = None
        self.stats = collections.Counter()

    @property
    def pending(self):
        return len(self.buf)

    def _step(self, out):
        b = self.buf
        if not b:
            return False
        if b[0] != PKT_MAGIC:
            i = b.find(PKT_MAGIC)
            n = len(b) if i < 0 else i
            del b[:n]
            self.stats["discarded_bytes"] += n
            return True
        if len(b) < PKT_HDR:
            return False
        length = b[6] | (b[7] << 8)
        if length > MAX_LEN:
            del b[:1]
            self.stats["discarded_bytes"] += 1
            return True
        total = PKT_HDR + length + 2
        if len(b) < total:
            return False
        if crc16_ccitt(b[:total - 2]) != (b[total - 2] | (b[total - 1] << 8)):
            del b[:1]
            self.stats["crc_resyncs"] += 1
            self.stats["discarded_bytes"] += 1
            return True
        out.append(bytes(b[:total]))
        del b[:total]
        self.stats["packets"] += 1
        return True

    def _peel(self, out):
        while self.buf:
            del self.buf[:1]
            self.stats["discarded_bytes"] += 1
            while self._step(out):
                pass

    def expire(self, now):
        out = []
        if self.buf and self.last is not None and now - self.last >= self.stale_s:
            self.stats["stale_flushes"] += 1
            self._peel(out)
        return out

    def flush(self):
        out = []
        if self.buf:
            self._peel(out)
        return out

    def feed(self, data: bytes, now=None):
        out = self.expire(now) if now is not None else []
        if data:
            self.buf += data
            if now is not None:
                self.last = now
            while self._step(out):
                pass
        return out


def _selftest():
    import random
    import struct
    rng = random.Random(1)

    def mk(seq, n):
        body = struct.pack("<BBHBBH", PKT_MAGIC, 0x10, seq, 0, 1, n) + bytes(
            rng.randrange(256) for _ in range(n))
        return body + struct.pack("<H", crc16_ccitt(body))

    pkts = [mk(i, rng.randrange(0, MAX_LEN + 1)) for i in range(300)]
    stream = bytearray()
    for p in pkts:
        for _ in range(rng.randrange(6)):
            stream.append(rng.choice([PKT_MAGIC, 0x00, 0xFF, rng.randrange(256)]))
        stream += p
    s = DownlinkStream()
    got, i = [], 0
    while i < len(stream):
        k = rng.randrange(1, 300)
        got += s.feed(bytes(stream[i:i + k]))
        i += k
    got += s.flush()
    assert got == pkts, f"{len(got)} of {len(pkts)}"

    s = DownlinkStream()
    noise = bytes(rng.randrange(256) for _ in range(100000))
    assert s.feed(noise) + s.flush() == [], "noise produced a packet"

    s = DownlinkStream(stale_s=1.0)
    p = mk(7, 20)
    assert s.feed(p[:5], now=0.0) == []
    assert s.expire(0.5) == [] and s.pending == 5
    assert s.expire(1.0) == [] and s.pending == 0
    assert s.feed(p, now=2.0) == [p]
    print("downlink_stream selftest: 300 packets in noise and random pieces, "
          "100 kB of noise, stale timeout -- ok")


if __name__ == "__main__":
    _selftest()
