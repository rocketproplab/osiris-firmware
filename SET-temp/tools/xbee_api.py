"""xbee_api.py -- the small piece of Digi's API-mode-1 framing that both the
ground station and the link emulator need. AP=1 only: no byte escaping."""

API_START = 0x7E
BROADCAST = 0x000000000000FFFF
MAX_FRAME_LEN = 1024          # anything longer is treated as line noise


def checksum(frame_data: bytes) -> int:
    return 0xFF - (sum(frame_data) & 0xFF)


def build_frame(frame_data: bytes) -> bytes:
    n = len(frame_data)
    return bytes([API_START, n >> 8, n & 0xFF]) + frame_data + bytes([checksum(frame_data)])


class FrameParser:
    """Incremental parser. feed() returns a list of events:
         ("frame", frame_data)   valid frame, start/length/checksum stripped
         ("junk",  raw_bytes)    bytes that are not part of any frame
         ("bad",   raw_bytes)    a candidate frame whose checksum failed
    On a bad checksum only the 0x7E is consumed, so a real frame starting
    inside the bad candidate is still found."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes):
        self.buf += chunk
        out = []
        while self.buf:
            if self.buf[0] != API_START:
                i = self.buf.find(API_START)
                end = len(self.buf) if i < 0 else i
                out.append(("junk", bytes(self.buf[:end])))
                del self.buf[:end]
                continue
            if len(self.buf) < 3:
                break
            n = (self.buf[1] << 8) | self.buf[2]
            if n == 0 or n > MAX_FRAME_LEN:
                out.append(("junk", bytes(self.buf[:1])))
                del self.buf[:1]
                continue
            if len(self.buf) < n + 4:
                break
            data = bytes(self.buf[3:3 + n])
            if checksum(data) != self.buf[3 + n]:
                out.append(("bad", bytes(self.buf[:n + 4])))
                del self.buf[:1]
                continue
            out.append(("frame", data))
            del self.buf[:n + 4]
        return out
