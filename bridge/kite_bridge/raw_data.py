"""Size and fingerprint of the raw data a sensor poll returned, and a plain-data copy of one
reading for the raw dump. Standard library only: it never imports numpy or PIL, it calls
`tobytes()` on whatever has it.

Input domain, and nothing beyond it: a raw value is None, bytes, bytearray, str, bool, int
(within 64 bits), float, a dict with str keys, a list, a tuple, an object with `tobytes()` (a
numpy array or scalar, a PIL image), or an object with `__len__` and `__iter__` whose items are
in this domain. Anything else raises ValueError. A raw value is never truth-tested (R28).

How bytes are counted, here and nowhere else. The count is the payload the bridge received, not
what crossed the wire:

- bytes and bytearray: their length.
- an object with `tobytes()`: the length of what it returns. It is called once per object.
- bool: 1. int: 8 (a signed 64-bit value). float: 8 (a 64-bit float).
- str, and every dict key: the length of its UTF-8 encoding.
- None: 0.
- a dict, list, tuple or other iterable: the sum of its items. Brackets, lengths and type tags
  that go into the fingerprint are not counted.

The fingerprint is a hash of the same content with type tags and lengths added. Two raw values
get the same fingerprint when they hold the same content, whatever the dict key order.
"""

import hashlib
import math
import struct
from dataclasses import dataclass

from kite_bridge.readings import readings_in
from kite_bridge.sim_api import BUFFERED_KINDS

INT64_MIN = -(2 ** 63)
INT64_MAX = 2 ** 63 - 1


@dataclass(frozen=True)
class PollStats:
    """`nbytes` is the payload size of the poll. `keys` has one fingerprint for the whole poll,
    or, for a sensor that buffers readings (GPS, IMU), one per reading in it. Empty when the
    poll returned nothing."""

    nbytes: int
    keys: tuple[str, ...]


EMPTY = PollStats(0, ())


class _Walk:
    def __init__(self):
        self.hash = hashlib.blake2b(digest_size=16)
        self.nbytes = 0

    def tag(self, text):
        self.hash.update(text)

    def payload(self, data):
        self.hash.update(len(data).to_bytes(8, "little"))
        self.hash.update(data)
        self.nbytes += len(data)

    def visit(self, value):
        if value is None:
            self.tag(b"N")
        elif isinstance(value, (bytes, bytearray)):
            self.tag(b"B")
            self.payload(bytes(value))
        elif hasattr(value, "tobytes"):
            self.tag(b"A")
            self.payload(value.tobytes())
        elif isinstance(value, str):
            self.tag(b"S")
            self.payload(value.encode("utf-8"))
        elif isinstance(value, bool):
            self.tag(b"O")
            self.payload(b"\x01" if value else b"\x00")
        elif isinstance(value, int):
            if not INT64_MIN <= value <= INT64_MAX:
                raise ValueError(f"integer {value} does not fit in 64 bits")
            self.tag(b"I")
            self.payload(struct.pack("<q", value))
        elif isinstance(value, float):
            self.tag(b"F")
            self.payload(struct.pack("<d", value))
        elif isinstance(value, dict):
            self.dict(value)
        elif hasattr(value, "__len__") and hasattr(value, "__iter__"):
            self.tag(b"L")
            self.hash.update(len(value).to_bytes(8, "little"))
            for item in value:
                self.visit(item)
        else:
            raise ValueError(f"cannot measure raw data of type {type(value).__name__}")

    def dict(self, value):
        if not all(isinstance(key, str) for key in value):
            raise ValueError("cannot measure a dict with a key that is not a str")
        self.tag(b"D")
        self.hash.update(len(value).to_bytes(8, "little"))
        for key in sorted(value):
            self.payload(key.encode("utf-8"))
            self.visit(value[key])


def describe(raw):
    """(payload bytes, fingerprint hex string) of one raw value, in the domain above."""
    walk = _Walk()
    walk.visit(raw)
    return walk.nbytes, walk.hash.hexdigest()


def poll_stats(kind, raw):
    """Size and fingerprints of a non-empty poll. A sensor that buffers readings (GPS, IMU) gets
    one fingerprint per reading in the poll, split as `readings.readings_in` does; any other
    sensor gets one for the whole poll. Raises ValueError outside the domain above."""
    if kind not in BUFFERED_KINDS:
        nbytes, key = describe(raw)
        return PollStats(nbytes, (key,))
    described = [describe(reading) for reading in readings_in(raw)]
    return PollStats(sum(nbytes for nbytes, _ in described), tuple(key for _, key in described))


def to_plain(value):
    """A JSON-safe copy of one raw reading holding numbers, names and structure only. Booleans,
    None and strings are kept. A float that is not finite becomes the string "nan", "inf" or
    "-inf". An object with `tolist()` (a numpy array) is converted through it. A dict key is
    written as text. Anything else, bytes included, becomes the text "<unsupported TypeName>"
    so that nothing but numbers and names reaches the file."""
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, dict):
        return {str(key): to_plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    if hasattr(value, "tolist"):
        return to_plain(value.tolist())
    return f"<unsupported {type(value).__name__}>"
