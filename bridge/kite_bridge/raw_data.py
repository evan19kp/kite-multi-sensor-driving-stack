"""Size and fingerprint of the raw data a sensor poll returned, and a plain-data copy of one
reading for the raw dump. Standard library only: it never imports numpy or PIL, it calls
`tobytes()` on whatever has it.

Input domain, and nothing beyond it: a raw value is None, bytes, bytearray, str, bool, int
(within 64 bits), float, a dict whose keys and values are in this domain, a list, a tuple, an
object with `tobytes()` (a numpy array or scalar, a PIL image), or an object with `__len__` and
`__iter__` whose items are in this domain. Anything else raises ValueError. A raw value is never
truth-tested (R28).

This covers every poll shape that `readings.py` reads, including a dict of readings keyed by int,
float, bool, None, bytes or a tuple. Three kinds of poll that `readings.py` reads are left
outside, so the measuring loop counts them under `unexp`: one holding a value of another type
anywhere in it (a custom object, a complex number), one holding an int beyond 64 bits, and an
object with `__len__` but neither `__iter__` nor `tobytes()`. The rule below gives no size for
them without guessing.

How bytes are counted, here and nowhere else. The count is the payload the bridge received, not
what crossed the wire:

- bytes and bytearray: their length.
- an object with `tobytes()`: the length of what it returns. It is called once per object.
- bool: 1. int: 8 (a signed 64-bit value). float: 8 (a 64-bit float).
- str: the length of its UTF-8 encoding.
- None: 0.
- a dict key: counted like a value of its type, so a str key its UTF-8 length, an int or float
  key 8, a bool key 1, a None key 0 and a tuple key the sum of its items.
- a dict, list, tuple or other iterable: the sum of its items (for a dict, its keys and values).
  Brackets, lengths and type tags that go into the fingerprint are not counted.

The fingerprint is a hash of the same content with type tags and lengths added, so the int key 1,
the float key 1.0, the bool key True and the str key "1" are four different keys. Two raw values
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
        pairs = sorted(describe(pair) for pair in value.items())
        self.tag(b"D")
        self.hash.update(len(pairs).to_bytes(8, "little"))
        for nbytes, fingerprint in pairs:
            self.hash.update(bytes.fromhex(fingerprint))
            self.nbytes += nbytes


def describe(raw):
    """(payload bytes, fingerprint hex string) of one raw value, in the domain above."""
    walk = _Walk()
    walk.visit(raw)
    return walk.nbytes, walk.hash.hexdigest()


def poll_stats(kind, raw):
    """Size and fingerprints of a non-empty poll. The size is always that of the whole poll, by
    the rule above, so a dict of readings counts its outer keys too. A sensor that buffers
    readings (GPS, IMU) gets one fingerprint per reading in the poll, split as
    `readings.readings_in` does; any other sensor gets one for the whole poll. Raises ValueError
    outside the domain above."""
    nbytes, key = describe(raw)
    if kind not in BUFFERED_KINDS:
        return PollStats(nbytes, (key,))
    return PollStats(nbytes, tuple(describe(reading)[1] for reading in readings_in(raw)))


def to_plain(value):
    """A JSON-safe copy of one raw reading holding numbers, names and structure only. Booleans,
    None and strings are kept. A float that is not finite becomes the string "nan", "inf" or
    "-inf". An object with `tolist()` (a numpy array) is converted through it. JSON object keys
    are strings, so a dict key is written as its text, `str(key)`, as `readings.py` names keys:
    1 becomes "1", 2.5 "2.5", True "True", None "None". Two keys of one dict with the same text,
    such as 1 and "1", keep only the later one. Anything else, bytes included, becomes the text
    "<unsupported TypeName>" so that nothing but numbers and names reaches the file."""
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
