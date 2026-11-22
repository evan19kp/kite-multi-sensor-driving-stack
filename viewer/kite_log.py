"""Reads a v0 log (`docs/LOG-FORMAT.md`) with the standard library only, under the rules of the C++
`read_log`: the exact header, `\\n` line ends, the field count of each record type, numbers in the
writer's canonical form and finite doubles. Every error is a LogError that names the file line.
"""

import decimal
import math
import re
from pathlib import Path
from typing import NamedTuple

HEADER = "# kite-log v0"
INT_MIN, INT_MAX = -(2 ** 31), 2 ** 31 - 1
_INT = re.compile(r"-?[0-9]+", re.ASCII)
_DOUBLE = re.compile(r"-?[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?", re.ASCII)


class LogError(ValueError):
    pass


class Gt(NamedTuple):
    t: float
    ego_id: int
    actor_id: int
    x: float
    y: float
    vx: float
    vy: float


class Meas(NamedTuple):
    t: float
    ego_id: int
    sensor_id: int
    x: float
    y: float


class Track(NamedTuple):
    t: float
    ego_id: int
    track_id: int
    x: float
    y: float
    vx: float
    vy: float


class Log(NamedTuple):
    gt: list
    meas: list
    tracks: list


D, I = "double", "int"
RECORDS = {
    "gt": (Gt, (D, I, I, D, D, D, D)),
    "meas": (Meas, (D, I, I, D, D)),
    "track": (Track, (D, I, I, D, D, D, D)),
}


def canonical_double(value):
    """The text `std::to_chars` writes for the finite double `value`: its shortest round-trip
    digits, in plain notation or in scientific notation (`1e+05`), whichever is shorter, plain
    on a tie."""
    if value == 0:
        return "-0" if math.copysign(1.0, value) < 0 else "0"
    sign, digit_tuple, exponent = decimal.Decimal(repr(value)).as_tuple()
    digits = "".join(map(str, digit_tuple)).rstrip("0")
    exponent += len(digit_tuple) - len(digits)
    point = len(digits) + exponent
    if point >= len(digits):
        plain = digits + "0" * (point - len(digits))
    elif point > 0:
        plain = digits[:point] + "." + digits[point:]
    else:
        plain = "0." + "0" * -point + digits
    sci_exponent = point - 1
    scientific = (digits[0] + ("." + digits[1:] if len(digits) > 1 else "")
                  + f"e{'-' if sci_exponent < 0 else '+'}{abs(sci_exponent):02d}")
    text = plain if len(plain) <= len(scientific) else scientific
    return "-" + text if sign else text


def _parse_number(text, kind):
    if kind == I:
        if not _INT.fullmatch(text):
            raise ValueError(f"not an integer: {text!r}")
        value = int(text)
        if not INT_MIN <= value <= INT_MAX:
            raise ValueError(f"integer out of range: {text!r}")
        if str(value) != text:
            raise ValueError(f"integer not in canonical form: {text!r}")
        return value
    if not _DOUBLE.fullmatch(text):
        raise ValueError(f"not a number: {text!r}")
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(f"number not finite or out of range: {text!r}")
    if canonical_double(value) != text:
        raise ValueError(f"number not in canonical form: {text!r} (the writer writes {canonical_double(value)!r})")
    return value


def parse_log(data):
    """The Log in `data`, the bytes of a v0 log. Raises LogError naming the first bad line."""
    if not data:
        raise LogError("line 1: header missing, the file is empty")
    lines = data.split(b"\n")
    last = lines.pop()
    cut_off = last != b""
    if cut_off:
        lines.append(last)
    log = Log([], [], [])
    for number, raw in enumerate(lines, start=1):
        if cut_off and number == len(lines):
            raise LogError(f"line {number}: line does not end with a newline (the file is cut off)")
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as error:
            raise LogError(f"line {number}: not valid UTF-8 ({error.reason})") from None
        if number == 1:
            if text != HEADER:
                raise LogError(f"line 1: header is not exactly {HEADER!r}")
        else:
            _add_record(log, text, number)
    return log


def _add_record(log, text, number):
    if "\r" in text:
        raise LogError(f"line {number}: carriage return; lines end with \\n only")
    fields = text.split(",")
    if fields[0] not in RECORDS:
        raise LogError(f"line {number}: unknown record type {fields[0]!r}")
    record_type, kinds = RECORDS[fields[0]]
    if len(fields) != len(kinds) + 1:
        raise LogError(f"line {number}: a {fields[0]} record has {len(kinds)} fields after its type, found {len(fields) - 1}")
    try:
        values = [_parse_number(field, kind) for field, kind in zip(fields[1:], kinds)]
    except ValueError as error:
        raise LogError(f"line {number}: {error}") from None
    {"gt": log.gt, "meas": log.meas, "track": log.tracks}[fields[0]].append(record_type(*values))


def read_log(path):
    """The Log in the file `path`. Raises LogError for bad content, OSError when the file cannot
    be read."""
    return parse_log(Path(path).read_bytes())
