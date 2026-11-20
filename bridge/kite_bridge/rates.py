"""How often each sensor is polled, in polls per simulated second (lock-step mode) or per wall
second (free mode).

Input domain, and nothing beyond it: a rate is a positive finite number of polls per simulated
second, at most STEP_HZ (the step rate, so at most one poll per step). Rates are exact rationals
(`Fraction`), so which steps are polled never depends on floating-point rounding.

The schedule: with N steps taken, a sensor at rate r is polled after step 1 and after every
step k where floor(k * r / STEP_HZ) is greater than floor((k - 1) * r / STEP_HZ). Polling after
step 1 means the first reading is picked up at once, and for a buffered sensor nothing made
before the first poll is missed. The poll count is floor(N * r / STEP_HZ) + 1 for r below
STEP_HZ, and N for r = STEP_HZ (every step). Example, r = 20: steps 1, 3, 6, 9, ..., which is
21 polls in 60 steps.

In free mode (`--mode free`) the simulator runs on its own and the rates are reads per wall
second, on a wall-clock schedule: a sensor at rate r is due at k / r wall seconds after the run
starts, k = 0, 1, 2, ... The loop works in passes. A pass at wall time t (`pass_reads`) reads
every due streamed sensor (its due time is at most t), then at most one due slow poll: a
request to the simulator, which is any sensor that is not streamed and the Timer. Of the due
slow polls it takes the most overdue (the earliest due time; on a tie, the first in the order
given). The loop keeps that order as the Timer, then the sensors least recently read first, so
the sensors take turns and a run ends at its Timer reading before a sensor due at the same time. A slow poll costs about 20 ms on the real simulator, so a due streamed read waits for
at most the one slow poll already running. A read at wall time a moves its sensor's due time to
the first k / r after a (`next_due`). A read that comes late therefore covers its own due time;
the due times it passed over are skipped (`skipped` counts them), never made up in a burst. With
reads that take no time a run of W wall seconds gives ceil(W * r) reads at times below W. A read
is late when it starts LATE_S or more after its due time. LATE_S is 2 ms: in three 2-second free
runs on the fake (real clock, developer machine) the worst start after a due time with no slow
poll in the way was 1.3 ms of sleep wake-up delay, and one slow poll is about 20 ms. Wall times
are finite seconds >= 0.

`parse_rates` is the boundary: it turns the `--rates` text into a dict by sensor kind and
raises ValueError for anything outside the domain.
"""

import math
from fractions import Fraction

from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ

DEFAULT_RATES = {"lidar": 20, "camera": 20, "radar": 20, "gps": 10, "imu": 10}
FREE_DEFAULT_RATES = {"lidar": 20, "camera": 40, "radar": 10, "gps": 10, "imu": 10}
DEFAULT_RATES_TEXT = ",".join(f"{kind}={rate}" for kind, rate in DEFAULT_RATES.items())
FREE_DEFAULT_RATES_TEXT = ",".join(f"{kind}={rate}" for kind, rate in FREE_DEFAULT_RATES.items())
ALL = "all"
LATE_S = Fraction(2, 1000)


def _rate(name, text):
    try:
        approx = float(text)
    except ValueError:
        raise ValueError(f"{name}: not a number: {text!r}") from None
    out_of_domain = ValueError(f"{name}: must be a number above 0 and at most {STEP_HZ}, got {text!r}")
    if not math.isfinite(approx) or approx <= 0:
        raise out_of_domain
    exact = Fraction(text.strip())
    if exact > STEP_HZ:
        raise out_of_domain
    return exact


def parse_rates(text, defaults=DEFAULT_RATES):
    """`kind=rate,...` to {kind: Fraction}. Kinds are the sensor names and `all`. Sensors not
    named keep their rate in `defaults` (DEFAULT_RATES, or FREE_DEFAULT_RATES in free mode), or
    `all`'s rate if `all` is given; None is no text, so every sensor keeps its default. `all`
    is the base and a named sensor overrides it wherever it appears in the text. A kind given
    twice, an unknown kind, a pair without exactly one `=` and an empty text raise ValueError."""
    given = {}
    for pair in [] if text is None else text.split(","):
        parts = pair.split("=")
        if len(parts) != 2:
            raise ValueError(f"expected kind=rate, got {pair!r}")
        name = parts[0].strip()
        if name != ALL and name not in SENSOR_KINDS:
            raise ValueError(f"unknown sensor {name!r}; use one of {', '.join(SENSOR_KINDS)} or {ALL}")
        if name in given:
            raise ValueError(f"{name} is given twice")
        given[name] = _rate(name, parts[1])
    base = {kind: given.get(ALL, Fraction(defaults[kind])) for kind in SENSOR_KINDS}
    return {kind: given.get(kind, base[kind]) for kind in SENSOR_KINDS}


def every_step():
    return dict.fromkeys(SENSOR_KINDS, Fraction(STEP_HZ))


def polled(step, rate):
    """Whether a sensor at `rate` is polled after step number `step` (1 is the first step)."""
    return step == 1 or _slot(step, rate) > _slot(step - 1, rate)


def next_due(now, rate):
    """The first wall time k / rate strictly after `now` (free mode), as an exact Fraction."""
    return (math.floor(Fraction(now) * rate) + 1) / rate


def pass_reads(now, due, streamed, slow):
    """The keys one free-mode pass at wall time `now` reads, in order: every key of `streamed`
    whose due time in `due` is at most `now`, then the most overdue such key of `slow`, if any."""
    waiting = [key for key in slow if due[key] <= now]
    most_overdue = [min(waiting, key=due.__getitem__)] if waiting else []
    return [key for key in streamed if due[key] <= now] + most_overdue


def skipped(due, at, rate):
    """How many due times k / rate a read at wall time `at` passed over: those after `due`, the
    due time it served, and at most `at`. At the end of a run, `at` is the start of the last pass
    and `due` the sensor's next due time: what a read then would have passed over (0 when `due`
    is after `at`)."""
    return max(0, math.floor(Fraction(at) * rate) - int(due * rate))


def _slot(step, rate):
    return step * rate.numerator // (STEP_HZ * rate.denominator)
