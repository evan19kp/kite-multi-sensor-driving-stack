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

In free mode (`--mode free`) the simulator runs on its own and the same rates are polls per
wall second, on a wall-clock schedule: a sensor at rate r is due at k / r wall seconds after
the run starts, k = 0, 1, 2, ... A pass at wall time t polls every sensor that is due (its due
time is at most t), once, and moves its due time to the first k / r after t (`next_due`). A
poll that comes late therefore covers its own slot and the slots it missed are skipped, never
made up in a burst. With polls that take no time a run of W wall seconds gives ceil(W * r)
polls at times below W. The wall time t is a finite number of seconds >= 0.

`parse_rates` is the boundary: it turns the `--rates` text into a dict by sensor kind and
raises ValueError for anything outside the domain.
"""

import math
from fractions import Fraction

from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ

DEFAULT_RATES = {"lidar": 20, "camera": 20, "radar": 20, "gps": 10, "imu": 10}
DEFAULT_RATES_TEXT = ",".join(f"{kind}={rate}" for kind, rate in DEFAULT_RATES.items())
ALL = "all"


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


def parse_rates(text):
    """`kind=rate,...` to {kind: Fraction}. Kinds are the sensor names and `all`. Sensors not
    named keep their default from DEFAULT_RATES_TEXT, or `all`'s rate if `all` is given. `all`
    is the base and a named sensor overrides it wherever it appears in the text. A kind given
    twice, an unknown kind, a pair without exactly one `=` and an empty text raise ValueError."""
    given = {}
    for pair in text.split(","):
        parts = pair.split("=")
        if len(parts) != 2:
            raise ValueError(f"expected kind=rate, got {pair!r}")
        name = parts[0].strip()
        if name != ALL and name not in SENSOR_KINDS:
            raise ValueError(f"unknown sensor {name!r}; use one of {', '.join(SENSOR_KINDS)} or {ALL}")
        if name in given:
            raise ValueError(f"{name} is given twice")
        given[name] = _rate(name, parts[1])
    base = {kind: given.get(ALL, Fraction(DEFAULT_RATES[kind])) for kind in SENSOR_KINDS}
    return {kind: given.get(kind, base[kind]) for kind in SENSOR_KINDS}


def every_step():
    return dict.fromkeys(SENSOR_KINDS, Fraction(STEP_HZ))


def polled(step, rate):
    """Whether a sensor at `rate` is polled after step number `step` (1 is the first step)."""
    return step == 1 or _slot(step, rate) > _slot(step - 1, rate)


def next_due(now, rate):
    """The first wall time k / rate strictly after `now` (free mode), as an exact Fraction."""
    return (math.floor(Fraction(now) * rate) + 1) / rate


def _slot(step, rate):
    return step * rate.numerator // (STEP_HZ * rate.denominator)
