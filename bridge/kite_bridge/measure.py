"""Pure measurement maths for the simulator spikes.

Input domain, and nothing beyond it:

- Timestamps, latencies, wall, step-wait and read times and simulated seconds are finite floats
  in seconds, >= 0. Counts are finite numbers >= 0.
- A fingerprint is any hashable value; None marks an empty poll.
- A series of readings is a list of timestamps in which None marks an empty poll. The
  non-empty timestamps are non-decreasing. A timestamp equal to the previous non-empty
  one is a repeated reading.

Numeric input outside this domain raises ValueError (a non-number raises TypeError). Where a
statistic is undefined (no data, or a zero time span) the result is None.
"""

import math

ROUNDING_S = 1e-9


def _check(values, name, *, non_decreasing=False):
    previous = None
    for value in values:
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name}: expected a finite value >= 0, got {value!r}")
        if non_decreasing and previous is not None and value < previous:
            raise ValueError(f"{name}: values decrease ({previous!r} then {value!r})")
        previous = value


def _present(series):
    present = [value for value in series if value is not None]
    _check(present, "series", non_decreasing=True)
    return present


def count_empty(series):
    _present(series)
    return sum(1 for value in series if value is None)


def count_repeated(series):
    """Number of readings whose timestamp equals the previous non-empty reading's."""
    present = _present(series)
    return sum(1 for a, b in zip(present, present[1:]) if a == b)


def fresh_timestamps(series):
    present = _present(series)
    return [b for a, b in zip([None] + present, present) if a != b]


def effective_rate(timestamps):
    """(n - 1) / (last - first) in Hz for n >= 2 readings with last > first, else None."""
    _check(timestamps, "timestamps", non_decreasing=True)
    if len(timestamps) < 2 or timestamps[-1] <= timestamps[0]:
        return None
    return (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])


def percentile(values, q):
    """The q-th percentile (0 <= q <= 100), interpolating linearly between closest ranks.
    percentile([1, 2, 3, 4], 50) is 2.5. None for an empty list."""
    if not 0 <= q <= 100:
        raise ValueError(f"q must be in [0, 100], got {q!r}")
    _check(values, "values")
    if not values:
        return None
    ordered = sorted(values)
    rank = q / 100 * (len(ordered) - 1)
    low = math.floor(rank)
    high = math.ceil(rank)
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


def latency_stats(latencies):
    """Mean, p50, p95 and max of poll latencies in seconds. Every entry None when empty."""
    _check(latencies, "latencies")
    if not latencies:
        return {"mean": None, "p50": None, "p95": None, "max": None}
    return {
        "mean": math.fsum(latencies) / len(latencies),
        "p50": percentile(latencies, 50),
        "p95": percentile(latencies, 95),
        "max": max(latencies),
    }


def steps_per_second(steps, seconds):
    """Simulator steps per wall-clock second. None when seconds is 0."""
    _check([steps, seconds], "steps_per_second")
    return steps / seconds if seconds > 0 else None


def real_time_factor(simulated_s, wall_s):
    """Simulated seconds per wall-clock second. 1.0 is real time, 0.1 is ten times slower. None
    when wall_s is 0: nothing was timed, so the ratio is undefined."""
    _check([simulated_s, wall_s], "real_time_factor")
    return simulated_s / wall_s if wall_s > 0 else None


def per_simulated_second(count, simulated_s):
    """count / simulated_s. None when simulated_s is 0."""
    _check([count, simulated_s], "per_simulated_second")
    return count / simulated_s if simulated_s > 0 else None


def processing_time(wall_s, step_wait_s, reads_s):
    """Our own processing: the wall time that was neither waiting for steps nor in sensor reads.
    It is the remainder, so it also holds anything else that happened between the timed calls.
    The two parts cannot add up to more than the wall time; a total above it by up to 1e-9 s is
    rounding and gives 0.0, and anything more raises ValueError."""
    _check([wall_s, step_wait_s, reads_s], "processing_time")
    remainder = wall_s - step_wait_s - reads_s
    if remainder < -ROUNDING_S:
        raise ValueError(f"processing_time: parts add up to {step_wait_s + reads_s!r} s, "
                         f"more than the wall time {wall_s!r} s")
    return max(0.0, remainder)


class Freshness:
    """Says, one poll at a time, whether a poll's key differs from the previous non-empty poll's.
    None is an empty poll and is never new. The first non-empty poll is new. A key that comes back
    after a different one is new again."""

    def __init__(self):
        self._previous = None

    def is_new(self, key):
        if key is None:
            return False
        new = key != self._previous
        self._previous = key
        return new


def count_new(keys):
    """Number of new polls by `Freshness`. `keys` has one fingerprint per poll, in order, and
    None for an empty poll."""
    fresh = Freshness()
    return sum(fresh.is_new(key) for key in keys)


def count_distinct(keys):
    """Number of different fingerprints among every reading a sensor returned, wherever in the
    run it came back."""
    return len(set(keys))


def possible_loss(new_readings, simulated_s, requested_hz):
    """Whether a sensor asked to update `requested_hz` times per simulated second gave fewer new
    readings than that over `simulated_s`, by more than one. A run that polls every update sees
    between floor(requested_hz * simulated_s) and one more, so one reading of slack covers the
    partial first and last interval; a shortfall beyond it means updates were missed."""
    _check([new_readings, simulated_s, requested_hz], "possible_loss")
    return new_readings < requested_hz * simulated_s - 1


BUFFER_LOSS_SHARE = 0.02


def buffered_rate(polls):
    """New readings per simulated second of a buffered sensor between its first and last
    non-empty poll. `polls` has one (time, fingerprints) pair per non-empty poll, in order: the
    poll's latest reading's time and the fingerprint of every reading in it. The readings first
    returned after the first poll were made between the two polls, so they are counted over the
    time between them. None with fewer than two polls or no time between them."""
    if len(polls) < 2 or polls[-1][0] <= polls[0][0]:
        return None
    first = set(polls[0][1])
    later = {key for _, keys in polls[1:] for key in keys} - first
    return per_simulated_second(len(later), polls[-1][0] - polls[0][0])


def possible_buffer_loss(rate_hz, own_hz):
    """Whether a buffered sensor that makes `own_hz` readings per simulated second gave new
    readings at `rate_hz`, more than BUFFER_LOSS_SHARE (2 %) below it: a poll found the buffer
    full and readings were lost. False when either is None (not measured, or not known)."""
    if rate_hz is None or own_hz is None:
        return False
    _check([rate_hz, own_hz], "possible_buffer_loss")
    return rate_hz < own_hz * (1 - BUFFER_LOSS_SHARE)


def summarise(series, latencies, points=(), detections=()):
    return {
        "polls": len(series),
        "rate_hz": effective_rate(fresh_timestamps(series)),
        "latency_s": latency_stats(latencies),
        "empty": count_empty(series),
        "repeated": count_repeated(series),
        "points_mean": math.fsum(points) / len(points) if points else None,
        "detections_mean": math.fsum(detections) / len(detections) if detections else None,
    }
