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


def count_new(keys):
    """Number of polls whose key differs from the previous non-empty poll's. `keys` has one
    fingerprint per poll, in order, and None for an empty poll. The first non-empty poll is new.
    A key that comes back after a different one counts again."""
    previous = None
    new = 0
    for key in keys:
        if key is None:
            continue
        new += key != previous
        previous = key
    return new


def count_distinct(keys):
    """Number of different fingerprints among every reading a sensor returned, wherever in the
    run it came back."""
    return len(set(keys))


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
