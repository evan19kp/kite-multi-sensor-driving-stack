import argparse
import contextlib
import dataclasses
import json
import math
import sys
import threading
import time
import traceback
import typing
from fractions import Fraction
from pathlib import Path

from kite_bridge import measure, raw_data, rates
from kite_bridge.fake_sim import FakeSim
from kite_bridge.output_guard import PROVENANCE, out_refusal, repo_roots
from kite_bridge.readings import readings_in
from kite_bridge.sensor_settings import requested_update_hz, settings_lines
from kite_bridge.sim_api import (BUFFERED_KINDS, POLL_TIME, SENSOR_KINDS, STEP_HZ, STREAMED_KINDS, TIMER_TIME,
                                 Reading)

RAW_DUMP_FILE = "raw_gps_imu.json"
DEFAULT_MODEL = "vivace"
DEFAULT_CONFIG = "vehicles/vivace/tograc_qE.pc"
TECH_VERSION_TEXT = "not reported by BeamNGpy 1.36"
MIXED_TIME = "mixed"
DEFAULT_TIMEOUT_S = 300.0
CLOSE_WAIT_S = 20.0  # BeamNGpy's close() waits up to 10 s for the simulator to quit, then ends it itself
JOIN_SLICE_S = 0.1
EXIT_INTERRUPTED = 130
LOCKSTEP = "lockstep"
FREE = "free"
WALL_LIMIT_FACTOR = 20
MIN_SLEEP_S = 1e-6
wall_clock = time.perf_counter
wall_sleep = time.sleep


class SpikeError(Exception):
    pass


class Stopped(Exception):
    """Ctrl+C or a simulator timeout stopped the run, and the session has shut down. `ended` is
    False when BeamNG.tech could not be ended."""

    def __init__(self, cause, ended):
        super().__init__(cause)
        self.interrupted = isinstance(cause, KeyboardInterrupt)
        self.ended = ended


def _cut_short(error):
    """Ctrl+C or a timeout can stop a simulator call before its reply is read. The connection is
    then out of step, and any further request may wait for a reply that never comes."""
    return isinstance(error, (KeyboardInterrupt, TimeoutError))


def _positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError(f"must be a number > 0, got {text}")
    return value


def _count(text):
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError(f"must be >= 0, got {text}")
    return value


def _rates_option(text):
    try:
        return rates.parse_rates(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from None


def add_measure_options(parser):
    parser.add_argument("--rates", type=_rates_option, default=rates.DEFAULT_RATES_TEXT,
                        metavar="KIND=RATE,...",
                        help="polls per simulated second for each sensor (lidar, camera, radar, gps, imu), "
                             "or per wall second with --mode free, "
                             f"each above 0 and at most {STEP_HZ}; all=RATE sets every sensor and a named "
                             f"sensor overrides it (default {rates.DEFAULT_RATES_TEXT}; all={STEP_HZ} polls "
                             "every sensor after every step)")
    parser.add_argument("--mode", choices=(LOCKSTEP, FREE), default=LOCKSTEP,
                        help=f"{LOCKSTEP} (default): step the simulator {STEP_HZ} times per simulated second "
                             f"and poll between steps; {FREE}: let the simulator run on its own, poll each "
                             "sensor at its --rates rate per wall second and read simulated time from a Timer "
                             "sensor; the run ends at --seconds of Timer time, or fails after --seconds x "
                             f"{WALL_LIMIT_FACTOR} wall seconds")
    parser.add_argument("--dump-raw", type=_count, default=0, metavar="N",
                        help=f"write the first N raw GPS and the first N raw IMU readings of each car, "
                             f"with each poll's wall time and step count, to {RAW_DUMP_FILE} in --out "
                             "(default 0: write nothing)")
    parser.add_argument("--stream", action="store_true",
                        help="create the lidar and camera with is_streaming=True and read them from shared "
                             "memory with stream() instead of poll(); readings that fail the checks in "
                             "kite_bridge/stream_check.py are dropped and counted under suspect "
                             "(default off: every sensor is polled)")


def parse_args(description, argv=None, extra=None):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--home", help="the simulator's folder (required unless --fake)")
    parser.add_argument("--headless", action="store_true", help="run the simulator without a window")
    parser.add_argument("--seconds", type=_positive_float, default=30.0,
                        help="simulated seconds to run (default 30)")
    parser.add_argument("--traffic", type=_count, default=8,
                        help="AI traffic cars to spawn (default 8)")
    parser.add_argument("--out", default="recordings",
                        help="output folder (default recordings/); inside the repo only recordings/ "
                             "or scratch/ is allowed, a folder outside the repo is too")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help=f"our car's vehicle model (default {DEFAULT_MODEL})")
    parser.add_argument("--config", default=DEFAULT_CONFIG, metavar="PATH",
                        help=f"our car's part configuration file (default {DEFAULT_CONFIG}); "
                             "--config= spawns the model's default configuration")
    parser.add_argument("--list-models", action="store_true",
                        help="print the simulator's vehicle model names, sorted, and exit without spawning")
    parser.add_argument("--timeout", type=_positive_float, default=DEFAULT_TIMEOUT_S, metavar="SECONDS",
                        help=f"longest wait for any one simulator call, map loading included, before "
                             f"the run stops as timed out (default {DEFAULT_TIMEOUT_S:g})")
    parser.add_argument("--fake", action="store_true", help="use the fake simulator")
    if extra:
        extra(parser)
    args = parser.parse_args(argv)
    if not args.fake and not args.home:
        parser.error("--home is required unless --fake is given")
    refusal = out_refusal(args.out, repo_roots())
    if refusal:
        parser.error(f"--out {refusal}")
    args.config = args.config or None
    return args


def list_models(args):
    with session(args) as sim:
        for name in sorted(sim.list_models()):
            print(name)


def guarded(run, args):
    try:
        (list_models if args.list_models else run)(args)
    except (Stopped, KeyboardInterrupt) as error:
        stopped = error if isinstance(error, Stopped) else Stopped(error, ended=True)
        state = "BeamNG.tech closed" if stopped.ended else "BeamNG.tech may still be running"
        if stopped.interrupted:
            print(f"interrupted: {state}", file=sys.stderr)
            return EXIT_INTERRUPTED
        print(f"timed out after {args.timeout:g} s: {state}", file=sys.stderr)
        return 1
    except SpikeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    except Exception as error:
        traceback.print_exc()
        print(f"error: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


@contextlib.contextmanager
def session(args):
    """Opens the simulator and always shuts it down through `_shut_down`. Ctrl+C or a timeout,
    in the run or during the shutdown, comes out as `Stopped`; any other failure is re-raised."""
    if args.fake:
        sim = FakeSim()
    else:
        if not Path(args.home).is_dir():
            raise SpikeError("--home is not a folder")
        from kite_bridge.beamng_adapter import BeamNGAdapter

        sim = BeamNGAdapter(args.home, args.headless, args.timeout)
    failure = None
    try:
        sim.open()
        yield sim
    except BaseException as error:
        failure = error
    ended, shutdown_failure = _shut_down(sim, not _cut_short(failure), CLOSE_WAIT_S)
    if failure is None:
        failure = shutdown_failure
    if _cut_short(failure):
        raise Stopped(failure, ended) from failure
    if failure is not None:
        raise failure


def _shut_down(sim, in_step, close_wait):
    """Ends BeamNG.tech without waiting forever. Only while the connection is `in_step`, pauses a
    free-running simulator and then removes the sensors; then closes. If close raises or overruns
    `close_wait`, or Ctrl+C comes during either, ends the process BeamNGpy launched.

    A pause that raises, a timeout included, is printed as one `warning: could not pause` line
    whatever the run's outcome, skips the removal and counts like a removal failure. Returns
    whether BeamNG.tech ended, and the close or Ctrl+C failure, else the pause or removal
    failure, else None."""
    removal_failure = None
    try:
        if in_step:
            try:
                if sim.free_running:
                    try:
                        sim.pause()
                    except Exception as error:
                        print(f"warning: could not pause: {type(error).__name__}: {error}; sensors not removed",
                              file=sys.stderr)
                        raise
                sim.remove_sensors()
            except Exception as error:
                removal_failure = error
        _close_within(sim, close_wait)
    except BaseException as error:
        return _end_process(sim, error), error
    return True, removal_failure


def _close_within(sim, wait):
    failures = []

    def close():
        try:
            sim.close()
        except BaseException as error:
            failures.append(error)

    closer = threading.Thread(target=close, daemon=True)
    closer.start()
    deadline = time.monotonic() + wait
    # Short joins: on Windows a Ctrl+C is not raised until a blocking join returns.
    while closer.is_alive() and time.monotonic() < deadline:
        closer.join(min(JOIN_SLICE_S, max(0.0, deadline - time.monotonic())))
    if closer.is_alive():
        raise SpikeError(f"close did not return within {wait:g} s")
    if failures:
        raise failures[0]


def _end_process(sim, reason):
    why = "Ctrl+C during shutdown" if isinstance(reason, KeyboardInterrupt) else _describe(reason)
    if sim.end_process():
        print(f"BeamNG.tech not closed ({why}); ended the process BeamNGpy launched", file=sys.stderr)
        return True
    print(f"BeamNG.tech not closed ({why}); BeamNGpy did not launch it, so nothing was ended: "
          "close BeamNG.tech by hand", file=sys.stderr)
    return False


def _describe(error):
    return str(error) if isinstance(error, SpikeError) else f"{type(error).__name__}: {error}"


def version_report(sim):
    return {**sim.versions(), "BeamNG.tech version": TECH_VERSION_TEXT}


def print_sensor_settings(stream=False):
    print("\n".join(settings_lines(stream)))


def spawn(sim, args, n_cars):
    cars = sim.load_scenario(args.model, n_cars, args.config)
    print(f"spawned {', '.join(cars)} (model: {args.model}, config: {args.config or 'model default'})")
    return cars


def start_traffic(sim, count):
    if count == 0:
        return False
    try:
        sim.spawn_traffic(count)
    except TimeoutError:
        raise
    except Exception as error:
        print(f"traffic not spawned ({type(error).__name__}: {error}); continuing with our car only")
        return False
    return True


def steps_for(seconds):
    return max(1, round(seconds * STEP_HZ))


def _time_source(sources):
    return MIXED_TIME if len(sources) > 1 else next(iter(sources), None)


class RawDump:
    """The first `per_sensor` raw GPS readings and the first `per_sensor` raw IMU readings of
    every car, each with its poll's wall time and step count. Written by `write_raw_dump`."""

    def __init__(self, per_sensor):
        self.per_sensor = per_sensor
        self.entries = []
        self._counts = {}

    def add(self, car, kind, step, wall_s, poll_wall_s, readings):
        """One entry per reading of the poll, until this car's sensor has `per_sensor` entries.
        `step` is the steps taken when the poll was made (None in free mode, which takes no
        steps), `wall_s` the wall seconds since the run's loop started when the poll returned and
        `poll_wall_s` how long the poll call took."""
        for reading in readings:
            if self._counts.get((car, kind), 0) >= self.per_sensor:
                return
            self._counts[car, kind] = self._counts.get((car, kind), 0) + 1
            self.entries.append({"car": car, "sensor": kind, "step": step, "wall_s": wall_s,
                                 "poll_wall_s": poll_wall_s, "reading": raw_data.to_plain(reading)})


def write_raw_dump(out, per_sensor, cases):
    """`cases` is a list of (car count, RawDump). Written as raw_gps_imu.json."""
    write_json(out, RAW_DUMP_FILE, {
        "readings_per_car_and_sensor": per_sensor,
        "cases": [{"cars": cars, "entries": dump.entries} for cars, dump in cases]})


def measure_sensors(sim, cars, seconds, clock=None, dump=None, poll_rates=None, stream=False, mode=LOCKSTEP,
                    sleep=None):
    """Steps the simulator for `seconds` of simulator time and polls each sensor at its rate in
    `poll_rates`, a dict by sensor kind of polls per simulated second (see `rates` for the
    schedule). By default every sensor is polled once after each step, STEP_HZ per simulator
    second. `clock` returns wall seconds and `sleep` waits for wall seconds; a test passes fake
    ones. Without them the run uses `wall_clock` and `wall_sleep`, real time. `dump`, a
    RawDump, collects the raw GPS and IMU readings.

    `mode` FREE replaces the steps. The simulator runs on its own and each sensor is polled on
    the wall-clock schedule in `rates`, so the rates are polls per wall second. The Timer, on
    the first car, is polled in its own request at the higher of the GPS and IMU rates. The run
    ends at the first Timer reading `seconds` past its first one, and raises SpikeError after
    `seconds` x WALL_LIMIT_FACTOR wall seconds, when the Timer goes back, or when a Timer
    reading is not finite seconds >= 0. Unlike a sensor reading outside its input domain, which
    counts under `unexp`, a bad Timer reading stops the run, because the Timer decides when it
    ends. `simulated_s` is
    how far the Timer advanced. A reading without its own time gets the latest Timer reading,
    with `time_source` TIMER_TIME. The timing has `idle_s` (sleeping until the next poll is due) and `timer_reads_s` (inside the
    Timer polls) in place of `step_wait_s`; `steps` and `steps_per_second` are None, a dump
    entry's step is None, and `timer` gives the Timer's poll count and rate. Every sensor
    counts as polled slower than every step for `possible_loss`. `mode` is in the result.

    With `stream`, the STREAMED_KINDS sensors (lidar, camera) are attached streaming and read
    with `sim.stream` instead of `sim.poll`, on the same schedule; everything below that says
    poll covers those reads too, except the timing split. Their time goes to `stream_reads_s`,
    not `sensor_reads_s`. It covers everything inside the adapter's stream call: BeamNGpy's
    `stream()`, which copies shared memory and sends no request, the checks in `stream_check`
    and the parse in `readings.py`. Each row says how it was read (`read`) and what was dropped
    (`suspect`: whole readings dropped, the first reason, and all-zero rows cut off kept lidar
    scans, summed over reads; None for a polled sensor). A dropped reading is left out of the
    same counts as an unexpected one. Without `stream` the result has none of these fields
    and every sensor is polled; `stream` in the result says which it was.

    A row's `configured_rate_hz` is the poll rate and `requested_update_hz` the update rate the
    adapter asks of the sensor (None for GPS and IMU). `possible_loss` is True only when the
    sensor is polled slower than every step, has a requested rate, and gave fewer new readings
    than it (see `measure.possible_loss`). For GPS and IMU it is always False: their buffer size
    is not known, so one run cannot say; compare `new_readings` with a run at every step.

    The wall time is the stepping and polling loop only, not the scenario load or the sensor
    set-up. `timing` splits it into `step_wait_s` (inside `sim.step`), `sensor_reads_s` and
    `processing_s`. `sensor_reads_s` is everything inside the adapter's poll call: BeamNGpy's
    poll, however it fetches and decodes the data, plus the parse in `readings.py`. It is more
    than the round trip to the simulator. `processing_s` is the rest: our own work between
    those calls, which includes sizing and fingerprinting each poll's raw data. `real_time_factor`
    is simulated seconds over wall seconds.

    A row's `rate_hz` is the rate of distinct reading timestamps seen at poll time. It counts
    at most one new reading per poll, so it is limited to about the poll rate.
    `readings_per_poll_max` is the most readings one poll returned; above 1, the sensor
    produced readings faster than it was polled. `new_readings` counts readings whose content
    is new: a lidar, camera or radar poll that differs from the previous non-empty poll, and
    for GPS and IMU every distinct reading returned, by its time and values. `bytes_read` is
    the raw payload size of the polls (see `raw_data`). A poll that raises counts as an
    unexpected reading, with the error as its message, and the run goes on, except for a
    timeout, which stops the run. Unexpected readings are left out of `polls`, `empty`,
    `rate_hz`, `readings_per_poll_max`, `new_readings` and `bytes_read`; the latency and read
    time figures include their polls."""
    clock = clock or wall_clock
    poll_rates = rates.every_step() if poll_rates is None else poll_rates
    for car in cars:
        sim.attach_sensors(car, stream=stream)
    keys = [(car, kind) for car in cars for kind in SENSOR_KINDS]
    streamed = {key for key in keys if stream and key[1] in STREAMED_KINDS}
    tally = _Tally(keys, streamed, dump)
    if mode == FREE:
        run = _run_free(sim, tally, cars[0], seconds, poll_rates, clock, sleep or wall_sleep)
    else:
        run = _run_lockstep(sim, tally, seconds, poll_rates, clock)
    steps, simulated, wall, waits = run.steps, run.simulated_s, run.wall_s, run.waits
    suspect, suspect_rows, series, latencies = tally.suspect, tally.suspect_rows, tally.series, tally.latencies
    points, detections, sources, unexpected = tally.points, tally.detections, tally.sources, tally.unexpected
    per_poll, sizes, poll_keys, reading_keys = tally.per_poll, tally.sizes, tally.poll_keys, tally.reading_keys
    reads = math.fsum(math.fsum(latencies[key]) for key in keys if key not in streamed)
    stream_reads = math.fsum(math.fsum(latencies[key]) for key in streamed)
    rows = []
    for car, kind in keys:
        new = (measure.count_distinct(reading_keys[car, kind]) if kind in BUFFERED_KINDS
               else measure.count_new(poll_keys[car, kind]))
        read_total = math.fsum(latencies[car, kind])
        requested = requested_update_hz(kind)
        rows.append({
            "car": car, "sensor": kind,
            "configured_rate_hz": float(poll_rates[kind]),
            "requested_update_hz": requested,
            "possible_loss": (_can_miss_updates(mode, poll_rates[kind]) and requested is not None
                              and measure.possible_loss(new, simulated, requested)),
            **measure.summarise(series[car, kind], latencies[car, kind], points[car, kind],
                                detections[car, kind]),
            "time_source": _time_source(sources[car, kind]),
            "readings_per_poll_max": max(per_poll[car, kind], default=None),
            "unexpected": {"count": len(unexpected[car, kind]),
                           "first": unexpected[car, kind][0] if unexpected[car, kind] else None},
            "read_total_s": read_total,
            "read_per_poll_s": read_total / len(latencies[car, kind]),
            "new_readings": new,
            "new_per_simulated_s": measure.per_simulated_second(new, simulated),
            "bytes_read": sizes[car, kind],
            "bytes_per_simulated_s": measure.per_simulated_second(sizes[car, kind], simulated)})
        if stream:
            dropped = suspect[car, kind]
            rows[-1]["read"] = "stream" if (car, kind) in streamed else "poll"
            rows[-1]["suspect"] = ({"readings": len(dropped), "first": dropped[0] if dropped else None,
                                    "rows": suspect_rows[car, kind]} if (car, kind) in streamed else None)
    timing = {"simulated_s": simulated, "wall_s": wall, "real_time_factor": measure.real_time_factor(simulated, wall),
              **waits, "sensor_reads_s": reads,
              **({"stream_reads_s": stream_reads} if stream else {}),
              "processing_s": measure.processing_time(wall, math.fsum(waits.values()), reads + stream_reads)}
    return {"cars": len(cars), "mode": mode, "steps": steps, "steps_per_second": run.steps_per_second,
            "stream": stream, **run.extra, "timing": timing, "sensors": rows}


def _can_miss_updates(mode, rate):
    return mode == FREE or rate < STEP_HZ


class _Run(typing.NamedTuple):
    steps: int | None
    steps_per_second: float | None
    simulated_s: float
    wall_s: float
    waits: dict
    extra: dict


def _run_lockstep(sim, tally, seconds, poll_rates, clock):
    steps = steps_for(seconds)
    step_time = 0.0
    loop_started = clock()
    for step_number in range(1, steps + 1):
        started = clock()
        sim.step(1)
        step_time += clock() - started
        for key in tally.keys:
            if rates.polled(step_number, poll_rates[key[1]]):
                tally.read(sim, key, clock, loop_started, step_number)
    wall = clock() - loop_started
    return _Run(steps, measure.steps_per_second(steps, step_time), steps / STEP_HZ, wall,
                {"step_wait_s": step_time}, {})


def _run_free(sim, tally, timer_car, seconds, poll_rates, clock, sleep):
    timer_rate = max(poll_rates["gps"], poll_rates["imu"])
    limit = Fraction(seconds) * WALL_LIMIT_FACTOR
    sim.start_free_running(timer_car)
    due = dict.fromkeys(tally.keys, 0)
    timer_due = 0
    idle = timer_time = 0.0
    timer_polls = 0
    first = latest = None
    loop_started = clock()
    while True:
        now = clock() - loop_started
        if now >= timer_due:
            started = clock()
            try:
                timer = sim.timer_seconds()
            except ValueError as error:
                raise SpikeError(f"free run: unusable Timer reading: {error}") from error
            timer_time += clock() - started
            timer_polls += 1
            if first is None:
                first = timer
            elif timer < latest:
                raise SpikeError(f"the Timer went back from {latest:g} s to {timer:g} s")
            latest = timer
            if latest - first >= seconds:
                break
            timer_due = rates.next_due(now, timer_rate)
        if now >= limit:
            raise SpikeError(f"free run: the Timer advanced {latest - first:.3f} of {seconds:g} simulated s "
                             f"in the wall-clock limit of {float(limit):g} s (--seconds x {WALL_LIMIT_FACTOR})")
        for key in tally.keys:
            if now >= due[key]:
                tally.read(sim, key, clock, loop_started, None, latest)
                due[key] = rates.next_due(now, poll_rates[key[1]])
        started = clock()
        gap = min(timer_due, limit, *due.values()) - Fraction(started - loop_started)
        if gap > 0:
            sleep(max(float(gap), MIN_SLEEP_S))
        idle += clock() - started
    wall = clock() - loop_started
    return _Run(None, None, latest - first, wall, {"idle_s": idle, "timer_reads_s": timer_time},
                {"timer": {"polls": timer_polls, "rate_hz": float(timer_rate)}})


class _Tally:
    def __init__(self, keys, streamed, dump):
        self.keys, self.streamed, self.dump = keys, streamed, dump
        self.suspect = {key: [] for key in keys}
        self.suspect_rows = {key: 0 for key in keys}
        self.series = {key: [] for key in keys}
        self.latencies = {key: [] for key in keys}
        self.points = {key: [] for key in keys}
        self.detections = {key: [] for key in keys}
        self.sources = {key: set() for key in keys}
        self.unexpected = {key: [] for key in keys}
        self.per_poll = {key: [] for key in keys}
        self.sizes = {key: 0 for key in keys}
        self.poll_keys = {key: [] for key in keys}
        self.reading_keys = {key: [] for key in keys}

    def read(self, sim, key, clock, loop_started, step, timer_s=None):
        method = "stream" if key in self.streamed else "poll"
        started = clock()
        try:
            reading = getattr(sim, method)(*key)
        except TimeoutError:
            raise
        except Exception as error:
            reading = Reading(None, unexpected=f"{method} raised {type(error).__name__}: {error}")
        finished = clock()
        self.latencies[key].append(finished - started)
        if reading.suspect:
            self.suspect[key].append(reading.suspect)
            return
        if timer_s is not None and reading.time_source == POLL_TIME:
            reading = dataclasses.replace(reading, timestamp=timer_s, time_source=TIMER_TIME)
        stats = raw_data.EMPTY
        if reading.timestamp is not None and not reading.unexpected:
            try:
                stats = raw_data.poll_stats(key[1], reading.raw)
            except ValueError as error:
                reading = Reading(None, unexpected=f"raw data not measurable: {error}")
        if reading.unexpected:
            self.unexpected[key].append(reading.unexpected)
            return
        self.suspect_rows[key] += reading.suspect_rows
        self.series[key].append(reading.timestamp)
        self.sizes[key] += stats.nbytes
        if key[1] in BUFFERED_KINDS:
            self.reading_keys[key].extend(stats.keys)
        else:
            self.poll_keys[key].append(stats.keys[0] if stats.keys else None)
        if reading.timestamp is not None:
            self.sources[key].add(reading.time_source)
            self.per_poll[key].append(reading.count)
            if self.dump is not None and key[1] in BUFFERED_KINDS:
                self.dump.add(key[0], key[1], step, finished - loop_started,
                              finished - started, readings_in(reading.raw))
        if reading.points is not None:
            self.points[key].append(reading.points)
        if reading.detections is not None:
            self.detections[key].append(reading.detections)


def _fmt(value, scale=1.0, digits=2):
    return "n/a" if value is None else f"{value * scale:.{digits}f}"


def _timing_lines(result):
    t = result["timing"]
    wall = t["wall_s"]

    def part(label, seconds):
        share = _fmt(seconds / wall * 100, digits=1) if wall > 0 else "n/a"
        return f"  {label:<18}{seconds:>9.3f} s{share:>7}%"

    reads = ([part("in sensor polls", t["sensor_reads_s"]), part("in stream reads", t["stream_reads_s"])]
             if result["stream"] else [part("in sensor reads", t["sensor_reads_s"])])
    free = result["mode"] == FREE
    waits = ([part("waiting to poll", t["idle_s"]), part("in Timer polls", t["timer_reads_s"])] if free
             else [part("waiting for steps", t["step_wait_s"])])
    lines = [f"timing: {_fmt(t['simulated_s'], digits=3)} simulated s{' (Timer)' if free else ''} in "
             f"{_fmt(wall, digits=3)} wall s, real-time factor {_fmt(t['real_time_factor'], digits=3)}",
             *waits, *reads, part("our processing", t["processing_s"]),
             f"{'car':<7}{'sensor':<8}{'read_s':>9}{'per_poll_ms':>13}{'poll_hz':>9}{'new':>7}"
             f"{'new_per_s':>11}{'req_hz':>8}{'bytes':>12}{'kB_per_s':>10}"]
    for r in result["sensors"]:
        lines.append(f"{r['car']:<7}{r['sensor']:<8}{r['read_total_s']:>9.3f}"
                     f"{_fmt(r['read_per_poll_s'], 1000):>13}{_fmt(r['configured_rate_hz']):>9}"
                     f"{r['new_readings']:>7}{_fmt(r['new_per_simulated_s']):>11}"
                     f"{_fmt(r['requested_update_hz']):>8}{r['bytes_read']:>12}"
                     f"{_fmt(r['bytes_per_simulated_s'], 0.001, 1):>10}")
    return lines


def _loss_lines(result):
    free = result["mode"] == FREE
    lines = []
    for r in result["sensors"]:
        if r["possible_loss"]:
            lines.append(f"possible loss: {r['car']} {r['sensor']} gave {r['new_per_simulated_s']:.2f} new "
                         f"readings per simulated second at {r['configured_rate_hz']:g} polls per "
                         f"{'wall ' if free else ''}second, "
                         f"fewer than the {r['requested_update_hz']:g} Hz requested. "
                         f"Compare with a run at --rates all={STEP_HZ}.")
    buffered = sorted({r["sensor"] for r in result["sensors"]
                       if r["sensor"] in BUFFERED_KINDS and _can_miss_updates(result["mode"], r["configured_rate_hz"])})
    if buffered:
        lines.append(f"loss not checkable for {', '.join(buffered)}: a buffered sensor's capacity is not "
                     f"known, so compare its new readings with a run at --rates all={STEP_HZ}.")
    return lines


def _stream_columns(r):
    s = r["suspect"]
    return f"{r['read']:>8}{s['readings'] if s else 'n/a':>9}{s['rows'] if s else 'n/a':>10}"


def _suspect_lines(result):
    lines = ["stream: lidar and camera were read from shared memory with stream(); radar, gps and imu "
             "with poll(). suspect counts streamed readings dropped whole, sus_rows the all-zero rows cut "
             "off the end of kept lidar scans. Rows left from an earlier scan cannot be detected."]
    for r in result["sensors"]:
        s = r["suspect"]
        if s and (s["readings"] or s["rows"]):
            reason = f", first reason: {s['first']}" if s["first"] else ""
            lines.append(f"suspect: {r['car']} {r['sensor']} dropped {s['readings']} readings and cut "
                         f"{s['rows']} all-zero rows{reason}")
    return lines


def _summary_line(result):
    if result["mode"] == FREE:
        timer = result["timer"]
        return (f"cars: {result['cars']}  mode: free, the Timer polled {timer['polls']} times at "
                f"{timer['rate_hz']:g} per wall second")
    return f"cars: {result['cars']}  steps per second: {_fmt(result['steps_per_second'], digits=1)}"


def _free_lines(result):
    if result["mode"] != FREE:
        return []
    return ["mode free: the simulator ran on its own and was polled on a wall-clock schedule. poll_hz is "
            "polls per wall second. simulated s, rate_hz, new_per_s and kB_per_s use Timer seconds. Where "
            "time is timer, the readings had no time of their own and got the latest Timer reading, so "
            "rate_hz and repeat there follow how often the Timer was polled; new_per_s does not. The "
            "real-time factor is what this run measured, not a target."]


def format_result(result):
    stream = result["stream"]
    lines = [_summary_line(result),
             f"{'car':<7}{'sensor':<8}{'polls':>6}{'rate_hz':>9}{'time':>7}{'per_poll':>10}{'mean_ms':>9}"
             f"{'p50_ms':>8}{'p95_ms':>8}{'max_ms':>8}{'empty':>7}{'repeat':>8}{'unexp':>7}{'points':>8}"
             f"{'detect':>8}" + (f"{'read':>8}{'suspect':>9}{'sus_rows':>10}" if stream else "")]
    for r in result["sensors"]:
        lat = r["latency_s"]
        lines.append(f"{r['car']:<7}{r['sensor']:<8}{r['polls']:>6}{_fmt(r['rate_hz']):>9}"
                     f"{r['time_source'] or 'n/a':>7}{r['readings_per_poll_max'] or 'n/a':>10}"
                     f"{_fmt(lat['mean'], 1000):>9}{_fmt(lat['p50'], 1000):>8}{_fmt(lat['p95'], 1000):>8}"
                     f"{_fmt(lat['max'], 1000):>8}{r['empty']:>7}{r['repeated']:>8}"
                     f"{r['unexpected']['count']:>7}{_fmt(r['points_mean'], digits=0):>8}"
                     f"{_fmt(r['detections_mean'], digits=0):>8}" + (_stream_columns(r) if stream else ""))
    lines += _timing_lines(result)
    if stream:
        lines += _suspect_lines(result)
    lines.append("rate_hz: distinct reading timestamps per simulator second. It counts at most one "
                 f"new reading per poll, so it is limited to about the sensor's poll rate (poll_hz, "
                 f"at most {STEP_HZ}). per_poll shows when a sensor delivered more.")
    lines += _free_lines(result)
    lines += _loss_lines(result)
    for r in result["sensors"]:
        if (r["readings_per_poll_max"] or 0) > 1:
            lines.append(f"{r['car']} {r['sensor']}: polls returned up to {r['readings_per_poll_max']} "
                         "readings. rate_hz counts at most one per poll, so it undercounts this sensor.")
    from_poll = [f"{r['car']} {r['sensor']}" for r in result["sensors"]
                 if r["time_source"] in (POLL_TIME, TIMER_TIME, MIXED_TIME)]
    if from_poll:
        lines.append(f"no sensor time for {', '.join(from_poll)}: rate_hz there is the "
                     "polling rate, not the sensor's")
    for r in result["sensors"]:
        if r["unexpected"]["count"]:
            lines.append(f"unexpected format: {r['car']} {r['sensor']} {r['unexpected']['count']} readings, "
                         f"first: {r['unexpected']['first']}")
    return "\n".join(lines)


def write_json(out, name, data):
    folder = Path(out)
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / name
    path.write_text(json.dumps({"provenance": PROVENANCE, **data}, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {path}")
