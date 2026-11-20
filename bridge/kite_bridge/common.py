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

from kite_bridge import measure, raw_data, rates, rerun_view
from kite_bridge.fake_sim import FakeSim
from kite_bridge.output_guard import PROVENANCE, out_refusal, repo_roots
from kite_bridge.readings import readings_in
from kite_bridge.self_hits import DEFAULT_EGO_RADIUS_M, SelfHits
from kite_bridge.sensor_settings import DEFAULT_LIDAR_POS, requested_update_hz, settings_lines
from kite_bridge.sim_api import (BUFFERED_KINDS, POLL_TIME, SENSOR_KINDS, STEP_HZ, STREAMED_KINDS, TIMER,
                                 TIMER_TIME, Reading)

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
RATES_RULE = (f"for each sensor (lidar, camera, radar, gps, imu), each above 0 and at most {STEP_HZ}; all=RATE "
              "sets every sensor and a named sensor overrides it")
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


def _lidar_pos(text):
    try:
        values = tuple(float(part) for part in text.split(","))
    except ValueError:
        values = ()
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError(f"must be three finite numbers x,y,z in metres, got {text!r}")
    return tuple(int(value) if value.is_integer() else value for value in values)


def _add_lidar_options(parser):
    default = ",".join(f"{value:g}" for value in DEFAULT_LIDAR_POS)
    parser.add_argument("--lidar-pos", type=_lidar_pos, default=DEFAULT_LIDAR_POS, metavar="X,Y,Z",
                        help=f"where the lidar sits on our car, x,y,z in vehicle space in metres (default {default}); "
                             "write a negative value as --lidar-pos=-0.5,0,1.9")
    parser.add_argument("--ego-radius", type=_positive_float, metavar="METRES",
                        help=f"count a lidar point as a self-hit, approximately, when it is within this many metres "
                             f"of our car in x and y, measured from the GPS position (default {DEFAULT_EGO_RADIUS_M:g})")


def make_self_hits(args):
    """The self-hit counter for a run: a pure-Python count for the fake, one array operation per
    scan for the real simulator."""
    arrays = rerun_view.ListArrays() if args.fake else rerun_view.NumpyArrays()
    return SelfHits(args.ego_radius, args.lidar_pos, arrays)


def _rates_option(text):
    try:
        rates.parse_rates(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from None
    return text


def _add_rates_and_mode(parser, default_mode, rates_help):
    parser.add_argument("--rates", type=_rates_option, metavar="KIND=RATE,...", help=rates_help)
    other = FREE if default_mode == LOCKSTEP else LOCKSTEP
    parser.add_argument("--mode", choices=(LOCKSTEP, FREE), default=default_mode,
                        help=f"{default_mode} (default) or {other}. {LOCKSTEP}: step the simulator {STEP_HZ} "
                             f"times per simulated second and poll between steps; {FREE}: let the simulator "
                             "run on its own, read each sensor at its --rates rate per wall second, streamed "
                             "sensors first and at most one poll per pass, and read simulated time from a "
                             "Timer sensor; the run ends at --seconds of Timer time, or fails after --seconds x "
                             f"{WALL_LIMIT_FACTOR} wall seconds")


def add_measure_options(parser):
    _add_rates_and_mode(parser, LOCKSTEP,
                        f"polls per simulated second, or reads per wall second with --mode free, {RATES_RULE} "
                        f"(default {rates.DEFAULT_RATES_TEXT}, with --mode free {rates.FREE_DEFAULT_RATES_TEXT}; "
                        f"all={STEP_HZ} polls every sensor after every step)")
    parser.add_argument("--dump-raw", type=_count, default=0, metavar="N",
                        help=f"write the first N raw GPS and the first N raw IMU readings of each car, "
                             f"with each poll's wall time and step count, to {RAW_DUMP_FILE} in --out "
                             "(default 0: write nothing)")
    parser.add_argument("--stream", action="store_true",
                        help="create the lidar and camera with is_streaming=True and read them from shared "
                             "memory with stream() instead of poll(); readings that fail the checks in "
                             "kite_bridge/stream_check.py are dropped and counted under suspect "
                             "(default off: every sensor is polled)")
    _add_lidar_options(parser)


def add_live_options(parser):
    _add_rates_and_mode(parser, FREE, f"reads per wall second {RATES_RULE} (default {rates.FREE_DEFAULT_RATES_TEXT}); "
                                      "needs --mode free")
    parser.add_argument("--stream", action=argparse.BooleanOptionalAction,
                        help="read the lidar and camera from shared memory with stream(), checked by "
                             "kite_bridge/stream_check.py (default: on with --mode free; --no-stream polls them)")
    _add_lidar_options(parser)


def parse_args(description, argv=None, extra=None, check=None):
    """`check(args)`, if given, returns a usage error message or None. It sees `--rates` as the
    text given (None without it), `--ego-radius` as given (None without it) and `--stream` as
    given (None without it, when the script's option has no default). After it, `--rates`
    becomes the rates of the chosen mode, an unset `--ego-radius` becomes DEFAULT_EGO_RADIUS_M
    and an unset `--stream` becomes on in free mode and off in lock-step."""
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
    problem = check(args) if check else None
    if problem:
        parser.error(problem)
    if hasattr(args, "rates"):
        defaults = rates.FREE_DEFAULT_RATES if args.mode == FREE else rates.DEFAULT_RATES
        args.rates = rates.parse_rates(args.rates, defaults)
    if hasattr(args, "stream") and args.stream is None:
        args.stream = args.mode == FREE
    if hasattr(args, "ego_radius") and args.ego_radius is None:
        args.ego_radius = DEFAULT_EGO_RADIUS_M
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
    whatever the run's outcome, and the removal is skipped. Returns whether BeamNG.tech ended,
    and the close or Ctrl+C failure, else the pause or removal failure, else None."""
    pause_or_removal_failure = None
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
                pause_or_removal_failure = error
        _close_within(sim, close_wait)
    except BaseException as error:
        return _end_process(sim, error), error
    return True, pause_or_removal_failure


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


def print_sensor_settings(stream=False, lidar_pos=None):
    print("\n".join(settings_lines(stream, lidar_pos)))


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
                    sleep=None, on_reading=None, lidar_pos=None, self_hits=None):
    """Steps the simulator for `seconds` of simulator time and polls each sensor at its rate in
    `poll_rates`, a dict by sensor kind of polls per simulated second (see `rates` for the
    schedule). By default every sensor is polled once after each step, STEP_HZ per simulator
    second. `clock` returns wall seconds and `sleep` waits for wall seconds; a test passes fake
    ones. Without them the run uses `wall_clock` and `wall_sleep`, real time. `dump`, a
    RawDump, collects the raw GPS and IMU readings. `on_reading(key, reading)`, if given, is
    called on the loop's thread with each reading whose content is new, after it is counted; its
    time counts as our processing. New is `measure.Freshness` on the poll's fingerprints from
    `raw_data`: the rule `new_readings` counts by for the lidar, camera and radar. For GPS and IMU
    it compares the fingerprints of all readings in the poll with the previous poll's.

    `mode` FREE replaces the steps. The simulator runs on its own and each sensor is read on
    the wall-clock schedule in `rates`, so the rates are reads per wall second. Each pass reads
    the due streamed sensors, then at most one due poll: a sensor that is not streamed or the
    Timer, the most overdue first (`rates.pass_reads`). The Timer, on the first car, is polled
    in its own request at the higher of the GPS and IMU rates, and first of all, so every
    reading has a Timer time. The run ends at the first Timer reading `seconds` past its first
    one, and raises SpikeError after `seconds` x WALL_LIMIT_FACTOR wall seconds, when the Timer
    goes back, or when a Timer reading is not finite seconds >= 0. `simulated_s` is how far the
    Timer advanced. A reading without its own time gets the latest Timer reading, with
    `time_source` TIMER_TIME. The timing has `idle_s` (sleeping until the next read is due) and
    `timer_reads_s` (inside the Timer polls) in place of `step_wait_s`; `steps` and
    `steps_per_second` are None, a dump entry's step is None, and `timer` gives the Timer's poll
    count and rate. Each row and `timer` have `schedule`: `late` (reads that started
    `rates.LATE_S` or more after their due time), `worst_late_ms` (the latest start after a due
    time, None without reads) and `skipped` (due times passed over with no read). Every sensor
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

    `lidar_pos` is where each car's lidar is attached (None: the position in
    `sensor_settings.py`). The time spent counting is part of `processing_s`. With `self_hits`, a `self_hits.SelfHits`, each lidar row has
    `self_hits_approx`, its summary for that car, from every new scan and the car's latest GPS
    position (see `self_hits` for what that does and does not measure); without it the rows
    have no such key.

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
        sim.attach_sensors(car, stream=stream, lidar_pos=lidar_pos)
    keys = [(car, kind) for car in cars for kind in SENSOR_KINDS]
    streamed = {key for key in keys if stream and key[1] in STREAMED_KINDS}
    tally = _Tally(keys, streamed, dump, on_reading, self_hits)
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
        if run.lags is not None:
            rows[-1]["schedule"] = run.lags[car, kind].summary()
        if self_hits is not None and kind == "lidar":
            rows[-1]["self_hits_approx"] = self_hits.summary(car)
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
    lags: dict | None = None


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
    timer_key = (timer_car, TIMER)
    rate = {key: poll_rates[key[1]] for key in tally.keys}
    rate[timer_key] = max(poll_rates["gps"], poll_rates["imu"])
    streamed = [key for key in tally.keys if key in tally.streamed]
    slow = [timer_key, *(key for key in tally.keys if key not in tally.streamed)]
    limit = Fraction(seconds) * WALL_LIMIT_FACTOR
    sim.start_free_running(timer_car)
    timer = _Timer(sim)
    due = dict.fromkeys(rate, 0)
    lags = {key: _Lag() for key in rate}
    idle = 0.0
    loop_started = clock()

    def read(key):
        at = clock() - loop_started
        lags[key].add(at - due[key], rates.skipped(due[key], at, rate[key]))
        due[key] = rates.next_due(at, rate[key])
        if key == timer_key:
            timer.read(clock)
            return
        if key in slow:
            slow.remove(key)
            slow.append(key)
        tally.read(sim, key, clock, loop_started, None, timer.latest)

    read(timer_key)
    while timer.latest - timer.first < seconds:
        now = clock() - loop_started
        for key in rates.pass_reads(now, due, streamed, slow):
            read(key)
            if timer.latest - timer.first >= seconds:
                break
        else:
            if now >= limit:
                raise SpikeError(f"free run: the Timer advanced {timer.latest - timer.first:.3f} of {seconds:g} "
                                 f"simulated s in the wall-clock limit of {float(limit):g} s "
                                 f"(--seconds x {WALL_LIMIT_FACTOR})")
            started = clock()
            gap = min(limit, *due.values()) - Fraction(started - loop_started)
            if gap > 0:
                sleep(max(float(gap), MIN_SLEEP_S))
            idle += clock() - started
    wall = clock() - loop_started
    for key in tally.keys:
        lags[key].skipped += rates.skipped(due[key], now, rate[key])
    return _Run(None, None, timer.latest - timer.first, wall, {"idle_s": idle, "timer_reads_s": timer.read_s},
                {"timer": {"polls": timer.polls, "rate_hz": float(rate[timer_key]),
                           "schedule": lags.pop(timer_key).summary()}}, lags)


class _Timer:
    def __init__(self, sim):
        self._sim = sim
        self.first = self.latest = None
        self.polls = 0
        self.read_s = 0.0

    def read(self, clock):
        started = clock()
        try:
            value = self._sim.timer_seconds()
        except ValueError as error:
            raise SpikeError(f"free run: unusable Timer reading: {error}") from error
        self.read_s += clock() - started
        self.polls += 1
        if self.first is None:
            self.first = value
        elif value < self.latest:
            raise SpikeError(f"the Timer went back from {self.latest:g} s to {value:g} s")
        self.latest = value


class _Lag:
    """How late one sensor's (or the Timer's) free-mode reads started after their due times."""

    def __init__(self):
        self.late = 0
        self.worst_s = None
        self.skipped = 0

    def add(self, lateness_s, skipped):
        self.late += lateness_s >= rates.LATE_S
        self.worst_s = lateness_s if self.worst_s is None else max(self.worst_s, lateness_s)
        self.skipped += skipped

    def summary(self):
        return {"late": self.late, "worst_late_ms": None if self.worst_s is None else self.worst_s * 1000,
                "skipped": self.skipped}


class _Tally:
    def __init__(self, keys, streamed, dump, on_reading, self_hits=None):
        self.keys, self.streamed, self.dump, self.on_reading = keys, streamed, dump, on_reading
        self.self_hits = self_hits
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
        self.fresh = {key: measure.Freshness() for key in keys}

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
        if self.fresh[key].is_new(stats.keys or None):
            if self.self_hits is not None:
                self.self_hits.observe(key[0], key[1], reading.raw)
            if self.on_reading is not None:
                self.on_reading(key, reading)


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


def _schedule_lines(result):
    if result["mode"] != FREE:
        return []
    rows = [(r["car"], r["sensor"], r["schedule"]) for r in result["sensors"]]
    rows.append((result["sensors"][0]["car"], TIMER, result["timer"]["schedule"]))
    lines = [f"schedule: late counts reads that started {float(rates.LATE_S * 1000):g} ms or more after their due "
             "time, worst_ms is the latest start, skipped counts due times passed over with no read",
             f"{'car':<7}{'sensor':<8}{'late':>7}{'worst_ms':>10}{'skipped':>9}"]
    lines += [f"{car:<7}{sensor:<8}{s['late']:>7}{_fmt(s['worst_late_ms']):>10}{s['skipped']:>9}"
              for car, sensor, s in rows]
    lines += [f"skipped: {car} {sensor} missed {s['skipped']} due reads; its latest read started "
              f"{s['worst_late_ms']:.2f} ms after its due time" for car, sensor, s in rows if s["skipped"]]
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


def _self_hit_lines(result):
    rows = [r for r in result["sensors"] if "self_hits_approx" in r]
    if not rows:
        return []
    radius = rows[0]["self_hits_approx"]["radius_m"]
    lines = [f"self hits (approximate): lidar points within {radius:g} m of our car in x and y, measured from the "
             "latest GPS position; a vertical cylinder, not the car's shape, so ground points inside it count and "
             "the count can be off when the GPS position is older than the scan. Per new scan."]
    for r in rows:
        s = r["self_hits_approx"]
        mount = "(" + ", ".join(f"{value:g}" for value in s["lidar_pos"]) + ")"
        counted = (f"{s['scans_counted']} scans counted, mean {_fmt(s['mean'], digits=1)} points, "
                   f"max {_fmt(s['max'], digits=0)}")
        lines.append(f"self hits (approximate): {r['car']} lidar at {mount}: {counted}")
        if s["scans_without_position"]:
            lines.append(f"{r['car']} lidar: {s['scans_without_position']} new scans had no GPS position yet "
                         "and were not counted")
        if s["scans_unreadable"]:
            lines.append(f"{r['car']} lidar: {s['scans_unreadable']} new scans could not be read as x, y, z "
                         "points and were not counted")
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
            "real-time factor is what this run measured, not a target. Each pass read the due streamed "
            "sensors first, then at most one due poll (a sensor that is not streamed, or the Timer), the "
            "most overdue."]


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
    lines += _schedule_lines(result)
    lines += _self_hit_lines(result)
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
