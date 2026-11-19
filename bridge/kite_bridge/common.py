import argparse
import contextlib
import json
import math
import sys
import threading
import time
import traceback
from pathlib import Path

from kite_bridge import measure, raw_data, rates
from kite_bridge.fake_sim import FakeSim
from kite_bridge.output_guard import PROVENANCE, out_refusal, repo_roots
from kite_bridge.readings import readings_in
from kite_bridge.sensor_settings import requested_update_hz, settings_lines
from kite_bridge.sim_api import BUFFERED_KINDS, POLL_TIME, SENSOR_KINDS, STEP_HZ, Reading

RAW_DUMP_FILE = "raw_gps_imu.json"
DEFAULT_MODEL = "vivace"
DEFAULT_CONFIG = "vehicles/vivace/tograc_qE.pc"
TECH_VERSION_TEXT = "not reported by BeamNGpy 1.36"
MIXED_TIME = "mixed"
DEFAULT_TIMEOUT_S = 300.0
CLOSE_WAIT_S = 20.0  # BeamNGpy's close() waits up to 10 s for the simulator to quit, then ends it itself
JOIN_SLICE_S = 0.1
EXIT_INTERRUPTED = 130


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
                             f"each above 0 and at most {STEP_HZ}; all=RATE sets every sensor and a named "
                             f"sensor overrides it (default {rates.DEFAULT_RATES_TEXT}; all={STEP_HZ} polls "
                             "every sensor after every step)")
    parser.add_argument("--dump-raw", type=_count, default=0, metavar="N",
                        help=f"write the first N raw GPS and the first N raw IMU readings of each car, "
                             f"with each poll's wall time and step count, to {RAW_DUMP_FILE} in --out "
                             "(default 0: write nothing)")


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
    """Ends BeamNG.tech without waiting forever. Removes the sensors only while the connection is
    `in_step`, then closes; if close raises or overruns `close_wait`, or Ctrl+C comes during
    either, ends the process BeamNGpy launched. Returns whether BeamNG.tech ended, and the close
    or Ctrl+C failure, else the removal failure, else None."""
    removal_failure = None
    try:
        if in_step:
            try:
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


def print_sensor_settings():
    print("\n".join(settings_lines()))


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
        `step` is the steps taken when the poll was made, `wall_s` the wall seconds since the run's
        first step when the poll returned and `poll_wall_s` how long the poll call took."""
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


def measure_sensors(sim, cars, seconds, clock=time.perf_counter, dump=None, poll_rates=None):
    """Steps the simulator for `seconds` of simulator time and polls each sensor at its rate in
    `poll_rates`, a dict by sensor kind of polls per simulated second (see `rates` for the
    schedule). By default every sensor is polled once after each step, STEP_HZ per simulator
    second. `clock` returns wall seconds; a test passes a fake one. `dump`, a RawDump, collects
    the raw GPS and IMU readings.

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
    poll_rates = rates.every_step() if poll_rates is None else poll_rates
    for car in cars:
        sim.attach_sensors(car)
    keys = [(car, kind) for car in cars for kind in SENSOR_KINDS]
    series = {key: [] for key in keys}
    latencies = {key: [] for key in keys}
    points = {key: [] for key in keys}
    detections = {key: [] for key in keys}
    sources = {key: set() for key in keys}
    unexpected = {key: [] for key in keys}
    per_poll = {key: [] for key in keys}
    sizes = {key: 0 for key in keys}
    poll_keys = {key: [] for key in keys}
    reading_keys = {key: [] for key in keys}
    steps = steps_for(seconds)
    step_time = 0.0
    loop_started = clock()
    for step_number in range(1, steps + 1):
        started = clock()
        sim.step(1)
        step_time += clock() - started
        for key in keys:
            if not rates.polled(step_number, poll_rates[key[1]]):
                continue
            started = clock()
            try:
                reading = sim.poll(*key)
            except TimeoutError:
                raise
            except Exception as error:
                reading = Reading(None, unexpected=f"poll raised {type(error).__name__}: {error}")
            finished = clock()
            latencies[key].append(finished - started)
            stats = raw_data.EMPTY
            if reading.timestamp is not None and not reading.unexpected:
                try:
                    stats = raw_data.poll_stats(key[1], reading.raw)
                except ValueError as error:
                    reading = Reading(None, unexpected=f"raw data not measurable: {error}")
            if reading.unexpected:
                unexpected[key].append(reading.unexpected)
                continue
            series[key].append(reading.timestamp)
            sizes[key] += stats.nbytes
            if key[1] in BUFFERED_KINDS:
                reading_keys[key].extend(stats.keys)
            else:
                poll_keys[key].append(stats.keys[0] if stats.keys else None)
            if reading.timestamp is not None:
                sources[key].add(reading.time_source)
                per_poll[key].append(reading.count)
                if dump is not None and key[1] in BUFFERED_KINDS:
                    dump.add(key[0], key[1], step_number, finished - loop_started,
                             finished - started, readings_in(reading.raw))
            if reading.points is not None:
                points[key].append(reading.points)
            if reading.detections is not None:
                detections[key].append(reading.detections)
    wall = clock() - loop_started
    simulated = steps / STEP_HZ
    reads = math.fsum(math.fsum(latencies[key]) for key in keys)
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
            "possible_loss": (poll_rates[kind] < STEP_HZ and requested is not None
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
    timing = {"simulated_s": simulated, "wall_s": wall, "real_time_factor": measure.real_time_factor(simulated, wall),
              "step_wait_s": step_time, "sensor_reads_s": reads,
              "processing_s": measure.processing_time(wall, step_time, reads)}
    return {"cars": len(cars), "steps": steps, "steps_per_second": measure.steps_per_second(steps, step_time),
            "timing": timing, "sensors": rows}


def _fmt(value, scale=1.0, digits=2):
    return "n/a" if value is None else f"{value * scale:.{digits}f}"


def _timing_lines(result):
    t = result["timing"]
    wall = t["wall_s"]

    def part(label, seconds):
        share = _fmt(seconds / wall * 100, digits=1) if wall > 0 else "n/a"
        return f"  {label:<18}{seconds:>9.3f} s{share:>7}%"

    lines = [f"timing: {_fmt(t['simulated_s'], digits=3)} simulated s in {_fmt(wall, digits=3)} wall s, "
             f"real-time factor {_fmt(t['real_time_factor'], digits=3)}",
             part("waiting for steps", t["step_wait_s"]), part("in sensor reads", t["sensor_reads_s"]),
             part("our processing", t["processing_s"]),
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
    lines = []
    for r in result["sensors"]:
        if r["possible_loss"]:
            lines.append(f"possible loss: {r['car']} {r['sensor']} gave {r['new_per_simulated_s']:.2f} new "
                         f"readings per simulated second at {r['configured_rate_hz']:g} polls per second, "
                         f"fewer than the {r['requested_update_hz']:g} Hz requested. "
                         f"Compare with a run at --rates all={STEP_HZ}.")
    buffered = sorted({r["sensor"] for r in result["sensors"]
                       if r["sensor"] in BUFFERED_KINDS and r["configured_rate_hz"] < STEP_HZ})
    if buffered:
        lines.append(f"loss not checkable for {', '.join(buffered)}: a buffered sensor's capacity is not "
                     f"known, so compare its new readings with a run at --rates all={STEP_HZ}.")
    return lines


def format_result(result):
    lines = [f"cars: {result['cars']}  steps per second: {_fmt(result['steps_per_second'], digits=1)}",
             f"{'car':<7}{'sensor':<8}{'polls':>6}{'rate_hz':>9}{'time':>7}{'per_poll':>10}{'mean_ms':>9}"
             f"{'p50_ms':>8}{'p95_ms':>8}{'max_ms':>8}{'empty':>7}{'repeat':>8}{'unexp':>7}{'points':>8}"
             f"{'detect':>8}"]
    for r in result["sensors"]:
        lat = r["latency_s"]
        lines.append(f"{r['car']:<7}{r['sensor']:<8}{r['polls']:>6}{_fmt(r['rate_hz']):>9}"
                     f"{r['time_source'] or 'n/a':>7}{r['readings_per_poll_max'] or 'n/a':>10}"
                     f"{_fmt(lat['mean'], 1000):>9}{_fmt(lat['p50'], 1000):>8}{_fmt(lat['p95'], 1000):>8}"
                     f"{_fmt(lat['max'], 1000):>8}{r['empty']:>7}{r['repeated']:>8}"
                     f"{r['unexpected']['count']:>7}{_fmt(r['points_mean'], digits=0):>8}"
                     f"{_fmt(r['detections_mean'], digits=0):>8}")
    lines += _timing_lines(result)
    lines.append("rate_hz: distinct reading timestamps per simulator second. It counts at most one "
                 f"new reading per poll, so it is limited to about the sensor's poll rate (poll_hz, "
                 f"at most {STEP_HZ}). per_poll shows when a sensor delivered more.")
    lines += _loss_lines(result)
    for r in result["sensors"]:
        if (r["readings_per_poll_max"] or 0) > 1:
            lines.append(f"{r['car']} {r['sensor']}: polls returned up to {r['readings_per_poll_max']} "
                         "readings. rate_hz counts at most one per poll, so it undercounts this sensor.")
    from_poll = [f"{r['car']} {r['sensor']}" for r in result["sensors"]
                 if r["time_source"] in (POLL_TIME, MIXED_TIME)]
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
