import argparse
import contextlib
import json
import math
import sys
import threading
import time
import traceback
from pathlib import Path

from kite_bridge import measure
from kite_bridge.fake_sim import FakeSim
from kite_bridge.output_guard import PROVENANCE, out_refusal, repo_roots
from kite_bridge.sensor_settings import settings_lines
from kite_bridge.sim_api import POLL_TIME, SENSOR_KINDS, STEP_HZ, Reading

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


def measure_sensors(sim, cars, seconds):
    """Steps the simulator for `seconds` of simulator time and polls every sensor once after
    each step, so the poll rate is STEP_HZ per simulator second.

    A row's `rate_hz` is the rate of distinct reading timestamps seen at poll time. It counts
    at most one new reading per poll, so it is limited to about the poll rate.
    `readings_per_poll_max` is the most readings one poll returned; above 1, the sensor
    produced readings faster than it was polled. A poll that raises counts as an unexpected
    reading, with the error as its message, and the run goes on, except for a timeout, which
    stops the run. Unexpected readings are left out of `polls`, `empty`, `rate_hz` and
    `readings_per_poll_max`; the latency figures include their polls."""
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
    steps = steps_for(seconds)
    step_time = 0.0
    for _ in range(steps):
        started = time.perf_counter()
        sim.step(1)
        step_time += time.perf_counter() - started
        for key in keys:
            started = time.perf_counter()
            try:
                reading = sim.poll(*key)
            except TimeoutError:
                raise
            except Exception as error:
                reading = Reading(None, unexpected=f"poll raised {type(error).__name__}: {error}")
            latencies[key].append(time.perf_counter() - started)
            if reading.unexpected:
                unexpected[key].append(reading.unexpected)
                continue
            series[key].append(reading.timestamp)
            if reading.timestamp is not None:
                sources[key].add(reading.time_source)
                per_poll[key].append(reading.count)
            if reading.points is not None:
                points[key].append(reading.points)
            if reading.detections is not None:
                detections[key].append(reading.detections)
    rows = [{"car": car, "sensor": kind,
             **measure.summarise(series[car, kind], latencies[car, kind], points[car, kind],
                                 detections[car, kind]),
             "time_source": _time_source(sources[car, kind]),
             "readings_per_poll_max": max(per_poll[car, kind], default=None),
             "unexpected": {"count": len(unexpected[car, kind]),
                            "first": unexpected[car, kind][0] if unexpected[car, kind] else None}}
            for car, kind in keys]
    return {"cars": len(cars), "steps": steps, "steps_per_second": measure.steps_per_second(steps, step_time),
            "sensors": rows}


def _fmt(value, scale=1.0, digits=2):
    return "n/a" if value is None else f"{value * scale:.{digits}f}"


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
    lines.append(f"rate_hz: distinct reading timestamps per simulator second. It counts at most one "
                 f"new reading per poll, so it is limited to about the poll rate of {STEP_HZ} Hz. "
                 "per_poll shows when a sensor delivered more.")
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
