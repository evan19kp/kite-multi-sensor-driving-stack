import argparse
import contextlib
import json
import math
import sys
import time
import traceback
from pathlib import Path

from kite_bridge import measure
from kite_bridge.fake_sim import FakeSim
from kite_bridge.sim_api import SENSOR_KINDS, STEP_HZ

MODEL_HINTS = ("tograc", "cherrier")


class SpikeError(Exception):
    pass


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


def parse_args(description, argv=None):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--home", help="the simulator's folder (required unless --fake)")
    parser.add_argument("--headless", action="store_true", help="run the simulator without a window")
    parser.add_argument("--seconds", type=_positive_float, default=30.0,
                        help="simulated seconds to run (default 30)")
    parser.add_argument("--traffic", type=_count, default=8,
                        help="AI traffic cars to spawn (default 8)")
    parser.add_argument("--out", default="recordings", help="output folder (default recordings/)")
    parser.add_argument("--model", help="our car's vehicle model (default: the single match found)")
    parser.add_argument("--fake", action="store_true", help="use the fake simulator")
    args = parser.parse_args(argv)
    if not args.fake and not args.home:
        parser.error("--home is required unless --fake is given")
    return args


def guarded(run, args):
    try:
        run(args)
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
    if args.fake:
        sim = FakeSim()
    else:
        if not Path(args.home).is_dir():
            raise SpikeError("--home is not a folder")
        from kite_bridge.beamng_adapter import BeamNGAdapter

        sim = BeamNGAdapter(args.home, args.headless)
    try:
        sim.open()
        yield sim
    finally:
        sim.close()


def resolve_model(sim, requested):
    if requested:
        return requested
    try:
        models = sim.list_models()
    except Exception as error:
        raise SpikeError(
            f"could not list vehicle models ({type(error).__name__}: {error}); pass --model") from error
    matches = [m for m in models if any(hint in m.lower() for hint in MODEL_HINTS)]
    for name in matches or models:
        print(f"model: {name}")
    if len(matches) != 1:
        found = "several models match" if matches else "no model matches"
        raise SpikeError(f"{found} {'/'.join(MODEL_HINTS)}; pass --model with one of the names above")
    return matches[0]


def start_traffic(sim, count):
    if count == 0:
        return False
    try:
        sim.spawn_traffic(count)
    except Exception as error:
        print(f"traffic not spawned ({type(error).__name__}: {error}); continuing with our car only")
        return False
    return True


def steps_for(seconds):
    return max(1, round(seconds * STEP_HZ))


def _time_source(sources):
    return "mixed" if len(sources) > 1 else next(iter(sources), None)


def measure_sensors(sim, cars, seconds):
    for car in cars:
        sim.attach_sensors(car)
    keys = [(car, kind) for car in cars for kind in SENSOR_KINDS]
    series = {key: [] for key in keys}
    latencies = {key: [] for key in keys}
    points = {key: [] for key in keys}
    sources = {key: set() for key in keys}
    unexpected = {key: [] for key in keys}
    steps = steps_for(seconds)
    step_time = 0.0
    for _ in range(steps):
        started = time.perf_counter()
        sim.step(1)
        step_time += time.perf_counter() - started
        for key in keys:
            started = time.perf_counter()
            reading = sim.poll(*key)
            latencies[key].append(time.perf_counter() - started)
            if reading.unexpected:
                unexpected[key].append(reading.unexpected)
                continue
            series[key].append(reading.timestamp)
            if reading.timestamp is not None:
                sources[key].add(reading.time_source)
            if reading.points is not None:
                points[key].append(reading.points)
    rows = [{"car": car, "sensor": kind,
             **measure.summarise(series[car, kind], latencies[car, kind], points[car, kind]),
             "time_source": _time_source(sources[car, kind]),
             "unexpected": {"count": len(unexpected[car, kind]),
                            "first": unexpected[car, kind][0] if unexpected[car, kind] else None}}
            for car, kind in keys]
    return {"cars": len(cars), "steps": steps, "steps_per_second": measure.steps_per_second(steps, step_time),
            "sensors": rows}


def _fmt(value, scale=1.0, digits=2):
    return "n/a" if value is None else f"{value * scale:.{digits}f}"


def format_result(result):
    lines = [f"cars: {result['cars']}  steps per second: {_fmt(result['steps_per_second'], digits=1)}",
             f"{'car':<7}{'sensor':<8}{'polls':>6}{'rate_hz':>9}{'time':>7}{'mean_ms':>9}{'p50_ms':>8}"
             f"{'p95_ms':>8}{'max_ms':>8}{'empty':>7}{'repeat':>8}{'unexp':>7}{'points':>8}"]
    for r in result["sensors"]:
        lat = r["latency_s"]
        lines.append(f"{r['car']:<7}{r['sensor']:<8}{r['polls']:>6}{_fmt(r['rate_hz']):>9}"
                     f"{r['time_source'] or 'n/a':>7}"
                     f"{_fmt(lat['mean'], 1000):>9}{_fmt(lat['p50'], 1000):>8}{_fmt(lat['p95'], 1000):>8}"
                     f"{_fmt(lat['max'], 1000):>8}{r['empty']:>7}{r['repeated']:>8}"
                     f"{r['unexpected']['count']:>7}{_fmt(r['points_mean'], digits=0):>8}")
    from_poll = [f"{r['car']} {r['sensor']}" for r in result["sensors"]
                 if r["time_source"] in ("poll", "mixed")]
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
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {path}")
