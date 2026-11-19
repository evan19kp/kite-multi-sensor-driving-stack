import array
import math
import random
import time

from kite_bridge.readings import reading_from_raw
from kite_bridge.sensor_settings import SENSOR_SETTINGS
from kite_bridge.sim_api import BUFFERED_KINDS, SENSOR_KINDS, STEP_HZ, STREAMED_KINDS, Simulator
from kite_bridge.stream_check import reading_from_stream

DEFAULT_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 20.0, "gps": 10.0, "imu": 100.0}
DEFAULT_MODELS = ("vivace", "fake_van", "fake_sedan")
FAKE_SPEED_M_S = 10.0
FAKE_LIDAR_POINTS = 200
FAKE_LIDAR_RANGE_M = 60.0
FAKE_IMAGE_WIDTH = 8
FAKE_IMAGE_HEIGHT = 4
FAKE_TRAFFIC_GAP_M = 15.0
FAKE_LANE_M = 3.5
FAKE_RADAR_ROWS = 20
FAKE_FLOAT32_BYTES = 4
FAKE_LIDAR_ROW_RANGE = (900, 1100)
FAKE_STREAM_ROWS = FAKE_LIDAR_ROW_RANGE[1]
FAKE_LIDAR_COLUMNS = 3


class FakeClock:
    """A clock that moves only when told to, so a test knows the wall time exactly."""

    def __init__(self):
        self._now = 0.0

    def now(self):
        return self._now

    def advance(self, seconds):
        self._now += seconds


class FakeCloud:
    """Stands in for a numpy array of float32 rows: it has `len()` and `tobytes()`, and its truth
    value raises, like numpy's. Its content depends on `tick` alone."""

    def __init__(self, rows, tick, columns):
        self._rows, self._tick, self._columns = rows, tick, columns

    def __len__(self):
        return self._rows

    def __bool__(self):
        raise ValueError("The truth value of an array with more than one element is ambiguous")

    def tobytes(self):
        return self._tick.to_bytes(FAKE_FLOAT32_BYTES, "little") * (self._rows * self._columns)


class FakeRows:
    """Stands in for a numpy float32 array of rows: it has `shape`, `len()`, `tobytes()` (the
    rows one after another) and slicing by rows, and its truth value raises, like numpy's."""

    def __init__(self, data, columns):
        self._data = bytes(data)
        self.shape = (len(self._data) // (FAKE_FLOAT32_BYTES * columns), columns)

    def __len__(self):
        return self.shape[0]

    def __bool__(self):
        raise ValueError("The truth value of an array with more than one element is ambiguous")

    def __getitem__(self, rows):
        start, stop, _ = rows.indices(self.shape[0])
        row_bytes = FAKE_FLOAT32_BYTES * self.shape[1]
        return FakeRows(self._data[start * row_bytes:max(start, stop) * row_bytes], self.shape[1])

    def tobytes(self):
        return self._data


class FakeImage:
    """Stands in for a PIL image: `size` is (width, height) and `tobytes()` its RGB bytes, which
    depend on `tick` alone."""

    def __init__(self, size, tick):
        self.size = tuple(size)
        self._tick = tick

    def tobytes(self):
        return bytes([self._tick % 256]) * (self.size[0] * self.size[1] * 3)


class FakeSim(Simulator):
    """A seeded synthetic simulator. With a `clock` it also has a cost in fake seconds: each poll
    advances the clock by `request_cost` (one number for every sensor, or a dict by sensor) and
    each step by `step_cost`; nothing waits for real. Sensor `kind` produces a new sample
    `rates[kind]` times per simulated second. A poll returns the sample current at its time, so a
    poll between two samples repeats the last one. GPS and IMU keep every sample made since the
    last poll and return them together, like BeamNGpy's buffer. `lidar_rows` fixes the lidar
    point count, or, as a tuple, gives sample `tick` the count `lidar_rows[tick % len(lidar_rows)]`;
    by default it varies between 900 and 1099 with the sample.

    Streaming (`attach_sensors(car, stream=True)`, then `stream`) costs no fake time and sends no
    request. The lidar's shared memory is a block of `stream_rows` float32 rows of x, y, z, all
    zero at first. Every sample, read or not, writes its scan over the start of the block and
    leaves the rest as it was, so after a shorter scan the block holds the scan, then rows left
    from an earlier longer scan, then zero rows. Row r of sample t is (t + 1, r + 1, 2). Until
    the next sample the block does not change. The
    streamed camera gives one image of the configured resolution per sample. Streamed readings
    carry no time, like BeamNGpy's lidar and camera."""

    def __init__(self, seed=0, rates=None, dropout=0.0, repeat=0.0, latency=0.0,
                 models=DEFAULT_MODELS, sensor_time=True, clock=None, request_cost=0.0,
                 step_cost=0.0, lidar_rows=None, stream_rows=FAKE_STREAM_ROWS):
        self._request_cost = (dict(request_cost) if isinstance(request_cost, dict)
                              else dict.fromkeys(SENSOR_KINDS, request_cost))
        if clock is None and any((step_cost, *self._request_cost.values())):
            raise ValueError("a request or step cost needs a clock")
        self._clock = clock
        self._step_cost = step_cost
        self._lidar_rows = lidar_rows
        self._stream_rows = stream_rows
        self._streamed = set()
        self._blocks = {}
        self._block_tick = {}
        self._sensor_time = sensor_time
        self._seed = seed
        self._rng = random.Random(seed)
        self._rates = dict(DEFAULT_RATES if rates is None else rates)
        self._dropout = dropout
        self._repeat = repeat
        self._latency = latency
        self._models = list(models)
        self._steps = 0
        self._positions = {}
        self._modes = {}
        self._attached = set()
        self._last = {}
        self._last_tick = {}
        self.injected = {}
        self.traffic_requested = None
        self.spawned_model = None
        self.spawned_config = None
        self.sensors_removed = False
        self.closed = False
        self.process_ended = False

    def open(self):
        pass

    def remove_sensors(self):
        self._attached.clear()
        self.sensors_removed = True

    def close(self):
        self.closed = True

    def end_process(self):
        self.process_ended = True
        return True

    def versions(self):
        return {"BeamNGpy version": "fake", "BeamNGpy protocol": "fake"}

    def list_models(self):
        return list(self._models)

    def load_scenario(self, model, n_cars, config=None):
        if n_cars < 1:
            raise ValueError("n_cars must be at least 1")
        self.spawned_model = model
        self.spawned_config = config
        cars = [f"kite{i + 1}" for i in range(n_cars)]
        self._positions = {car: (5.0 * i, 0.0, 0.0) for i, car in enumerate(cars)}
        self._modes = {car: "disabled" for car in cars}
        return cars

    def spawn_traffic(self, n):
        self.traffic_requested = n

    def attach_sensors(self, car, stream=False):
        self._attached.update((car, kind) for kind in SENSOR_KINDS)
        if stream:
            self._streamed.update((car, kind) for kind in STREAMED_KINDS)
        for kind in SENSOR_KINDS:
            self.injected[(car, kind)] = {"empty": 0, "repeated": 0}

    def _charge(self, seconds):
        if self._clock is not None:
            self._clock.advance(seconds)

    def _scan_rows(self, car, tick):
        if self._lidar_rows is None:
            return random.Random(f"{self._seed}:{car}:{tick}").randrange(*FAKE_LIDAR_ROW_RANGE)
        if isinstance(self._lidar_rows, int):
            return self._lidar_rows
        return self._lidar_rows[tick % len(self._lidar_rows)]

    def _tick(self, kind):
        return math.floor(self.sim_time * self._rates[kind] + 1e-9)

    def _sample(self, car, kind, tick):
        sample = {}
        if self._sensor_time:
            sample["time"] = tick / self._rates[kind]
        if kind == "lidar":
            sample["pointCloud"] = FakeCloud(self._scan_rows(car, tick), tick, FAKE_LIDAR_COLUMNS)
        elif kind == "camera":
            pixels = FAKE_IMAGE_WIDTH * FAKE_IMAGE_HEIGHT * 3
            sample["colour"] = tick.to_bytes(FAKE_FLOAT32_BYTES, "little") * pixels
        elif kind == "radar":
            sample["returns"] = FakeCloud(FAKE_RADAR_ROWS, tick, 7)
        else:
            sample["tick"] = tick
            sample["value"] = tick * 0.5
        return sample

    def poll(self, car, kind):
        self._require_attached(car, kind)
        if self._latency:
            time.sleep(self._latency)
        self._charge(self._request_cost.get(kind, 0.0))
        key = (car, kind)
        if self._rng.random() < self._dropout:
            self.injected[key]["empty"] += 1
            return reading_from_raw(kind, None, self.sim_time)
        tick = self._tick(kind)
        made = max(1, tick - self._last_tick.get(key, tick))
        self._last_tick[key] = tick
        samples = [self._sample(car, kind, t) for t in range(tick - made + 1, tick + 1)]
        last = self._last.get(key)
        if self._sensor_time and last is not None and self._rng.random() < self._repeat:
            self.injected[key]["repeated"] += 1
            samples = [last]
        self._last[key] = samples[-1]
        raw = samples if kind in BUFFERED_KINDS else samples[-1]
        return reading_from_raw(kind, raw, self.sim_time)

    def stream(self, car, kind):
        self._require_attached(car, kind)
        if (car, kind) not in self._streamed:
            raise ValueError(f"{kind!r} of {car!r} was not attached with stream=True")
        tick = self._tick(kind)
        if kind == "lidar":
            raw = {"pointCloud": FakeRows(self._lidar_block(car, tick), FAKE_LIDAR_COLUMNS)}
        else:
            raw = {"colour": FakeImage(SENSOR_SETTINGS["camera"]["resolution"], tick)}
        return reading_from_stream(kind, raw, self.sim_time)

    def _lidar_block(self, car, tick):
        row_bytes = FAKE_FLOAT32_BYTES * FAKE_LIDAR_COLUMNS
        block = self._blocks.setdefault(car, bytearray(self._stream_rows * row_bytes))
        for written in range(self._block_tick.get(car, -1) + 1, tick + 1):
            rows = self._scan_rows(car, written)
            if rows > self._stream_rows:
                raise ValueError(f"a scan of {rows} rows does not fit a block of {self._stream_rows}")
            scan = array.array("f", (v for row in range(rows) for v in (written + 1.0, row + 1.0, 2.0)))
            block[:rows * row_bytes] = scan.tobytes()
            self._block_tick[car] = written
        return block

    def step(self, n=1):
        self._charge(self._step_cost * n)
        self._steps += n
        for car, mode in self._modes.items():
            if mode == "traffic":
                x, y, z = self._positions[car]
                self._positions[car] = (x + FAKE_SPEED_M_S * n / STEP_HZ, y, z)

    def lidar_points(self, car):
        self._require_attached(car, "lidar")
        rng = random.Random(f"{self._seed}:{self._steps}")
        x, y, z = self._positions[car]
        points = []
        for _ in range(FAKE_LIDAR_POINTS):
            angle = rng.uniform(0.0, 2.0 * math.pi)
            radius = rng.uniform(1.0, FAKE_LIDAR_RANGE_M)
            points.append((x + radius * math.cos(angle), y + radius * math.sin(angle), z + rng.uniform(0.0, 3.0)))
        return points

    def camera_image(self, car):
        self._require_attached(car, "camera")
        shift = self._steps % FAKE_IMAGE_WIDTH
        return [[(255 * ((column + shift) % FAKE_IMAGE_WIDTH) // (FAKE_IMAGE_WIDTH - 1), 60 * row, 128)
                 for column in range(FAKE_IMAGE_WIDTH)] for row in range(FAKE_IMAGE_HEIGHT)]

    def other_positions(self):
        travelled = FAKE_SPEED_M_S * self.sim_time
        return [(FAKE_TRAFFIC_GAP_M * (i + 1) + travelled, FAKE_LANE_M * (-1) ** i, 0.0)
                for i in range(self.traffic_requested or 0)]

    def _require_attached(self, car, kind):
        if (car, kind) not in self._attached:
            raise KeyError(f"sensor {kind!r} is not attached to {car!r}")

    def set_ai_mode(self, car, mode):
        self._modes[car] = mode

    def position(self, car):
        return self._positions[car]

    @property
    def sim_time(self):
        return self._steps / STEP_HZ
