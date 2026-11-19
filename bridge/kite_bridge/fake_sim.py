import math
import random
import time

from kite_bridge.sim_api import POLL_TIME, SENSOR_KINDS, STEP_HZ, Reading, Simulator

DEFAULT_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 20.0, "gps": 10.0, "imu": 100.0}
DEFAULT_MODELS = ("vivace", "fake_van", "fake_sedan")
FAKE_SPEED_M_S = 10.0
FAKE_LIDAR_POINTS = 200
FAKE_LIDAR_RANGE_M = 60.0
FAKE_IMAGE_WIDTH = 8
FAKE_IMAGE_HEIGHT = 4
FAKE_TRAFFIC_GAP_M = 15.0
FAKE_LANE_M = 3.5


class FakeSim(Simulator):
    def __init__(self, seed=0, rates=None, dropout=0.0, repeat=0.0, latency=0.0,
                 models=DEFAULT_MODELS, sensor_time=True):
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

    def attach_sensors(self, car):
        self._attached.update((car, kind) for kind in SENSOR_KINDS)
        for kind in SENSOR_KINDS:
            self.injected[(car, kind)] = {"empty": 0, "repeated": 0}

    def poll(self, car, kind):
        self._require_attached(car, kind)
        if self._latency:
            time.sleep(self._latency)
        key = (car, kind)
        if self._rng.random() < self._dropout:
            self.injected[key]["empty"] += 1
            return Reading(None)
        points = self._rng.randrange(900, 1100) if kind == "lidar" else None
        if not self._sensor_time:
            return Reading(self.sim_time, points, POLL_TIME)
        rate = self._rates[kind]
        tick = math.floor(self.sim_time * rate + 1e-9)
        count = max(1, tick - self._last_tick.get(key, tick))
        self._last_tick[key] = tick
        timestamp = tick / rate
        last = self._last.get(key)
        if last is not None and self._rng.random() < self._repeat:
            self.injected[key]["repeated"] += 1
            timestamp, count = last, 1
        self._last[key] = timestamp
        return Reading(timestamp, points, count=count)

    def step(self, n=1):
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
