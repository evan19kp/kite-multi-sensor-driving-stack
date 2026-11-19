import math
import random
import time

from kite_bridge.sim_api import POLL_TIME, SENSOR_KINDS, STEP_HZ, Reading, Simulator

DEFAULT_RATES = {"lidar": 10.0, "camera": 20.0, "radar": 20.0, "gps": 10.0, "imu": 100.0}
DEFAULT_MODELS = ("fake_sedan", "fake_cherrier_tograc", "fake_van")
FAKE_SPEED_M_S = 10.0


class FakeSim(Simulator):
    def __init__(self, seed=0, rates=None, dropout=0.0, repeat=0.0, latency=0.0,
                 models=DEFAULT_MODELS, sensor_time=True):
        self._sensor_time = sensor_time
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

    def open(self):
        pass

    def close(self):
        pass

    def versions(self):
        return {"BeamNG.tech": "fake", "BeamNGpy": "fake"}

    def list_models(self):
        return list(self._models)

    def load_scenario(self, model, n_cars):
        if n_cars < 1:
            raise ValueError("n_cars must be at least 1")
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
        if (car, kind) not in self._attached:
            raise KeyError(f"sensor {kind!r} is not attached to {car!r}")
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

    def set_ai_mode(self, car, mode):
        self._modes[car] = mode

    def position(self, car):
        return self._positions[car]

    @property
    def sim_time(self):
        return self._steps / STEP_HZ
