"""The real simulator behind `sim_api.Simulator`, through BeamNGpy.

This is the only module that imports `beamngpy`, and it does so lazily, inside methods, so
importing the module needs no simulator and no installed packages. Every BeamNGpy call lives
here, and none has run against BeamNGpy 1.36 yet. Calls whose name, arguments or result format
are guesses carry an `# ASSUMPTION:` comment, here and in `readings.py`; the first real run
checks them.
"""

import importlib.metadata

from kite_bridge.readings import image_from_raw, points_from_raw, reading_from_raw
from kite_bridge.sensor_settings import SENSOR_SETTINGS
from kite_bridge.sim_api import STEP_HZ, Simulator, release

PORT = 25252
SCENARIO_LEVEL = "west_coast_usa"
SPAWN_POS = (-717.121, 101.0, 118.675)
SPAWN_ROT_QUAT = (0.0, 0.0, 0.3826834, 0.9238795)
SPAWN_SPACING_M = 5.0  # ASSUMPTION: extra cars start this far apart along x, on or near the road


class BeamNGAdapter(Simulator):
    def __init__(self, home, headless):
        self._home = home
        self._headless = headless
        self._bng = None
        self._vehicles = {}
        self._sensors = {}
        self._steps = 0

    def open(self):
        from beamngpy import BeamNGpy

        self._bng = BeamNGpy("localhost", PORT, home=self._home, headless=self._headless,
                             gfx="dx11", quit_on_close=True)
        self._bng.open(launch=True)

    def close(self):
        sensors, self._sensors = self._sensors, {}
        bng, self._bng = self._bng, None
        release(sensors, bng.close if bng is not None else lambda: None)

    def versions(self):
        from beamngpy.connection.connection import Connection

        return {"BeamNGpy version": importlib.metadata.version("beamngpy"),
                "BeamNGpy protocol": str(Connection.PROTOCOL_VERSION)}

    def list_models(self):
        return [str(name) for name in self._bng.vehicles.get_available()]

    def load_scenario(self, model, n_cars, config=None):
        from beamngpy import Scenario, Vehicle

        scenario = Scenario(SCENARIO_LEVEL, "kite_spike", description="Kite sensor spike")
        x, y, z = SPAWN_POS
        part_config = {"part_config": config} if config else {}
        for i in range(n_cars):
            name = f"kite{i + 1}"
            vehicle = Vehicle(name, model=model, license="KITE", **part_config)
            scenario.add_vehicle(vehicle, pos=(x + i * SPAWN_SPACING_M, y, z),
                                 rot_quat=SPAWN_ROT_QUAT)
            self._vehicles[name] = vehicle
        scenario.make(self._bng)
        self._bng.settings.set_deterministic(STEP_HZ)
        self._bng.scenario.load(scenario)
        self._bng.scenario.start()
        self._steps = 0
        return list(self._vehicles)

    def spawn_traffic(self, n):
        self._bng.traffic.spawn(max_amount=n)  # ASSUMPTION: not verified for 1.36

    def attach_sensors(self, car):
        from beamngpy.sensors import GPS, AdvancedIMU, Camera, Lidar, Radar

        bng, vehicle = self._bng, self._vehicles[car]
        classes = {"lidar": Lidar, "camera": Camera, "radar": Radar, "gps": GPS, "imu": AdvancedIMU}
        self._sensors.update({
            (car, kind): sensor_class(f"{kind}_{car}", bng, vehicle, **SENSOR_SETTINGS[kind])
            for kind, sensor_class in classes.items()
        })

    def poll(self, car, kind):
        return reading_from_raw(kind, self._sensors[(car, kind)].poll(), self.sim_time)

    def lidar_points(self, car):
        return points_from_raw(self._sensors[(car, "lidar")].poll())

    def camera_image(self, car):
        return image_from_raw(self._sensors[(car, "camera")].poll())

    def other_positions(self):
        positions = []
        for name, vehicle in self._bng.vehicles.get_current().items():  # ASSUMPTION: a dict of every vehicle by name in 1.36
            if name in self._vehicles:
                continue
            if not vehicle.is_connected():  # ASSUMPTION: traffic cars need connecting before their state can be polled
                vehicle.connect(self._bng)
            vehicle.sensors.poll()  # ASSUMPTION: refreshes vehicle.state in 1.36
            x, y, z = vehicle.state["pos"]
            positions.append((float(x), float(y), float(z)))
        return positions

    def step(self, n=1):
        self._bng.control.step(n)  # ASSUMPTION: the 1.36 call, and it returns once the n steps are done
        self._steps += n

    def set_ai_mode(self, car, mode):
        self._vehicles[car].ai.set_mode(mode)

    def position(self, car):
        vehicle = self._vehicles[car]
        vehicle.sensors.poll()  # ASSUMPTION: refreshes vehicle.state in 1.36
        x, y, z = vehicle.state["pos"]
        return (float(x), float(y), float(z))

    @property
    def sim_time(self):
        return self._steps / STEP_HZ
