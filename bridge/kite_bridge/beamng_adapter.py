"""The real simulator behind `sim_api.Simulator`, through BeamNGpy.

This is the only module that imports `beamngpy`, and it does so lazily, inside methods, so
importing the module needs no simulator and no installed packages. Every BeamNGpy call lives
here. Calls not verified against BeamNGpy 1.36 are marked ASSUMPTION; the first real run
checks them.
"""

import importlib.metadata

from kite_bridge.sim_api import POLL_TIME, STEP_HZ, Reading, Simulator

PORT = 25252
SCENARIO_LEVEL = "west_coast_usa"
SPAWN_POS = (-717.121, 101.0, 118.675)
SPAWN_ROT_QUAT = (0.0, 0.0, 0.3826834, 0.9238795)
SPAWN_SPACING_M = 5.0  # ASSUMPTION: extra cars start this far apart along x, on or near the road
FORWARD = (0, -1, 0)
UP = (0, 0, 1)


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
        failures = []
        for sensor in self._sensors.values():
            try:
                sensor.remove()
            except Exception as error:
                failures.append(error)
        self._sensors.clear()
        if self._bng is not None:
            bng, self._bng = self._bng, None
            bng.close()
        if failures:
            raise failures[0]

    def versions(self):
        try:
            tech = str(self._bng.get_tech_version())  # ASSUMPTION: this call exists in 1.36
        except Exception as error:
            tech = f"unavailable ({type(error).__name__}: {error})"
        return {"BeamNG.tech": tech, "BeamNGpy": importlib.metadata.version("beamngpy")}

    def list_models(self):
        return [str(name) for name in self._bng.vehicles.get_available()]  # ASSUMPTION: this is the 1.36 call, and it yields names

    def load_scenario(self, model, n_cars):
        from beamngpy import Scenario, Vehicle

        scenario = Scenario(SCENARIO_LEVEL, "kite_spike", description="Kite sensor spike")
        x, y, z = SPAWN_POS
        for i in range(n_cars):
            name = f"kite{i + 1}"
            vehicle = Vehicle(name, model=model, license="KITE")
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
        self._sensors.update({
            (car, "lidar"): Lidar(f"lidar_{car}", bng, vehicle, requested_update_time=0.01,
                                  is_using_shared_memory=True, is_360_mode=True),
            (car, "camera"): Camera(f"camera_{car}", bng, vehicle, requested_update_time=0.05,
                                    is_using_shared_memory=True, resolution=(640, 360),
                                    pos=(0, -0.5, 1.5), dir=FORWARD, field_of_view_y=70),
            (car, "radar"): Radar(f"radar_{car}", bng, vehicle, requested_update_time=0.01,
                                  pos=(0, 0, 1.7), dir=FORWARD, up=UP),
            (car, "gps"): GPS(f"gps_{car}", bng, vehicle, pos=(0, 1.5, 2.0),
                              ref_lon=0.0, ref_lat=0.0),
            (car, "imu"): AdvancedIMU(f"imu_{car}", bng, vehicle, pos=(0, 0, 0.5),
                                      dir=FORWARD, up=UP, gfx_update_time=0.01,
                                      physics_update_time=0.01),
        })

    def poll(self, car, kind):
        raw = self._sensors[(car, kind)].poll()
        points = None
        if kind == "lidar" and raw:
            if not isinstance(raw, dict) or "pointCloud" not in raw:
                keys = sorted(map(str, raw)) if isinstance(raw, dict) else type(raw).__name__
                return Reading(None, unexpected=f"lidar reading has no 'pointCloud'; got {keys}")
            points = len(raw["pointCloud"])
            if points == 0:
                return Reading(None)
        latest = _latest(raw)
        if latest is None:
            return Reading(None)
        # ASSUMPTION: readings carry "time" in simulator seconds; without it the poll time is used
        if isinstance(latest, dict) and "time" in latest:
            return Reading(float(latest["time"]), points)
        return Reading(self.sim_time, points, POLL_TIME)

    def step(self, n=1):
        self._bng.control.step(n)
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


def _latest(raw):
    if isinstance(raw, list):
        return raw[-1] if raw else None
    if isinstance(raw, dict) and raw and all(isinstance(v, dict) for v in raw.values()):
        return list(raw.values())[-1]
    return raw or None
