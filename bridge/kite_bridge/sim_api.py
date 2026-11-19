"""The small interface the spike scripts use to talk to a simulator.

Two classes implement it: `fake_sim.FakeSim` (synthetic, for tests and dry runs) and
`beamng_adapter.BeamNGAdapter` (the real simulator). The scripts only see this interface.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

STEP_HZ = 60
SENSOR_KINDS = ("lidar", "camera", "radar", "gps", "imu")


@dataclass(frozen=True)
class Reading:
    """One poll result. `timestamp` is the simulator time in seconds the reading was taken;
    None means the poll returned nothing. `points` is the lidar point count, else None."""

    timestamp: float | None
    points: int | None = None


class Simulator(ABC):
    @abstractmethod
    def open(self) -> None:
        """Start or connect to the simulator."""

    @abstractmethod
    def close(self) -> None:
        """Release everything. Safe to call more than once and after a failed open."""

    @abstractmethod
    def versions(self) -> dict[str, str]:
        """Version strings the simulator and its Python library report."""

    @abstractmethod
    def list_models(self) -> list[str]:
        """Names of the vehicle models the simulator offers."""

    @abstractmethod
    def load_scenario(self, model: str, n_cars: int) -> list[str]:
        """Load the West Coast scenario with n_cars of `model` and start it.
        Returns the car names."""

    @abstractmethod
    def spawn_traffic(self, n: int) -> None:
        """Spawn up to n AI traffic cars. Raises if the simulator cannot."""

    @abstractmethod
    def attach_sensors(self, car: str) -> None:
        """Attach one sensor of each kind in SENSOR_KINDS to `car`."""

    @abstractmethod
    def poll(self, car: str, kind: str) -> Reading:
        """Read one sensor once."""

    @abstractmethod
    def step(self, n: int = 1) -> None:
        """Advance the simulator by n steps of 1 / STEP_HZ seconds."""

    @abstractmethod
    def set_ai_mode(self, car: str, mode: str) -> None:
        """Set the car's AI mode, for example "traffic" or "disabled"."""

    @abstractmethod
    def position(self, car: str) -> tuple[float, float, float]:
        """Ground-truth position of the car. For scoring only, never for driving."""

    @property
    @abstractmethod
    def sim_time(self) -> float:
        """Simulator seconds since the scenario started."""
