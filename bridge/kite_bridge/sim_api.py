import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

STEP_HZ = 60
SENSOR_KINDS = ("lidar", "camera", "radar", "gps", "imu")
BUFFERED_KINDS = ("gps", "imu")
SENSOR_TIME = "sensor"
POLL_TIME = "poll"


@dataclass(frozen=True)
class Reading:
    """One poll result. `timestamp` is the simulator time in seconds; None means the poll
    returned nothing. `time_source` says where the timestamp came from: SENSOR_TIME when the
    reading carried its own time, POLL_TIME when it had none and the time of the poll was used
    (a rate computed from those timestamps is the polling rate, not the sensor's). `points` is
    the lidar point count, else None. `unexpected` is a message when the poll raised or the
    reading was not in the format the adapter expects; such a reading has no timestamp and is
    not an empty poll.
    `count` is how many readings the poll returned; `timestamp` is the latest one's. It means
    nothing for an empty poll. `detections` is the number of rows in a radar array, else None.
    `raw` is the poll result itself, kept only so the run can size and fingerprint it after the
    poll's timing has stopped; it is left out of comparison and repr, and never truth-tested.
    BUFFERED_KINDS are the sensors BeamNGpy 1.36 does not stream: they buffer readings between
    polls, so one poll can return several."""

    timestamp: float | None
    points: int | None = None
    time_source: str = SENSOR_TIME
    unexpected: str | None = None
    count: int = 1
    detections: int | None = None
    raw: object = field(default=None, compare=False, repr=False)


def remove_all(sensors):
    """Calls `remove()` on every sensor in `sensors`, a dict keyed by (car, kind), prints each
    failure to stderr and raises the first after trying them all. A timeout is printed and raised
    at once: the connection is then out of step, so every later removal would wait for its
    timeout too."""
    failure = None
    for (car, kind), sensor in sensors.items():
        try:
            sensor.remove()
        except Exception as error:
            print(f"warning: could not remove {car} {kind}: {type(error).__name__}: {error}", file=sys.stderr)
            if isinstance(error, TimeoutError):
                raise
            failure = failure or error
    if failure is not None:
        raise failure


class Simulator(ABC):
    @abstractmethod
    def open(self) -> None:
        """Start or connect to the simulator."""

    @abstractmethod
    def remove_sensors(self) -> None:
        """Remove every attached sensor, as `remove_all` does."""

    @abstractmethod
    def close(self) -> None:
        """Ask the simulator to quit. Safe after a failed open."""

    @abstractmethod
    def end_process(self) -> bool:
        """End the simulator process the library launched. Returns False, ending nothing, when
        the library launched none (for example, it connected to a simulator already running)."""

    @abstractmethod
    def versions(self) -> dict[str, str]:
        """The Python library's version lines, label to value: its package version and its
        protocol version. The simulator's own version is not among them."""

    @abstractmethod
    def list_models(self) -> list[str]:
        """Names of the vehicle models the simulator offers."""

    @abstractmethod
    def load_scenario(self, model: str, n_cars: int, config: str | None = None) -> list[str]:
        """Load the West Coast scenario with n_cars of `model` and start it. `config` is the
        part configuration file path; None spawns the model's default configuration.
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
    def lidar_points(self, car: str) -> list[tuple[float, float, float]] | None:
        """The car's latest lidar scan as (x, y, z) points in world metres. Needs the sensors
        attached. None means the lidar gave no reading, which includes a scan with no points."""

    @abstractmethod
    def camera_image(self, car: str) -> list[list[tuple[int, int, int]]] | None:
        """The car's latest camera image as rows of (red, green, blue) pixels, 0 to 255.
        Needs the sensors attached. None means the camera gave no reading."""

    @abstractmethod
    def other_positions(self) -> list[tuple[float, float, float]]:
        """Ground-truth positions of every car that is not ours (the traffic). For display
        only, never for driving. Raises if the simulator cannot say."""

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
