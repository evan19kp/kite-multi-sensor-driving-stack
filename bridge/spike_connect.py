"""Spike 1: connect to the simulator, spawn our car, drive in traffic mode and print its position.

Prints the BeamNG.tech and BeamNGpy versions the simulator reports. Writes connect.json to
--out. The positions in it are ground truth for scoring; they never drive anything.
"""

import math
import sys

from kite_bridge import common
from kite_bridge.sim_api import STEP_HZ


def run(args):
    with common.session(args) as sim:
        versions = sim.versions()
        for name, version in versions.items():
            print(f"{name} version: {version}")
        model = common.resolve_model(sim, args.model)
        car = sim.load_scenario(model, 1)[0]
        print(f"spawned {car} ({model})")
        traffic = common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(car, "traffic")
        ground_truth = []
        for second in range(1, math.ceil(args.seconds) + 1):
            sim.step(STEP_HZ)
            x, y, z = sim.position(car)
            ground_truth.append({"t": second, "pos": [x, y, z]})
            print(f"t={second}s position=({x:.2f}, {y:.2f}, {z:.2f})")
        sim.set_ai_mode(car, "disabled")
    common.write_json(args.out, "connect.json", {
        "versions": versions, "model": model, "traffic_spawned": traffic,
        "ground_truth": ground_truth})


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv))


if __name__ == "__main__":
    sys.exit(main())
