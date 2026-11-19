"""Spike 2: attach lidar, camera, radar, GPS and IMU, poll each every step, and report.

Per sensor: effective update rate, poll latency (mean, p50, p95, max), empty and repeated
readings, and lidar points per scan. Writes the same numbers to sensors.json in --out.
"""

import sys

from kite_bridge import common


def run(args):
    with common.session(args) as sim:
        cars = common.spawn(sim, args, 1)
        common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(cars[0], "traffic")
        result = common.measure_sensors(sim, cars, args.seconds)
        sim.set_ai_mode(cars[0], "disabled")
        print(common.format_result(result))
        common.write_json(args.out, "sensors.json", {"model": args.model, "config": args.config, **result})


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv))


if __name__ == "__main__":
    sys.exit(main())
