"""Spike 2: attach lidar, camera, radar, GPS and IMU, poll each every step, and report.

Per sensor: effective update rate, poll latency (mean, p50, p95, max), empty and repeated
readings, lidar points per scan and radar detections per poll. Also the wall time split into
waiting for steps, sensor reads and our own processing, the real-time factor, and per sensor the
new readings and the bytes read per simulated second. Prints one `sensor:` line per sensor with
the settings the bridge passes, before the run. Writes the same numbers and the settings to
sensors.json in --out. A sensor whose poll raises or returns an unexpected format is counted
under `unexp`; the run goes on with the other sensors. --dump-raw N also writes the first N raw
GPS and IMU readings to raw_gps_imu.json.
"""

import sys

from kite_bridge import common
from kite_bridge.sensor_settings import SENSOR_SETTINGS


def run(args):
    common.print_sensor_settings()
    with common.session(args) as sim:
        cars = common.spawn(sim, args, 1)
        common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(cars[0], "traffic")
        dump = common.RawDump(args.dump_raw)
        result = common.measure_sensors(sim, cars, args.seconds, dump=dump)
        sim.set_ai_mode(cars[0], "disabled")
        print(common.format_result(result))
        common.write_json(args.out, "sensors.json", {"model": args.model, "config": args.config,
                                                     "sensor_settings": SENSOR_SETTINGS, **result})
        if args.dump_raw:
            common.write_raw_dump(args.out, args.dump_raw, [(len(cars), dump)])


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, common.add_measure_options))


if __name__ == "__main__":
    sys.exit(main())
