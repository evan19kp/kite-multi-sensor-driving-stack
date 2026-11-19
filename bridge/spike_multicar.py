"""Spike 3: run 1, 2 and 4 cars, each with all five sensors, one case after another.

Each case starts a fresh simulator session. Prints steps per second, the wall time split, the
real-time factor and every sensor's rate and fresh-data counts per case. Writes multicar.json to
--out. --dump-raw N also writes the first N raw GPS and IMU readings per car, for every case, to
raw_gps_imu.json.
"""

import sys

from kite_bridge import common

CAR_COUNTS = (1, 2, 4)


def run(args):
    common.print_sensor_settings()
    cases = []
    dumps = []
    for n_cars in CAR_COUNTS:
        with common.session(args) as sim:
            cars = common.spawn(sim, args, n_cars)
            common.start_traffic(sim, args.traffic)
            for car in cars:
                sim.set_ai_mode(car, "traffic")
            dump = common.RawDump(args.dump_raw)
            result = common.measure_sensors(sim, cars, args.seconds, dump=dump)
            for car in cars:
                sim.set_ai_mode(car, "disabled")
            print(f"\n{common.format_result(result)}")
            cases.append(result)
            dumps.append((n_cars, dump))
            common.write_json(args.out, "multicar.json", {"model": args.model, "config": args.config, "cases": cases})
            if args.dump_raw:
                common.write_raw_dump(args.out, args.dump_raw, dumps)


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, common.add_measure_options))


if __name__ == "__main__":
    sys.exit(main())
