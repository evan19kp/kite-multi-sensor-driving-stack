"""Spike 3: run 1, 2 and 4 cars, each with all five sensors, one case after another.

Each case starts a fresh simulator session. Prints steps per second, the wall time split, the
real-time factor and every sensor's rate and fresh-data counts per case. Every sensor is polled
at its --rates rate. Writes multicar.json to --out, with the sensor settings under
`sensor_settings`, as sensors.json has them. --dump-raw N also writes the first N raw GPS and IMU
readings per car, for every case, to raw_gps_imu.json. --stream reads the lidar and camera from
shared memory with stream() and counts what it drops under `suspect`; in Evan's 0B.3 and 0B.4
runs on the real simulator a streamed read took 0.75 to 1.02 ms. --mode free lets the simulator
run on its own, polls on a wall-clock schedule and reads simulated time from a Timer sensor; its
default rate for the GPS and IMU is 5 per second since R40. --lidar-pos x,y,z sets where each car's
lidar sits (vehicle space, metres) and --ego-radius the radius of the approximate self-hit count,
which is printed and written to multicar.json per car and case, as in spike_sensors.py. In
Evan's 0B.3 runs the approximate count per scan had a mean of 233.0 in three runs and 233.2 in one
at 1.7 m (maximum 234 to 275) and was 0 at 1.9, 2.1 and 2.3 m; 1.9 m is the default.
--visualise-sensors turns BeamNG's drawing of the lidar, radar, IMU and GPS back on, for debugging.
Sensors are created with is_visualised=False by default; that the drawing is then gone is not yet
confirmed on the real simulator.
"""

import sys

from kite_bridge import common
from kite_bridge.sensor_settings import settings_table

CAR_COUNTS = (1, 2, 4)


def run(args):
    common.print_sensor_settings(args.stream, args.lidar_pos, args.visualise_sensors)
    cases = []
    dumps = []
    for n_cars in CAR_COUNTS:
        with common.session(args) as sim:
            cars = common.spawn(sim, args, n_cars)
            common.start_traffic(sim, args.traffic)
            for car in cars:
                sim.set_ai_mode(car, "traffic")
            dump = common.RawDump(args.dump_raw)
            result = common.measure_sensors(sim, cars, args.seconds, dump=dump, poll_rates=args.rates,
                                            stream=args.stream, mode=args.mode, lidar_pos=args.lidar_pos,
                                            self_hits=common.make_self_hits(args), stagger=True,
                                            visualise=args.visualise_sensors)
            for car in cars:
                sim.set_ai_mode(car, "disabled")
            print(f"\n{common.format_result(result)}")
            cases.append(result)
            dumps.append((n_cars, dump))
            common.write_json(args.out, "multicar.json", {
                "model": args.model, "config": args.config,
                "sensor_settings": settings_table(args.stream, args.lidar_pos, args.visualise_sensors),
                "cases": cases})
            if args.dump_raw:
                common.write_raw_dump(args.out, args.dump_raw, dumps)


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, common.add_measure_options))


if __name__ == "__main__":
    sys.exit(main())
