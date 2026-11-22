"""Spike 2: attach lidar, camera, radar, GPS and IMU, poll each at its --rates rate, and report.

Per sensor: effective update rate, poll latency (mean, p50, p95, max), empty and repeated
readings, lidar points per scan and radar detections per poll. Also the wall time split into
waiting for steps, sensor reads and our own processing, the real-time factor, and per sensor the
configured poll rate, the new readings and the bytes read per simulated second, with a `possible
loss` note when a sensor read slower than every step gave fewer new readings than it was asked
for. `--rates all=60` polls every sensor after every step. Prints one `sensor:` line per sensor
with the settings the bridge passes, before the run. Writes the same numbers and the settings to
sensors.json in --out. A sensor whose poll raises or returns an unexpected format is counted
under `unexp`; the run goes on with the other sensors. --dump-raw N also writes the first N raw
GPS and IMU readings to raw_gps_imu.json. --stream reads the lidar and camera from shared memory
with stream() and counts what it drops under `suspect`; in Evan's 0B.3 and 0B.4 runs on the real
simulator a streamed read took 0.75 to 1.02 ms. --mode free lets the simulator run on its
own, polls on a wall-clock schedule and reads simulated time from a Timer sensor; its default
rate for the GPS and IMU is 5 per second since R40.
--lidar-pos x,y,z sets where the lidar sits on our car (vehicle space, metres); it shows on the
`sensor:` line. Every sensor is created with is_visualised=False. That this keeps BeamNG from
drawing them into the world and the camera frames is not yet confirmed on the real simulator.
--visualise-sensors turns the drawing back on for the lidar, radar, IMU and GPS (never
the camera), for debugging. Every run counts, approximately, the lidar points within --ego-radius
metres of our car in x and y, measured from the GPS position, per new scan, and prints their mean
and maximum and writes them to sensors.json under `self_hits_approx` on the lidar row. In Evan's
0B.3 runs the approximate count per scan had a mean of 233.0 in three runs and 233.2 in one at
1.7 m (maximum 234 to 275) and was 0 at 1.9, 2.1 and 2.3 m; 1.9 m is the default.
"""

import sys

from kite_bridge import common
from kite_bridge.sensor_settings import settings_table


def run(args):
    common.print_sensor_settings(args.stream, args.lidar_pos, args.visualise_sensors)
    with common.session(args) as sim:
        cars = common.spawn(sim, args, 1)
        common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(cars[0], "traffic")
        dump = common.RawDump(args.dump_raw)
        result = common.measure_sensors(sim, cars, args.seconds, dump=dump, poll_rates=args.rates,
                                        stream=args.stream, mode=args.mode, lidar_pos=args.lidar_pos,
                                        self_hits=common.make_self_hits(args), stagger=True,
                                        visualise=args.visualise_sensors)
        sim.set_ai_mode(cars[0], "disabled")
        print(common.format_result(result))
        common.write_json(args.out, "sensors.json", {
            "model": args.model, "config": args.config,
            "sensor_settings": settings_table(args.stream, args.lidar_pos, args.visualise_sensors), **result})
        if args.dump_raw:
            common.write_raw_dump(args.out, args.dump_raw, [(len(cars), dump)])


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, common.add_measure_options))


if __name__ == "__main__":
    sys.exit(main())
