"""Demo 1: our car in traffic mode with AI traffic on West Coast, USA, streaming lidar and camera into Rerun.

By default (--mode free --stream) the simulator runs on its own. The lidar and camera are read
from shared memory with stream() and the radar, GPS, IMU and a Timer are polled, on the free-mode
wall-clock schedule. Each pass reads the due streamed sensors first, then at most one due poll.
Each lidar scan and camera image read is logged at its Timer time, as the simulator gave it. The
points are not coloured and no ground truth is logged, because reading our car's position is a
request that would hold up the lidar. The run then prints the measuring table of
spike_sensors.py --mode free, which says how late each sensor's reads came. --no-stream polls the
lidar and camera too. --rates sets the reads per wall second.

--mode lockstep runs the demo as before: it logs, once per simulator step on one timeline, lidar
points coloured by distance from our car (near blue, far red), our car's position and the camera
image. Our car's and the traffic cars' positions come from simulator state, so they are ground
truth, logged for display only under `ground_truth/`, apart from the sensor layers. A step where
the lidar or the camera gives no reading logs nothing for that sensor; the run says how many steps
that was. --mode lockstep polls; it takes neither --stream nor --rates.

--save <name.rrd> writes the recording to that file inside --out and does not open a viewer;
open it later with `rerun <file>`. After Ctrl+C the file keeps everything logged before it.
--fake uses the fake simulator and a recorder that keeps nothing: it opens no viewer, imports
no Rerun and writes no file, even with --save.
"""

import argparse
import sys

from kite_bridge import common, rerun_view


def _save_name(text):
    try:
        return rerun_view.recording_name(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _add_options(parser):
    common.add_live_options(parser)
    parser.add_argument("--save", type=_save_name, metavar="NAME.rrd",
                        help="write the recording to NAME.rrd inside --out instead of opening a viewer")


def _check(args):
    if args.mode == common.LOCKSTEP and (args.stream or args.rates is not None):
        return "--stream and --rates need --mode free; --mode lockstep polls the lidar and camera after every step"
    return None


def make_recorder(save_path, fake):
    if fake:
        return rerun_view.NullRecorder()
    return rerun_view.RerunRecorder(save_path)


def run(args):
    save_path = rerun_view.recording_path(args.out, args.save) if args.save else None
    common.print_sensor_settings(args.stream)
    with common.session(args) as sim:
        car = common.spawn(sim, args, 1)[0]
        show_traffic = common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(car, "traffic")
        if args.mode == common.FREE:
            report = _run_free(sim, car, args, save_path)
        else:
            report = _run_lockstep(sim, car, args, save_path, show_traffic)
        sim.set_ai_mode(car, "disabled")
    print("\n".join(report))
    if save_path is None:
        return
    if args.fake:
        print(f"--fake keeps nothing; the recording would have gone to {save_path}")
    else:
        print(f"wrote {save_path}")


def _run_free(sim, car, args, save_path):
    logged = {"lidar": 0, "camera": 0}
    recorder = make_recorder(save_path, args.fake)

    def log(key, reading):
        if key[1] in logged:
            logged[key[1]] += rerun_view.log_reading(recorder, key[1], reading)

    try:
        result = common.measure_sensors(sim, [car], args.seconds, poll_rates=args.rates, stream=args.stream,
                                        mode=common.FREE, on_reading=log)
    finally:
        recorder.close()
    print(common.format_result(result))
    return [f"logged {logged['lidar']} lidar scans and {logged['camera']} camera images at Timer times",
            "no ground truth logged in free mode, because reading our car's or the other cars' positions is a "
            "request that would hold up the lidar"]


def _run_lockstep(sim, car, args, save_path, show_traffic):
    no_lidar = no_image = 0
    sim.attach_sensors(car)
    steps = common.steps_for(args.seconds)
    recorder = make_recorder(save_path, args.fake)
    try:
        for _ in range(steps):
            sim.step(1)
            others = None
            if show_traffic:
                try:
                    others = sim.other_positions()
                except TimeoutError:
                    raise
                except Exception as error:
                    print(f"other cars not logged ({type(error).__name__}: {error})")
                    show_traffic = False
            lidar = sim.lidar_points(car)
            image = sim.camera_image(car)
            no_lidar += lidar is None
            no_image += image is None
            rerun_view.log_step(recorder, sim.sim_time, sim.position(car), lidar, image, others)
    finally:
        recorder.close()
    report = [f"logged {steps} steps"]
    if no_lidar:
        report.append(f"lidar gave no reading on {no_lidar} of {steps} steps; no lidar was logged for them")
    if no_image:
        report.append(f"camera gave no reading on {no_image} of {steps} steps; no image was logged for them")
    return report


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, _add_options, _check))


if __name__ == "__main__":
    sys.exit(main())
