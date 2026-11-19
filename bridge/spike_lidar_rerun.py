"""Demo 1: our car in traffic mode with AI traffic on West Coast, USA, streaming lidar and camera into Rerun.

Opens a Rerun viewer and logs, once per simulator step on one timeline: lidar points coloured
by distance from our car (near blue, far red), our car's position and the camera image. The
traffic cars' positions are simulator ground truth, logged for display only under
`ground_truth/other_cars`, apart from the sensor layers.

--save <name.rrd> writes the recording to that file inside --out and does not open a viewer;
open it later with `rerun <file>`. --fake uses the fake simulator and a recorder that keeps
nothing: it opens no viewer, imports no Rerun and writes no file, even with --save.
"""

import argparse
import sys
from pathlib import Path

from kite_bridge import common, rerun_view


def _save_name(text):
    try:
        return rerun_view.recording_name(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _add_save(parser):
    parser.add_argument("--save", type=_save_name, metavar="NAME.rrd",
                        help="write the recording to NAME.rrd inside --out instead of opening a viewer")


def make_recorder(save_path, fake):
    if fake:
        return rerun_view.NullRecorder()
    return rerun_view.RerunRecorder(save_path)


def run(args):
    save_path = Path(args.out) / args.save if args.save else None
    with common.session(args) as sim:
        model = common.resolve_model(sim, args.model)
        car = sim.load_scenario(model, 1)[0]
        show_traffic = common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(car, "traffic")
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
                    except Exception as error:
                        print(f"other cars not logged ({type(error).__name__}: {error})")
                        show_traffic = False
                rerun_view.log_step(recorder, sim.sim_time, sim.position(car), sim.lidar_points(car),
                                    sim.camera_image(car), others)
        finally:
            recorder.close()
        sim.set_ai_mode(car, "disabled")
    print(f"logged {steps} steps")
    if save_path is None:
        return
    if args.fake:
        print(f"--fake keeps nothing; the recording would have gone to {save_path}")
    else:
        print(f"wrote {save_path}")


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, _add_save))


if __name__ == "__main__":
    sys.exit(main())
