"""Demo 1: our car in traffic mode with AI traffic on West Coast, USA, streaming lidar and camera into Rerun.

By default (--mode free --stream) the simulator runs on its own. The lidar and camera are read
from shared memory with stream() and the radar, GPS, IMU and a Timer are polled, on the free-mode
wall-clock schedule. Each pass reads the due streamed sensors first, then at most one due poll.
A lidar scan or camera image is logged at its Timer time only when its content is new. The
points are coloured by distance in x and y from world/ego, our car at the x, y the GPS
measures (the origin until it gives one), and the default layout's 3D view follows world/ego.
No ground truth is logged, because reading our car's position from the simulator is a request
that would hold up the lidar. The run then prints the measuring table of spike_sensors.py --mode free, which
says how late each sensor's reads came. --no-stream polls the lidar and camera too. --rates sets
the reads per wall second; the free-mode default for the GPS and IMU is 5 per second since R40.
In Evan's 0B.3 and 0B.4 runs on the real simulator a streamed lidar or camera read took 0.75 to
1.02 ms. --live-stride N shows every Nth lidar point in the live viewer.

--mode lockstep runs the demo as before: it logs, once per simulator step on one timeline, lidar
points coloured by distance from our car (near blue, far red), our car's position and the camera
image. Our car's and the traffic cars' positions come from simulator state, so they are ground
truth, logged for display only under `ground_truth/`, apart from the sensor layers. A step where
the lidar or the camera gives no reading logs nothing for that sensor; the run says how many steps
that was. A lidar or camera read that raises counts as `unexp` (one line with the count and the
first error) and the run goes on; a timeout stops the run. --mode lockstep polls; it takes
neither --stream nor --rates, and it does not count
self-hits.

--lidar-pos x,y,z sets where the lidar sits on our car (vehicle space, metres), in both modes. In
free mode the measuring table is followed by an approximate self-hit count: the lidar points
within --ego-radius metres of our car in x and y, measured from the GPS position, per new scan.
It is printed only, never logged to Rerun. The default height is 1.9 m. In Evan's 0B.3 runs the
approximate count per scan had a mean of 233.0 in three runs and 233.2 in one at 1.7 m (maximum
234 to 275) and was 0 at 1.9, 2.1 and 2.3 m.

Sensors are created with is_visualised=False. That the lidar's dots are then no longer drawn in
the BeamNG window or the camera frames is not yet confirmed on the real simulator. --visualise-sensors
turns BeamNG's drawing of the lidar, radar, IMU and GPS back on, for debugging; the camera is
never drawn.

--save <name.rrd> writes the recording to that file inside --out and does not open a viewer;
open it later with `rerun <file>`. It keeps every lidar point and stores each camera image as a
PNG, which is lossless. The PNGs are encoded on one thread beside the loop, through a queue of
rerun_view.PNG_QUEUE_FRAMES (8) frames; the loop adds its frame before it waits for space, so a
full queue can hold 9. When the queue is full the loop waits, and no frame is dropped. The run
prints the file's size per simulated second, the time spent encoding PNGs and how often and how
long the loop waited. After Ctrl+C every frame already queued is encoded and logged before the file
is closed. A second Ctrl+C while the file closes stops the wait and prints how many camera frames
were not saved; if the encoder had failed first, that line names the failure. If encoding fails,
the run stops with an error that says how many frames were not recorded. A real --save recording
with lidar and camera opened in an installed Rerun on 2026-11-19 (0B rerun, Run 4b, 354 MiB for
about 5 simulated s); the 0B.3 and 0B.4 recordings were 6.90 and 6.17 MB per simulated second.
--fake uses the fake simulator and a recorder that keeps nothing: it opens no viewer, imports
no Rerun and writes no file, even with --save.
"""

import argparse
import sys

from kite_bridge import common, measure, rerun_view
from kite_bridge.sim_api import STEP_HZ

BYTES_PER_MB = 1_000_000


def _save_name(text):
    try:
        return rerun_view.recording_name(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _stride(text):
    try:
        value = int(text)
    except ValueError:
        value = 0
    if value < 1:
        raise argparse.ArgumentTypeError(f"must be a whole number >= 1, got {text}")
    return value


def _add_options(parser):
    common.add_live_options(parser)
    parser.add_argument("--save", type=_save_name, metavar="NAME.rrd",
                        help="write the recording to NAME.rrd inside --out instead of opening a viewer; it keeps "
                             "every lidar point and stores camera images as PNG (lossless)")
    parser.add_argument("--live-stride", type=_stride, default=1, metavar="N",
                        help="show every Nth lidar point in the live viewer (default 1: every point); "
                             "a --save recording keeps every point")


def _check(args):
    if args.mode == common.LOCKSTEP and (args.stream or args.rates is not None):
        return "--stream and --rates need --mode free; --mode lockstep polls the lidar and camera after every step"
    if args.mode == common.LOCKSTEP and args.ego_radius is not None:
        return "--ego-radius needs --mode free; --mode lockstep does not count self-hits"
    if args.save and args.live_stride != 1:
        return "--live-stride thins the live viewer only; a --save recording keeps every lidar point"
    return None


def make_recorder(save_path, fake, live_stride):
    if fake:
        return rerun_view.NullRecorder()
    return rerun_view.RerunRecorder(save_path, live_stride, clock=common.wall_clock)


def run(args):
    save_path = rerun_view.recording_path(args.out, args.save) if args.save else None
    common.print_sensor_settings(args.stream, args.lidar_pos, args.visualise_sensors)
    with common.session(args) as sim:
        car = common.spawn(sim, args, 1)[0]
        show_traffic = common.start_traffic(sim, args.traffic)
        sim.set_ai_mode(car, "traffic")
        recorder = make_recorder(save_path, args.fake, args.live_stride)
        try:
            if args.mode == common.FREE:
                report, simulated_s = _run_free(sim, car, args, recorder)
            else:
                report, simulated_s = _run_lockstep(sim, car, args, recorder, show_traffic)
        except BaseException:
            _close_reporting_encode_error(recorder)
            raise
        recorder.close()
        sim.set_ai_mode(car, "disabled")
    if args.live_stride != 1:
        report.append(f"the live viewer showed 1 of every {args.live_stride} lidar points")
    print("\n".join(report))
    if save_path is None:
        return
    if args.fake:
        print(f"--fake keeps nothing; the recording would have gone to {save_path}")
        return
    if not save_path.is_file():
        raise common.SpikeError(f"no recording at {save_path} after it was closed")
    print(f"wrote {save_path}")
    stats = recorder.stats
    print("\n".join([*recording_lines(save_path.stat().st_size, simulated_s, stats.encoded, stats.encode_s),
                     backlog_line(stats)]))


def _close_reporting_encode_error(recorder):
    try:
        recorder.close()
    except rerun_view.EncodeError as error:
        print(f"error: EncodeError: {error}", file=sys.stderr)


def recording_lines(nbytes, simulated_s, encoded, encode_s):
    """What a saved recording cost: its size on disk after close, per simulated second, and the
    time spent encoding camera images as PNG."""
    per_s = measure.per_simulated_second(nbytes / BYTES_PER_MB, simulated_s)
    per_s_text = "n/a" if per_s is None else f"{per_s:.2f}"
    return [f"recording: {nbytes} bytes on disk after close, {nbytes / BYTES_PER_MB:.2f} MB (1 MB = 10^6 bytes) "
            f"over {simulated_s:.3f} simulated s: {per_s_text} MB per simulated s",
            f"PNG encoding: {encoded} camera images in {encode_s:.3f} s on the encoder thread"]


def backlog_line(stats):
    return (f"PNG backlog: the loop found the queue of {rerun_view.PNG_QUEUE_FRAMES} frames full {stats.backlog_waits} "
            f"times and waited {stats.backlog_wait_s:.3f} s in all for space; no frame was dropped")


def _run_free(sim, car, args, recorder):
    logged = {"lidar": 0, "camera": 0}
    not_logged = {"lidar": 0, "camera": 0}
    no_position = 0
    ego = None

    def log(key, reading):
        nonlocal ego, no_position
        kind = key[1]
        if kind == "gps":
            position = rerun_view.gps_position(reading.raw)
            if position is None:
                no_position += 1
            else:
                ego = position
        elif kind in logged:
            if rerun_view.log_reading(recorder, kind, reading, ego):
                logged[kind] += 1
            else:
                not_logged[kind] += 1

    result = common.measure_sensors(sim, [car], args.seconds, poll_rates=args.rates, stream=args.stream,
                                    mode=common.FREE, on_reading=log, lidar_pos=args.lidar_pos,
                                    self_hits=common.make_self_hits(args), stagger=True,
                                    visualise=args.visualise_sensors)
    print(common.format_result(result))
    report = [f"logged {logged['lidar']} lidar scans and {logged['camera']} camera images at Timer times, "
              "each only when its content was new"]
    if not_logged["lidar"]:
        report.append(f"lidar gave {not_logged['lidar']} new scans with a non-finite point; not logged")
    if not_logged["camera"]:
        report.append(f"camera gave {not_logged['camera']} new readings without a colour image; not logged")
    if no_position:
        report.append(f"gps gave {no_position} new polls without a position in their latest reading; "
                      f"{rerun_view.EGO_FRAME_PATH} did not move for them")
    report.append("no ground truth logged in free mode, because reading our car's or the other cars' positions "
                  "is a request that would hold up the lidar")
    return report, result["timing"]["simulated_s"]


def _run_lockstep(sim, car, args, recorder, show_traffic):
    no_lidar = no_image = 0
    lidar_raised, image_raised = [], []
    sim.attach_sensors(car, lidar_pos=args.lidar_pos, visualise=args.visualise_sensors)
    steps = common.steps_for(args.seconds)
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
        lidar, lidar_error = _try_read(sim.lidar_points, car)
        image, image_error = _try_read(sim.camera_image, car)
        if lidar_error:
            lidar_raised.append(lidar_error)
        else:
            no_lidar += lidar is None
        if image_error:
            image_raised.append(image_error)
        else:
            no_image += image is None
        rerun_view.log_step(recorder, sim.sim_time, sim.position(car), lidar, image, others)
    report = [f"logged {steps} steps"]
    if no_lidar:
        report.append(f"lidar gave no reading on {no_lidar} of {steps} steps; no lidar was logged for them")
    if no_image:
        report.append(f"camera gave no reading on {no_image} of {steps} steps; no image was logged for them")
    for name, raised in (("lidar", lidar_raised), ("camera", image_raised)):
        if raised:
            report.append(f"unexp: {name} read raised on {len(raised)} of {steps} steps, first: {raised[0]}; "
                          f"nothing was logged for them")
    return report, steps / STEP_HZ


def _try_read(read, car):
    """`read(car)` and None, or None and the error as text when it raises. A timeout is never
    caught (R29): it stops the run."""
    try:
        return read(car), None
    except TimeoutError:
        raise
    except Exception as error:
        return None, f"{type(error).__name__}: {error}"


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv, _add_options, _check))


if __name__ == "__main__":
    sys.exit(main())
