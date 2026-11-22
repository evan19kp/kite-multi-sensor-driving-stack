"""Shows a v0 log (`docs/LOG-FORMAT.md`) in Rerun: ground truth, measurements and tracks, with each
track's velocity, on one timeline `t`. See `viewer/README.md`.

    python viewer/log_to_rerun.py <log> [--save <name.rrd>] [--out <folder>]

Without `--save` a live viewer opens. With it the recording goes to `<out>/<name.rrd>` and no
viewer opens. Exit codes: 0 done, 1 the log or Rerun could not be used, 2 bad arguments.
"""

import argparse
import sys
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").is_file())
sys.path.insert(0, str(ROOT / "bridge"))

from kite_bridge.output_guard import out_refusal, repo_roots  # noqa: E402
from kite_bridge.rerun_view import recording_path  # noqa: E402
from kite_log import LogError, read_log  # noqa: E402
from rerun_sink import RerunSink  # noqa: E402
from scene import log_scene  # noqa: E402

DEFAULT_OUT = "recordings"


def parse_args(argv):
    parser = argparse.ArgumentParser(prog="log_to_rerun.py", description=__doc__.split("\n\n")[0])
    parser.add_argument("log", type=Path, help="the v0 log to show")
    parser.add_argument("--save", metavar="NAME.rrd",
                        help="write a recording named NAME.rrd inside --out instead of opening a viewer")
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="folder for the recording; inside the repo only recordings/ and scratch/ "
                             f"are allowed (default {DEFAULT_OUT}/)")
    args = parser.parse_args(argv)
    refusal = out_refusal(args.out, repo_roots())
    if refusal:
        parser.error(f"--out {refusal}")
    args.save_path = None
    if args.save is not None:
        try:
            args.save_path = recording_path(args.out, args.save)
        except ValueError as error:
            parser.error(f"--save {error}")
    return args


def main(argv=None, sink_factory=RerunSink):
    """Runs the command. `sink_factory(save_path)` makes the sink the scene is logged to."""
    args = parse_args(argv)
    try:
        log = read_log(args.log)
    except (LogError, OSError) as error:
        print(f"{args.log}: {error}", file=sys.stderr)
        return 1
    try:
        sink = sink_factory(args.save_path)
    except ImportError as error:
        print(f"Rerun is not installed ({error}); run: python -m pip install -r bridge/requirements.txt",
              file=sys.stderr)
        return 1
    try:
        summary = log_scene(sink, log)
    finally:
        sink.close()
    print(f"{args.log}: {summary.steps} time steps, {summary.gt} ground truth, {summary.meas} measurement "
          f"and {summary.tracks} track records")
    if args.save_path is not None:
        print(f"recording: {args.save_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
