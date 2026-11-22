# viewer

`log_to_rerun.py` shows a v0 log (`docs/LOG-FORMAT.md`) in Rerun: ground truth, measurements and
tracks on one timeline `t`. Python standard library only, apart from `rerun`, which only
`rerun_sink.py` imports, and only when it makes the viewer or recording.

The reader, the scene logic and the command line are tested with a fake logger. The Rerun calls
have not run against an installed Rerun yet. Each carries an `# ASSUMPTION:` comment in
`rerun_sink.py`, and your first run decides.

## Demo 1's second clip

Three commands, run in order, from a fresh build. Each block starts in the repo with `.venv`
activated; see `docs/SPIKE-0B.md`, "Set up once", to create it and install `rerun-sdk`. The
commands are for PowerShell on Windows; on Linux and macOS use `source .venv/bin/activate` and drop
the `.exe` and the leading `.\`.

1. Build with the `dev` preset, then generate a synthetic log:

   ```powershell
   cd <your repo folder>
   .\.venv\Scripts\Activate.ps1
   cmake --preset dev
   cmake --build --preset dev
   .\build\dev\tools\kite_gen.exe --seed 42 --vehicles 5 --duration 20 --dt 0.1 --noise 0.5 --pd 0.9 --out build\dev\demo.log
   ```

2. Run the Kalman tracker over it. It reads only the `meas` records and writes the log back with
   its `track` records added. Add `--tracker baseline` to run the baseline tracker instead.

   ```powershell
   cd <your repo folder>
   .\.venv\Scripts\Activate.ps1
   .\build\dev\tools\kite_track.exe --in build\dev\demo.log --out build\dev\demo_tracks.log
   ```

3. Show it. This opens a live viewer:

   ```powershell
   cd <your repo folder>
   .\.venv\Scripts\Activate.ps1
   python viewer/log_to_rerun.py build/dev/demo_tracks.log
   ```

   To write a recording instead, add `--save`. It goes to `recordings/demo.rrd`; `--out <folder>`
   changes the folder, and `--save` takes a name inside it, never a path:

   ```powershell
   cd <your repo folder>
   .\.venv\Scripts\Activate.ps1
   python viewer/log_to_rerun.py build/dev/demo_tracks.log --save demo.rrd
   ```

   Open it with `rerun recordings/demo.rrd`.

`--out` inside the repo is refused (exit 2) unless it is `recordings/` or `scratch/`, both
git-ignored. `.rrd` files are never committed. A log with a bad line stops the run with exit 1 and
a message that names the file and the line.

## What you see

| Entity | Colour | Meaning |
|---|---|---|
| `ground_truth/actors` | orange | The true actors' positions, labelled with `actor_id`. For checking only: the tracker never sees them. Hide them by switching off `ground_truth` in the viewer's entity list. |
| `measurements` | grey | The noisy detections the tracker was given. They have no identity. |
| `tracks` | one colour per `track_id` | The tracker's tracks, labelled with `track_id`. A track keeps its colour for the whole run. |
| `tracks/velocity` | the track's colour | Five points along each track's velocity, at 0.1 to 0.5 s ahead. They are dots in a row, not a drawn line, because a line needs a Rerun call this repo does not use yet. |

Track colours come from a fixed list of 12 (blue, green, magenta, cyan, red, purple, lime, pink,
teal, lavender, brown, white), chosen by `track_id` modulo 12, so ids 12 apart share a colour. None
is orange or grey. A `track_id` has no link to an `actor_id`: the same number can mean different
things in the two.

Every step logs all four entities, empty ones included, so a step with no detection shows none.
Positions are drawn at height 0 in metres. The `ego_id` field is ignored.

## Top-down view

The 3D view does not start top-down. Setting that would need a Rerun call that nothing in the repo
uses yet, so the script leaves the viewer's default camera. In the 3D view, drag to orbit until you
look straight down the z axis; every point is at height 0. The viewer menu entry for a fixed top
view has not been checked in Rerun 0.43.2.

## Where the data comes from

The logs in this demo are synthetic: `kite_gen` draws them from a seeded random generator and
`kite_track` adds tracker output. They are this project's own data, not simulator data, so nothing
here carries the simulator provenance marker that the bridge's recordings carry.

## Tests

From the repo root: `python -m unittest discover -s viewer -v`. They read the committed logs in
`sim/tests/data/`, use a fake logger and never import `rerun`, open a viewer or build anything.
