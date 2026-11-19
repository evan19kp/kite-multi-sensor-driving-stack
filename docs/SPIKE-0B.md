# Run the Sprint 0B spike

This guide is for Evan. It tells you how to run the four bridge scripts against BeamNG.tech on your machine and what to send back to the director. Nothing in this repo has run against the real simulator yet, so expect some BeamNGpy calls to fail. A failure with the full error text is a useful result.

The commands are for PowerShell. Run every command from the repo root, the folder that contains `AGENTS.md`.

Your simulator is BeamNG.tech 0.39.4.0. `bridge/requirements.txt` pins `beamngpy==1.36` and leaves `rerun-sdk` unpinned.

## Set up once

1. Create a virtual environment named `.venv` in the repo root. Git ignores `.venv/`.

   ```powershell
   python -m venv .venv
   ```

2. Activate it.

   ```powershell
   .\.venv\Scripts\Activate.ps1
   ```

   If PowerShell refuses to run the script because of its execution policy, allow scripts for this window only, then activate again.

   ```powershell
   Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
   ```

3. Install the packages.

   ```powershell
   python -m pip install -r bridge/requirements.txt
   ```

4. Record the installed versions. You send this output back with the first run.

   ```powershell
   python --version
   python -m pip show beamngpy rerun-sdk
   ```

Activate the environment again (step 2) in every new PowerShell window.

## What every script does

All four scripts take these options. `--home` is required for a real run.

| Option | Default | Meaning |
|---|---|---|
| `--home "<your BeamNG.tech folder>"` | none | The folder of your BeamNG.tech install. Write your real folder in place of the placeholder. |
| `--headless` | off | Run the simulator without a window. |
| `--seconds` | 30 | Simulated seconds to run, not wall-clock seconds. |
| `--traffic` | 8 | AI traffic cars to spawn. |
| `--out` | `recordings/` | Folder for the output files. |
| `--model` | the single match found | Our car's vehicle model. |
| `--fake` | off | Use the fake simulator. Do not use it for the real run. |

Each script starts BeamNG.tech itself and closes it when the run ends. Close any BeamNG.tech that is already running before you start a script. The bridge connects to the simulator on a fixed local port, so a second copy may block the connection.

Without `--home` and without `--fake`, a script prints a usage error and exits with code 2. If `--home` is not a folder, it prints `error: --home is not a folder` and exits with code 1. Any other failure prints an error and exits with code 1.

### Choosing the car model

Unless you pass `--model`, a script asks the simulator for its vehicle models and picks the one whose name contains `tograc` or `cherrier`. It prints a `model:` line for that match.

If no model or several models match, the script prints every `model:` line it found and stops with an error that tells you to pass `--model`. Run it again with one of the printed names:

```powershell
python bridge/spike_connect.py --home "<your BeamNG.tech folder>" --model <name>
```

If the script prints `could not list vehicle models`, the BeamNGpy call for that list failed. Send the error back (see "If a call fails"), and pass `--model` with a model name you know.

### How to read the output

The numbers in this guide come from `--fake` runs. The fake simulator is synthetic, so the version lines read `fake` and the rates, counts and step speed will differ on the real simulator. The shape of the output is what carries over.

## Run 1: connect (`spike_connect.py`)

This script connects to the simulator, spawns our car, drives it in traffic mode, and prints its position once per simulated second.

```powershell
python bridge/spike_connect.py --home "<your BeamNG.tech folder>"
```

### What it prints

Version lines, a `model:` line, a `spawned` line, then one `t=` line per second, then the path of the file it wrote. With `--seconds 2` on the fake simulator:

```
BeamNG.tech version: fake
BeamNGpy version: fake
model: fake_cherrier_tograc
spawned kite1 (fake_cherrier_tograc)
t=1s position=(10.00, 0.00, 0.00)
t=2s position=(20.00, 0.00, 0.00)
wrote scratch\R20\out\connect.json
```

On the real run, expect 30 `t=` lines and `wrote recordings\connect.json`. The two version lines show what the simulator reports. If AI traffic cannot start, the script prints `traffic not spawned (...)` with the error and continues with our car alone.

The positions are ground truth from the simulator. They go only into `connect.json`, apart from any sensor data.

### How long it takes

The run covers 30 simulated seconds, plus the time the simulator needs to start. Startup time is unknown. A minute or so is a reasonable guess. Wall-clock time for the 30 simulated seconds depends on how fast the simulator steps, which is one of the things this spike measures.

### What to send back

- The full printed output, including both version lines.
- `recordings/connect.json`.
- The `pip show` output from setup.
- Any error, in full (see "If a call fails").

## Run 2: sensors (`spike_sensors.py`)

This script attaches a lidar, a camera, a radar, GPS and an IMU to our car. It polls every sensor once per simulator step, 60 times per simulated second, and reports the rates and delays.

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>"
```

### What it prints

A `model:` line, a summary line, a table with one row per sensor, and footer lines. With `--seconds 2` on the fake simulator:

```
model: fake_cherrier_tograc
cars: 1  steps per second: 3037960.5
car    sensor   polls  rate_hz   time  per_poll  mean_ms  p50_ms  p95_ms  max_ms  empty  repeat  unexp  points
kite1  lidar      120    10.00 sensor         1     0.00    0.00    0.00    0.01      0      99      0    1000
kite1  camera     120    20.00 sensor         1     0.00    0.00    0.00    0.00      0      79      0     n/a
kite1  radar      120    20.00 sensor         1     0.00    0.00    0.00    0.00      0      79      0     n/a
kite1  gps        120    10.00 sensor         1     0.00    0.00    0.00    0.00      0      99      0     n/a
kite1  imu        120    59.80 sensor         2     0.00    0.00    0.00    0.00      0       0      0     n/a
rate_hz: distinct reading timestamps per simulator second. It counts at most one new reading per poll, so it is limited to about the poll rate of 60 Hz. per_poll shows when a sensor delivered more.
kite1 imu: polls returned up to 2 readings. rate_hz counts at most one per poll, so it undercounts this sensor.
wrote scratch\R20\out\sensors.json
```

The columns:

- `steps per second`: how many simulator steps run per wall-clock second. The fake's figure means nothing. The real figure is one of the things this spike measures.
- `polls`: how many times the script polled the sensor.
- `rate_hz`: distinct reading timestamps per simulated second. It counts at most one new reading per poll, so it cannot exceed about 60 Hz.
- `time`: where the timestamps came from. `sensor` means the readings carried their own time. `poll` means they had none and the poll time was used. `mixed` means both.
- `per_poll`: the most readings that one poll returned.
- `mean_ms`, `p50_ms`, `p95_ms`, `max_ms`: how long a poll took, in milliseconds.
- `empty`: polls that returned no reading.
- `repeat`: readings with the same timestamp as the previous reading.
- `unexp`: readings in a format the bridge did not expect.
- `points`: mean lidar points per scan. Other sensors show `n/a`.

Footer lines appear only when they apply:

- `<car> <sensor>: polls returned up to N readings...` appears when `per_poll` is above 1.
- `no sensor time for <car> <sensor>: rate_hz there is the polling rate, not the sensor's` appears when a sensor gave no timestamps of its own.
- `unexpected format: <car> <sensor> N readings, first: ...` appears when `unexp` is above 0. The first unexpected reading is printed, so send that line back in full.

A missing reading is never filled in. An empty poll counts under `empty`, and an unexpected format counts under `unexp`.

### How long it takes

30 simulated seconds plus simulator startup. The same hedge as Run 1 applies.

### What to send back

- The full printed output, including the table and every footer line.
- `recordings/sensors.json`.
- The BeamNG.tech and BeamNGpy versions (from Run 1 and the `pip show` output).
- Any error, in full.

### Headless run

Run the same script without a simulator window. Headless still needs the GPU, and the cameras still render.

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --headless
```

The output has the same shape. This run overwrites `recordings/sensors.json`. To keep the windowed result, rename that file before the headless run, or pass a different folder with `--out recordings/headless`. Send back the full output and the JSON, and say whether a window opened.

## Run 3: several cars (`spike_multicar.py`)

This script repeats the sensor measurement with 1, 2 and 4 cars, each with all five sensors. Each case starts a fresh simulator session.

```powershell
python bridge/spike_multicar.py --home "<your BeamNG.tech folder>"
```

### What it prints

One `model:` line from the first case, then three blocks, one for each case. Each block starts with a blank line and the same summary line, table and footer lines as Run 2, then `wrote ...multicar.json`. The summary lines read `cars: 1`, `cars: 2` and `cars: 4`, and the table has one row per sensor per car (`kite1` to `kite4`). Compare the `steps per second` figure across the three summary lines to see whether more cars slow the simulator.

The script rewrites `multicar.json` after each case, so a run that fails in case 2 still leaves the result of case 1.

### How long it takes

The simulator starts three times, and each case runs 30 simulated seconds. Expect about three times the startup time of Run 1 plus three times the stepping time. For a shorter first try, add `--seconds 10`.

### What to send back

- The full printed output of all three cases.
- `recordings/multicar.json`.
- Any error, in full, with the case it happened in.

## Demo 1: live lidar in Rerun (`spike_lidar_rerun.py`)

This script drives our car in traffic mode with AI traffic and sends the lidar points and camera image into Rerun, once per simulator step, on one timeline. Lidar points are coloured by distance from our car: blue near, red at 50 m and beyond. Our car's position and the traffic cars' positions come from simulator state, so they are ground truth. They appear under `ground_truth/` and only for display. Hide the `ground_truth/` root in the viewer to see the sensor data alone.

It takes the common options and one more: `--save NAME.rrd`.

### With a live viewer

```powershell
python bridge/spike_lidar_rerun.py --home "<your BeamNG.tech folder>"
```

The script is written to open a Rerun viewer window and stream into it. That call has not run against an installed Rerun yet, so the viewer may not open. Tell the director what you see.

### Save a recording instead

```powershell
python bridge/spike_lidar_rerun.py --home "<your BeamNG.tech folder>" --save demo.rrd
```

With `--save`, the script writes `recordings/demo.rrd` and opens no viewer. The name must end in `.rrd` and stay inside `--out`. An absolute path, a drive or `..` gives a usage error. Change the folder with `--out` and the file goes there instead.

To view the saved file later, run `rerun` with the file path. The script's `--help` gives this form.

```powershell
rerun recordings/demo.rrd
```

### What it prints

A `model:` line, then a count of the logged steps. With `--seconds 2` on the fake simulator:

```
model: fake_cherrier_tograc
logged 120 steps
```

These lines appear only when they apply:

- `lidar gave no reading on N of M steps; no lidar was logged for them`
- `camera gave no reading on N of M steps; no image was logged for them`
- `other cars not logged (...)`, with the error, when the script cannot read the traffic cars' positions. It keeps running without them.
- `traffic not spawned (...)`, when AI traffic fails to start.
- `wrote recordings\demo.rrd`, with `--save`, after a successful run.

On the fake simulator with `--save demo.rrd`, the script writes no file and prints `--fake keeps nothing; the recording would have gone to scratch\R20\out\demo.rrd` instead.

### How long it takes

30 simulated seconds plus startup, as in Run 1.

### What to send back

- The full printed output.
- What the viewer showed: whether it opened, and whether you saw the lidar points, the camera image and the car positions.
- A screenshot of the viewer, if it works. Check that no folder path from your machine is visible in it first.
- Any error, in full.

Do not send the `.rrd` file unless the director asks for it.

## If a call fails

Many BeamNGpy and Rerun calls are marked as unverified for BeamNGpy 1.36, so failures are expected.

1. Copy the whole error and the traceback above it. Do not shorten it.
2. Copy the exact command you ran, with the placeholder in place of your real folder.
3. Send both to the director, with the output of `python -m pip show beamngpy rerun-sdk`.

A failed run prints `error: <type>: <message>` and exits with code 1. A traceback may contain a folder path from your machine. The repo is public, so do not paste such text into a pull request, issue or comment. Send it to the director only.

## Recordings stay out of the repo

Everything these scripts write is BeamNG-generated data: the JSON files and the `.rrd` recordings. The files go to `recordings/`, which Git ignores. Never commit them, never copy them into another folder in the repo, and never post them publicly.
