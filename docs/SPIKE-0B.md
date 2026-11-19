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
| `--model` | `vivace` | Our car's vehicle model. |
| `--config` | `vehicles/vivace/tograc_qE.pc` | Our car's part configuration file. `--config=` (nothing after the equals sign) spawns the model's default configuration. |
| `--list-models` | off | Print the simulator's vehicle model names, sorted, one per line, and exit without spawning anything. |
| `--fake` | off | Use the fake simulator. Do not use it for the real run. |

Each script starts BeamNG.tech itself and closes it when the run ends. Close any BeamNG.tech that is already running before you start a script. The bridge connects to the simulator on a fixed local port, so a second copy may block the connection.

Without `--home` and without `--fake`, a script prints a usage error and exits with code 2. If `--home` is not a folder, it prints `error: --home is not a folder` and exits with code 1. Any other failure exits with code 1 too. A failure the script checks for prints `error: <message>`. An unexpected failure prints a traceback, then `error: <type>: <message>`.

### Choosing the car

By default every script spawns the Cherrier Tograc qE: model `vivace` with the part configuration `vehicles/vivace/tograc_qE.pc`. The script does not search for a model by name. A `spawned` line says what it asked for:

```
spawned kite1 (model: vivace, config: vehicles/vivace/tograc_qE.pc)
```

To spawn a different configuration of the same model, pass `--config`. To spawn the model's own default configuration, pass `--config=` with nothing after the equals sign. The `spawned` line then reads `config: model default`.

```powershell
python bridge/spike_connect.py --home "<your BeamNG.tech folder>" --config=
```

Write `--config=` and not `--config ""`. Windows PowerShell 5.1 and PowerShell 7 before 7.3 drop an empty argument when they start a program, so `--config ""` fails there with "expected one argument". `--config=` works in every shell.

If you change `--model`, also change `--config`. The default configuration file belongs to `vivace`, and the scripts do not check that a file matches the model.

To see which model names the simulator offers, run any script with `--list-models`. It starts the simulator, prints the names sorted, one per line, and exits with code 0 without loading a scenario.

```powershell
python bridge/spike_connect.py --home "<your BeamNG.tech folder>" --list-models
```

If the listing fails, the script prints the error and exits with code 1. Send the error back (see "If a call fails").

### The sensor settings

Before the run, every script prints one `sensor:` line for each of the five sensors: lidar, camera, radar, GPS and IMU. Each line lists every setting the bridge passes to BeamNGpy for that sensor, as `name=value`. All settings come from one table, `bridge/kite_bridge/sensor_settings.py`, and the adapter passes nothing else, so the lines say what the simulator was asked for. They print with `--fake` too, in the same shape, although the fake simulator ignores them. `spike_connect.py` prints them after its version lines. `spike_multicar.py` prints them once, not once per case.

```
sensor: lidar requested_update_time=0.1 pos=(0, 0, 1.7) vertical_resolution=64 frequency=20 horizontal_angle=360 max_distance=120 is_360_mode=True is_using_shared_memory=True
sensor: camera requested_update_time=0.05 resolution=(640, 360) field_of_view_y=70 pos=(0, -0.5, 1.5) dir=(0, -1, 0) is_render_colours=True is_render_annotations=False is_render_depth=False is_using_shared_memory=True
sensor: radar requested_update_time=0.1 pos=(0, 0, 1.7) dir=(0, -1, 0) up=(0, 0, 1) range_min=0.1 range_max=100.0 half_angle_deg=30.0 field_of_view_y=70
sensor: gps pos=(0, 1.5, 2.0) ref_lon=0.0 ref_lat=0.0
sensor: imu pos=(0, 0, 0.5) dir=(0, -1, 0) up=(0, 0, 1) gfx_update_time=0.01 physics_update_time=0.01
```

The camera renders colour only: `is_render_annotations` and `is_render_depth` are `False`. The lidar and radar values are the BeamNGpy 1.36 defaults, passed explicitly, except `is_360_mode`, `is_using_shared_memory` and the radar `dir` and `up`, which are the bridge's own choices. The camera, GPS and IMU values are the bridge's own choices too. None of these values has run against the real simulator yet, so a rejected setting is a useful result. `spike_sensors.py` also writes the table into `sensors.json` under `sensor_settings`.

### How to read the output

The numbers in this guide come from `--fake` runs. The fake simulator is synthetic, so the BeamNGpy version lines read `fake` and the rates, counts and step speed will differ on the real simulator. The shape of the output is what carries over.

## Run 1: connect (`spike_connect.py`)

This script connects to the simulator, spawns our car, drives it in traffic mode, and prints its position once per simulated second.

```powershell
python bridge/spike_connect.py --home "<your BeamNG.tech folder>"
```

### What it prints

Three version lines, the five `sensor:` lines, a `spawned` line, then one `t=` line per second, then the path of the file it wrote. With `--seconds 2` on the fake simulator (the `sensor:` lines are shortened here; see "The sensor settings"):

```
BeamNGpy version: fake
BeamNGpy protocol: fake
BeamNG.tech version: not reported by BeamNGpy 1.36
sensor: lidar requested_update_time=0.1 ...
sensor: camera requested_update_time=0.05 ...
sensor: radar requested_update_time=0.1 ...
sensor: gps pos=(0, 1.5, 2.0) ...
sensor: imu pos=(0, 0, 0.5) ...
spawned kite1 (model: vivace, config: vehicles/vivace/tograc_qE.pc)
t=1s position=(10.00, 0.00, 0.00)
t=2s position=(20.00, 0.00, 0.00)
wrote scratch\R27\out\connect.json
```

On the real run, expect 30 `t=` lines and `wrote recordings\connect.json`. The first two version lines are the installed BeamNGpy package version and the protocol version it speaks to the simulator. BeamNGpy 1.36 has no call that returns the BeamNG.tech version, so the third line is fixed text. Read the BeamNG.tech version from the simulator itself, not from this output. If AI traffic cannot start, the script prints `traffic not spawned (...)` with the error and continues with our car alone.

The positions are ground truth from the simulator. The script prints them and writes them to `connect.json`, apart from any sensor data.

### How long it takes

The run covers 30 simulated seconds, plus the time the simulator needs to start. Startup time is unknown. A minute or so is a reasonable guess. Wall-clock time for the 30 simulated seconds depends on how fast the simulator steps, which is one of the things this spike measures.

### What to send back

- The full printed output, including the three version lines.
- `recordings/connect.json`.
- The `pip show` output from setup.
- Any error, in full (see "If a call fails").

## Run 2: sensors (`spike_sensors.py`)

This script attaches a lidar, a camera, a radar, GPS and an IMU to our car. It polls every sensor once per simulator step, 60 times per simulated second, and reports the rates and delays.

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>"
```

### What it prints

The five `sensor:` lines, a `spawned` line, a summary line, a table with one row per sensor, and footer lines. With `--seconds 2` on the fake simulator (the `sensor:` lines are shortened here):

```
sensor: lidar requested_update_time=0.1 ...
sensor: camera requested_update_time=0.05 ...
sensor: radar requested_update_time=0.1 ...
sensor: gps pos=(0, 1.5, 2.0) ...
sensor: imu pos=(0, 0, 0.5) ...
spawned kite1 (model: vivace, config: vehicles/vivace/tograc_qE.pc)
cars: 1  steps per second: 3069039.5
car    sensor   polls  rate_hz   time  per_poll  mean_ms  p50_ms  p95_ms  max_ms  empty  repeat  unexp  points  detect
kite1  lidar      120    10.00 sensor         1     0.00    0.00    0.00    0.01      0      99      0    1000     n/a
kite1  camera     120    20.00 sensor         1     0.00    0.00    0.00    0.01      0      79      0     n/a     n/a
kite1  radar      120    20.00 sensor         1     0.00    0.00    0.00    0.00      0      79      0     n/a     n/a
kite1  gps        120    10.00 sensor         1     0.00    0.00    0.00    0.00      0      99      0     n/a     n/a
kite1  imu        120    59.80 sensor         2     0.00    0.00    0.00    0.00      0       0      0     n/a     n/a
rate_hz: distinct reading timestamps per simulator second. It counts at most one new reading per poll, so it is limited to about the poll rate of 60 Hz. per_poll shows when a sensor delivered more.
kite1 imu: polls returned up to 2 readings. rate_hz counts at most one per poll, so it undercounts this sensor.
wrote scratch\R27\out\sensors.json
```

The columns:

- `steps per second`: how many simulator steps run per wall-clock second. The fake's figure means nothing. The real figure is one of the things this spike measures.
- `polls`: how many polls returned a reading in the expected format, or an empty poll. A poll with an unexpected format is not counted here, so the total number of polls is `polls` plus `unexp`.
- `rate_hz`: distinct reading timestamps per simulated second. It counts at most one new reading per poll, so it cannot exceed about 60 Hz.
- `time`: where the timestamps came from. `sensor` means the readings carried their own time. `poll` means they had none and the poll time was used. `mixed` means both. `n/a` means no reading had a timestamp.
- `per_poll`: the most readings that one poll returned.
- `mean_ms`, `p50_ms`, `p95_ms`, `max_ms`: how long a poll took, in milliseconds.
- `empty`: polls that returned no reading.
- `repeat`: readings with the same timestamp as the previous reading.
- `unexp`: polls that the bridge could not read. This counts a poll that raised an error, a poll that returned a type outside None, a dict, a list, bytes or an array-like object (anything with a length), and a lidar poll without a `pointCloud`.
- `points`: mean lidar points per scan. Other sensors show `n/a`.
- `detect`: mean radar detections per poll. A radar poll that returns an array counts as one reading, and `detect` is its number of rows. The bridge does not read the array's columns. Other sensors show `n/a`.

A poll with no rows, such as a radar array with zero detections, counts under `empty`, like a lidar scan with no points.

The `rate_hz:` explanation line always prints. The other footer lines appear only when they apply:

- `<car> <sensor>: polls returned up to N readings...` appears when `per_poll` is above 1.
- `no sensor time for <car> <sensor>, <car> <sensor>: rate_hz there is the polling rate, not the sensor's` is one line that lists every affected sensor, separated by commas. It appears when at least one sensor has `time` of `poll` or `mixed`.
- `unexpected format: <car> <sensor> N readings, first: ...` appears when `unexp` is above 0. It prints the first message for each sensor, so send those lines back in full. A poll that raised an error reads `first: poll raised <type>: <message>`.

One sensor that fails never stops the run. Its polls count under `unexp`, and the other sensors keep their rows. A run where every sensor fails still exits with code 0, so read the `unexp` column.

A missing reading is never filled in. An empty poll counts under `empty`, and an unexpected format or a failed poll counts under `unexp`.

### How long it takes

30 simulated seconds plus simulator startup. The same hedge as Run 1 applies.

### What to send back

- The full printed output, including the table and every footer line.
- `recordings/sensors.json`.
- The BeamNGpy version lines (from Run 1 and the `pip show` output).
- Any error, in full.

### Headless run

Run the same script without a simulator window. BeamNGpy's headless mode is expected to still need a GPU and to still render the cameras. Nothing in this repo checks that. Look at the camera row. If it shows no readings, say so in your report.

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

The five `sensor:` lines once, then three blocks, one for each case. Each block starts with a `spawned` line that lists the cars of that case, then a blank line and the same summary line, table and footer lines as Run 2, then `wrote ...multicar.json`. The summary lines read `cars: 1`, `cars: 2` and `cars: 4`, and the table has one row per sensor per car (`kite1` to `kite4`). Compare the `steps per second` figure across the three summary lines to see whether more cars slow the simulator.

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

The five `sensor:` lines, a `spawned` line, then a count of the logged steps. With `--seconds 2` on the fake simulator (the `sensor:` lines are left out here):

```
spawned kite1 (model: vivace, config: vehicles/vivace/tograc_qE.pc)
logged 120 steps
```

These lines appear only when they apply:

- `traffic not spawned (...)`, with the error, when AI traffic fails to start. It prints right after the `spawned` line, before the run starts.
- `other cars not logged (...)`, with the error, when the script cannot read the traffic cars' positions. It keeps running without them. It prints during the run, before `logged N steps`.
- `lidar gave no reading on N of M steps; no lidar was logged for them`, after `logged N steps`.
- `camera gave no reading on N of M steps; no image was logged for them`, after `logged N steps`.
- `wrote recordings\demo.rrd`, with `--save`, after a successful run.

On the fake simulator with `--save demo.rrd`, the script writes no file and prints `--fake keeps nothing; the recording would have gone to scratch\R27\out\demo.rrd` instead.

### How long it takes

30 simulated seconds plus startup, as in Run 1.

### What to send back

- The full printed output.
- What the viewer showed: whether it opened, and whether you saw the lidar points, the camera image and the car positions.
- A screenshot of the viewer, if it works. Check that no folder path from your machine is visible in it first.
- Any error, in full.

Do not send the `.rrd` file unless the director asks for it.

## If a call fails

Many BeamNGpy calls are marked as unverified for BeamNGpy 1.36, and the Rerun calls are unverified against the installed `rerun-sdk`, so failures are expected.

1. Copy the whole output, including the traceback above the last line if there is one. Do not shorten it.
2. Copy the exact command you ran, with the placeholder in place of your real folder.
3. Send both to the director, with the output of `python -m pip show beamngpy rerun-sdk`.

A failed run exits with code 1 and ends with one of two lines. A failure the script checks for prints `error: <message>` and no traceback. An unexpected failure prints a traceback, then `error: <type>: <message>`. The output may contain a folder path from your machine. The repo is public, so do not paste such text into a pull request, issue or comment. Send it to the director only.

## Recordings stay out of the repo

Everything these scripts write is BeamNG-generated data: the JSON files and the `.rrd` recordings. The files go to `recordings/`, which Git ignores. Never commit them, never copy them into another folder in the repo, and never post them publicly.
