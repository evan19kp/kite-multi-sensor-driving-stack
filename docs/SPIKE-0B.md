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
| `--out` | `recordings/` | Folder for the output files. Inside the repo only `recordings/` and `scratch/` are allowed. See "Where output may go". |
| `--model` | `vivace` | Our car's vehicle model. |
| `--config` | `vehicles/vivace/tograc_qE.pc` | Our car's part configuration file. `--config=` (nothing after the equals sign) spawns the model's default configuration. |
| `--list-models` | off | Print the simulator's vehicle model names, sorted, one per line, and exit without spawning anything. |
| `--timeout` | 300 | The longest wait, in seconds, for any one simulator call, including the map load. Must be above 0. See "Stopping a run". |
| `--fake` | off | Use the fake simulator. Do not use it for the real run. |

`spike_sensors.py` and `spike_multicar.py` also take `--rates` (how often each sensor is read, see "Reading each sensor at its own rate" under Run 2), `--dump-raw N` (default 0, which writes nothing; see "The raw GPS and IMU dump" under Run 2), `--stream` (default off; see "Streaming the lidar and camera" under Run 2) and `--mode` (default `lockstep`; see "Letting the simulator run on its own" under Run 2).

Each script starts BeamNG.tech itself and closes it when the run ends. Close any BeamNG.tech that is already running before you start a script. The bridge connects to the simulator on a fixed local port, so a second copy may block the connection.

Without `--home` and without `--fake`, a script prints a usage error and exits with code 2. If `--home` is not a folder, it prints `error: --home is not a folder` and exits with code 1. Any other failure exits with code 1 too. A failure the script checks for prints `error: <message>`. An unexpected failure prints a traceback, then `error: <type>: <message>`.

### Stopping a run

Press Ctrl+C once to stop a run. The script closes BeamNG.tech, prints one line and exits with code 130:

```
interrupted: BeamNG.tech closed
```

A run that is waiting for the simulator does not wait forever. If one simulator call takes longer than `--timeout` seconds (300 by default), the run stops, closes BeamNG.tech and exits with code 1. A call that waits too long while the script removes the sensors at the end also counts. The script then prints `warning: could not remove <car> <sensor>: <type>: <message>` before the timeout line. If the run had already failed, the warning still appears, but the last line reports that earlier failure. The timeout line names the limit:

```
timed out after 300 s: BeamNG.tech closed
```

Neither line comes with a traceback. The limit covers loading the map too, which can take a minute or more, so keep the limit well above that. Raise it if the map load times out, for example with `--timeout 600`.

The script ends BeamNG.tech the same way whether the run succeeded, failed, timed out or was stopped with Ctrl+C:

1. After a normal run or an ordinary error, it removes the sensors, then closes BeamNG.tech. With `--mode free`, it pauses the simulator first. If the pause fails, the script skips the sensors, still closes BeamNG.tech, and reports the pause error.
2. After Ctrl+C or a timeout, a call may have stopped halfway, so the script skips the pause and the sensors and closes BeamNG.tech straight away.
3. If closing raises an error or takes longer than 20 seconds, the script ends the BeamNG.tech process that BeamNGpy launched and prints `BeamNG.tech not closed (<reason>); ended the process BeamNGpy launched`.

If shutdown is slow, press Ctrl+C a second time. The script skips to step 3 at once.

The script ends only the process BeamNGpy launched. If BeamNGpy connected to a BeamNG.tech that was already running, the script does not end it. It prints `...BeamNGpy did not launch it, so nothing was ended: close BeamNG.tech by hand`, and the last line reads `BeamNG.tech may still be running`. Close BeamNG.tech yourself in that case.

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

This script attaches a lidar, a camera, a radar, GPS and an IMU to our car. It polls each sensor at its own rate (see "Reading each sensor at its own rate"; `--rates all=60` polls every sensor once per simulator step, 60 times per simulated second) and reports the rates and delays, where the wall time goes, how many new readings each sensor gave and how many bytes it read. It also takes `--rates`, `--dump-raw N` (see "The raw GPS and IMU dump") and `--stream` (see "Streaming the lidar and camera"); so does `spike_multicar.py`. `spike_connect.py` polls no sensors, and the Rerun demo (`spike_lidar_rerun.py`) draws sensor data and does not measure it, so neither prints timing or takes these options.

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>"
```

### Compare two runs

To see whether reading less loses any readings, run the script twice into different folders and compare the `new` and `new_per_s` columns. The first run polls every sensor after every step, as before. The second uses the default rates.

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --rates all=60 --out recordings/every-step
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --out recordings/default-rates
```

Look at three things:

- `new` and `new_per_s` for each sensor. If a sensor's count is the same in both runs, reading less lost nothing for that sensor. A lower count in the second run means that sensor missed readings.
- GPS and IMU above all. The tool cannot tell from one run whether their buffers overflowed, so this comparison is the check for them. The two runs are separate drives in traffic, so their counts can differ a little for reasons that have nothing to do with polling. Judge a gap by its size: a few readings in thousands is probably that, and a large share of the readings is probably loss.
- The `polls`, `in sensor reads` and `real-time factor` numbers. Fewer polls should mean less read time and a higher real-time factor. If the factor barely changes, something other than polling limits it, so look at `waiting for steps`.

Send back both printed outputs and both `sensors.json` files.

### What it prints

The five `sensor:` lines, a `spawned` line, a summary line, a table with one row per sensor, a timing block with a second table, and footer lines. With `--seconds 2` on the fake simulator, default rates (the `sensor:` lines are shortened here):

```
sensor: lidar requested_update_time=0.1 ...
sensor: camera requested_update_time=0.05 ...
sensor: radar requested_update_time=0.1 ...
sensor: gps pos=(0, 1.5, 2.0) ...
sensor: imu pos=(0, 0, 0.5) ...
spawned kite1 (model: vivace, config: vehicles/vivace/tograc_qE.pc)
cars: 1  steps per second: 2061868.8
car    sensor   polls  rate_hz   time  per_poll  mean_ms  p50_ms  p95_ms  max_ms  empty  repeat  unexp  points  detect
kite1  lidar       41    10.00 sensor         1     0.02    0.01    0.03    0.10      0      20      0    1017     n/a
kite1  camera      41    20.00 sensor         1     0.01    0.00    0.01    0.15      0       0      0     n/a     n/a
kite1  radar       41    20.00 sensor         1     0.00    0.00    0.01    0.01      0       0      0     n/a     n/a
kite1  gps         21    10.00 sensor         1     0.00    0.00    0.00    0.00      0       0      0     n/a     n/a
kite1  imu         21    10.05 sensor        10     0.00    0.00    0.01    0.01      0       0      0     n/a     n/a
timing: 2.000 simulated s in 0.006 wall s, real-time factor 345.471
  waiting for steps     0.000 s    1.0%
  in sensor reads       0.001 s   21.8%
  our processing        0.004 s   77.2%
car    sensor     read_s  per_poll_ms  poll_hz    new  new_per_s  req_hz       bytes  kB_per_s
kite1  lidar       0.001         0.02    20.00     21      10.50   10.00      501206     250.6
kite1  camera      0.000         0.01    20.00     41      20.50   20.00       16482       8.2
kite1  radar       0.000         0.00    20.00     41      20.50   10.00       23739      11.9
kite1  gps         0.000         0.00    10.00     21      10.50     n/a         777       0.4
kite1  imu         0.000         0.00    10.00    200     100.00     n/a        7400       3.7
rate_hz: distinct reading timestamps per simulator second. It counts at most one new reading per poll, so it is limited to about the sensor's poll rate (poll_hz, at most 60). per_poll shows when a sensor delivered more.
loss not checkable for gps, imu: a buffered sensor's capacity is not known, so compare its new readings with a run at --rates all=60.
kite1 imu: polls returned up to 10 readings. rate_hz counts at most one per poll, so it undercounts this sensor.
wrote scratch\example\sensors.json
```

With `--rates all=60` the `polls` column reads 120 for every sensor and `poll_hz` reads 60.00. The `new` counts are the same as above: 21, 41, 41, 21 and 200.

The last line shows the `--out` folder of that run. Without `--out`, it is `recordings`.

The columns:

- `steps per second`: how many simulator steps run per wall-clock second. The fake's figure means nothing. The real figure is one of the things this spike measures.
- `polls`: how many polls returned a reading in the expected format, or an empty poll. A poll with an unexpected format is not counted here, so the total number of polls is `polls` plus `unexp`.
- `rate_hz`: distinct reading timestamps per simulated second. It counts at most one new reading per poll, so it cannot exceed about the sensor's poll rate (`poll_hz` in the second table, at most 60).
- `time`: where the timestamps came from. `sensor` means the readings carried their own time. `poll` means they had none and the poll time was used. `mixed` means both. `n/a` means no reading had a timestamp.
- `per_poll`: the most readings that one poll returned.
- `mean_ms`, `p50_ms`, `p95_ms`, `max_ms`: how long a poll took, in milliseconds.
- `empty`: polls that returned no reading.
- `repeat`: readings with the same timestamp as the previous reading.
- `unexp`: polls that the bridge could not read. This counts a poll that raised an error, a poll that returned a type outside None, a dict, a list, bytes or an array-like object (anything with a length), a lidar poll without a `pointCloud`, and a poll holding a value the bridge cannot size or fingerprint (`first: raw data not measurable: ...`). That last case is a value of a type `raw_data.py` does not list, such as a complex number, an integer beyond 64 bits, or an object with a length that can be neither iterated nor turned into bytes. The bridge measures dict keys of every type that `raw_data.py` lists: text, numbers, true/false, None, bytes and tuples.
- `points`: mean lidar points per scan. Other sensors show `n/a`.
- `detect`: mean radar detections over the polls that returned an array. A radar poll that returns an array counts as one reading, and its detections are the array's rows. The bridge does not read the array's columns. Other sensors, and a radar that returned no array, show `n/a`.

A lidar scan with no points counts under `empty`, like any poll that returned None or an empty dict, list or bytes. A radar array with zero rows does not count under `empty`. It is one reading that records 0 detections.

#### Where the time goes

The `timing:` line and the three lines under it split the wall-clock time of the stepping-and-polling loop. The loop does not include starting the simulator, loading the map or attaching the sensors.

- `simulated s`: simulated seconds, the number of steps divided by 60.
- `wall s`: wall-clock seconds from just before the first step to just after the last poll.
- `real-time factor`: simulated seconds divided by wall seconds. 1.0 is real time and 0.1 is ten times slower than real time. It prints `n/a`, and is `null` in the JSON, when the wall time is 0, because there is nothing to divide by. The factor says how fast this run went. It does not say whether any target is met.
- `waiting for steps`: time spent inside the step call. The bridge calls `step` with `wait=True`, which is expected to return once the simulator has finished the step. That has not run against the real simulator yet.
- `in sensor reads`: time spent inside the poll calls of all sensors. Each poll call covers everything inside the adapter's poll: BeamNGpy's poll, however it fetches and decodes the data, plus the bridge's parse of the result in `readings.py`. It is more than the round trip to the simulator. The second table shows it per sensor: `read_s` is the total over the run and `per_poll_ms` is `read_s` divided by the number of poll calls, in milliseconds, failed polls included.
- `our processing`: wall time minus the two above. It is a remainder, so it also holds anything else that happened between the timed calls. This run's own measuring work is part of it: sizing and fingerprinting each poll's raw data for the second table. When this share is large, some of it is the measurement and not the bridge.

The three parts add up to the wall time by construction. The percentage after each is its share of the wall time. The JSON holds the same numbers under `timing` (`simulated_s`, `wall_s`, `real_time_factor`, `step_wait_s`, `sensor_reads_s`, `processing_s`) and, per sensor row, `read_total_s` and `read_per_poll_s`. A sensor read at a lower rate is polled fewer times, so its `read_s` is smaller while `per_poll_ms` stays about the same. The saving shows in `in sensor reads` and in the real-time factor, not in the cost of one poll.

#### Reading each sensor at its own rate

`--rates` sets how many polls per simulated second each sensor gets. The value is a comma-separated list of `sensor=rate` pairs. The sensors are `lidar`, `camera`, `radar`, `gps` and `imu`; `all` sets every sensor. A rate is a number above 0 and at most 60, because the simulator steps 60 times per simulated second and a sensor is polled at most once per step. Anything else, including an unknown sensor name, a pair without `=`, `nan`, `inf`, a rate of 0 or below, a rate above 60 and a sensor named twice, is a usage error. The script prints it and exits with code 2 before it starts anything.

| Command | Meaning |
|---|---|
| (no `--rates`) | The default: `lidar=20,camera=20,radar=20,gps=10,imu=10`. |
| `--rates all=60` | Every sensor after every step, the way the scripts read before `--rates` existed. |
| `--rates all=30` | Every sensor 30 times per simulated second. |
| `--rates all=60,gps=10` | `all` is the base, and a named sensor overrides it wherever it is written: GPS at 10, the rest at 60. `gps=10,all=60` means the same. |
| `--rates gps=60` | Sensors that are not named keep their default: GPS at 60, the rest at the default rates. |

The lidar and the radar ask for 10 updates per second (`requested_update_time=0.1`) and the camera for 20 (`requested_update_time=0.05`). The defaults poll the lidar and the radar at twice the rate asked of them, so a new reading is picked up within half its interval. The default polls the camera at the rate asked of it, 20, so a new reading is picked up within one interval and may be missed if the polls and the updates drift apart. The camera default stays at 20, not 40. The comparison below shows whether the camera loses readings at 20. GPS and IMU buffer their readings between polls, so a poll returns everything made since the last one.

Which steps a sensor is polled on is exact and the same every run. A sensor at rate `r` is polled after step 1 and after every step `k` where `floor(k * r / 60)` is greater than `floor((k - 1) * r / 60)`. The rate is an exact fraction, so a decimal such as `2.5` carries no rounding error. After `N` steps the sensor has had `floor(N * r / 60) + 1` polls for any `r` below 60, and `N` polls at 60. For example, at `r = 20` the polled steps are 1, 3, 6, 9 and so on, and 60 steps give 21 polls. At `r = 7` they are 1, 9, 18, 26, 35, 43, 52 and 60: 8 polls. Step 1 is always polled so that the first reading is not missed and a buffered sensor's first poll collects everything made so far.

How the table and the JSON show it:

- `poll_hz`: the configured poll rate. The JSON row has `configured_rate_hz`.
- `polls`: the polls made, so at the default rates it is about a third of the polls at `all=60` for lidar, camera and radar, and about a sixth for GPS and IMU.
- `new` and `new_per_s`, as before. Compare them with `req_hz`.
- `req_hz`: the update rate the adapter asks of the sensor, `1 / requested_update_time` from `bridge/kite_bridge/sensor_settings.py`. GPS and IMU show `n/a` because the adapter passes no `requested_update_time` for them. The JSON row has `requested_update_hz`, `null` for those two.

The `possible loss` note decides like this, for each lidar, camera or radar row. It prints only when the sensor is polled slower than every step (`poll_hz` below 60) and `new` is below `req_hz` times the simulated seconds, less one reading. The one reading is slack for the first and last interval of the run, which are partial. For a 30-second run of a sensor that asks for 10 Hz, that is a note when `new` is below 299. The JSON row has `possible_loss` as `true` or `false`. A line reads:

```
possible loss: kite1 lidar gave 6.00 new readings per simulated second at 5 polls per second, fewer than the 10 Hz requested. Compare with a run at --rates all=60.
```

The note means the sensor, at this poll rate, delivered fewer new readings than it was asked to make, so some updates were probably missed between polls. It can also mean the simulator made fewer updates than asked (for example, because the run is slow), which is why it says "possible". Run again with `--rates all=60` and compare.

The note is never printed for GPS and IMU. Their buffer size is not known, so one run cannot say whether a poll returned as many readings as the buffer could hold. When one of them is polled slower than every step, the script prints `loss not checkable for ...` instead. Compare their `new` counts between a run at `--rates all=60` and a run at the rate you want; see "Compare two runs".

`--rates` does not change `--seconds`, which stays in simulated seconds, and it does not change how fast the simulator steps. It changes how many times a sensor is read in each simulated second.

#### New readings and bytes

- `new`: how many readings had content that was new. For lidar, camera and radar, a poll counts when its raw data differs from the previous non-empty poll's, compared by a hash of the raw bytes and values. A poll that is identical to the one before is not new. For GPS and IMU, every different reading a poll returned counts once, where "different" means a different `time` or different values; a reading returned again in a later poll is not counted again. A run of S simulated seconds on a sensor that makes R new readings per second shows about R times S, and one more when the first reading counts, so a short run overstates the rate by up to 1 reading per run.
- `new_per_s`: `new` divided by the simulated seconds. Unlike `rate_hz`, it does not depend on timestamps. For lidar, camera and radar, which carry no timestamp, `rate_hz` is the polling rate (the table says so), and `new_per_s` shows how often their content changed. For GPS and IMU, `rate_hz` counts at most one reading per poll, and `new_per_s` counts all of them. A sensor that returns identical data while it is running, for example a lidar in a scene where nothing changes, shows fewer new readings than it produced.
- `bytes`: the raw payload size of all the polls that returned data, counted as `bridge/kite_bridge/raw_data.py` documents: an array or image counts its buffer, a number 8 bytes, a name its UTF-8 length, and a dict key the same as a value of its type, so the key `0` counts 8 bytes and the key `"gps_0"` 5. The whole poll is counted by that one rule, so a GPS or IMU poll that holds a dict of readings also counts the dict's keys. It is what the bridge received, not what crossed the wire. Empty polls and unexpected polls count 0.
- `kB_per_s`: `bytes` divided by the simulated seconds, in kilobytes of 1000 bytes. The JSON has `bytes_read` and `bytes_per_simulated_s` in bytes.

The JSON rows also have `new_readings` and `new_per_simulated_s`.

#### The raw GPS and IMU dump

`--dump-raw N` writes `raw_gps_imu.json` to `--out`: the first N GPS readings and the first N IMU readings of each car, as the simulator returned them. It exists so that the director can work out how a reading's `time` field relates to simulated time and to wall time. With N missing or 0, no file is written. If the run returns fewer than N readings of a sensor, the file has fewer.

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --dump-raw 50
```

The file has the top-level `provenance` field, `readings_per_car_and_sensor` (N) and `cases`, a list with one entry per run case: `cars` and `entries`. `spike_sensors.py` has one case. `spike_multicar.py` has one for each car count (1, 2 and 4). An entry has:

- `car` and `sensor` (`gps` or `imu`).
- `step`: the number of steps taken when the poll was made. The simulated time of the poll is `step` divided by 60.
- `wall_s`: wall seconds from the first step to the end of the poll.
- `poll_wall_s`: how long the poll call took, in wall seconds. When one poll returned several readings, they share `step`, `wall_s` and `poll_wall_s`.
- `reading`: every field of one raw reading, numbers and names only. A number that is not finite is written as the text `nan`, `inf` or `-inf`. JSON object keys are always text, so a key that is not text is written as its text: the key `2` becomes `"2"`. If one reading has two keys with the same text, such as `2` and `"2"`, only the later one is written. Anything that is not a number, a name, a list or a dict is written as `<unsupported TypeName>`, never copied.

Readings are in the order the polls returned them, and a reading that a later poll returns again is dumped again. Count entries with the same `time` and the same values to see how often that happens.

#### Streaming the lidar and camera

Every poll costs about 21 ms in your earlier runs, whatever the sensor. `--stream` tests whether streaming avoids that cost for the lidar and the camera. With `--stream`, the bridge creates those two sensors with `is_streaming=True`. The simulator then writes each reading into shared memory, and the bridge reads it with `stream()`, which sends no request to the simulator. The radar, GPS and IMU are still polled, because BeamNGpy 1.36 does not stream them. `--rates` still decides when each sensor is read. Without `--stream`, nothing changes: every sensor is polled, and the output looks as described above.

Nobody has run streaming against the real simulator yet. Your run decides whether it works and whether its data can be trusted.

Run the script twice with the same rates, once without and once with `--stream`:

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --out recordings/poll
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --stream --out recordings/stream
```

What `--stream` adds to the output:

- The `sensor:` lines for lidar and camera end in `is_streaming=True`, and `sensor_settings` in `sensors.json` has it too.
- Three more columns in the first table. `read` says how the sensor was read: `stream` or `poll`. `suspect` and `sus_rows` are explained below. They show `n/a` for polled sensors.
- In the timing block, `in sensor reads` becomes two lines. `in sensor polls` is the time inside poll calls. `in stream reads` is the time inside stream calls. That covers BeamNGpy's `stream()`, which copies the shared memory, and the bridge's checks and parse. The check reads every byte of each lidar scan. The bridge does not time the check on its own, so `in stream reads` holds the bridge's time as well as BeamNGpy's. Neither line counts the other's time. The second table's `read_s` and `per_poll_ms` for lidar and camera are stream time.
- A `stream:` footer line that says which sensors were streamed, and one `suspect:` line for each streamed sensor that dropped something.
- In the JSON: `"stream": true` at the top (`false` without `--stream`), `timing.stream_reads_s`, and on each sensor row `read` and `suspect`. For a streamed sensor, `suspect` is `{"readings": ..., "rows": ..., "first": ...}`. For a polled sensor it is `null`.

With `--seconds 2` on the fake simulator, default rates (the `sensor:` lines are left out, the second table and the usual footer lines are cut):

```
car    sensor   polls  rate_hz   time  per_poll  mean_ms  p50_ms  p95_ms  max_ms  empty  repeat  unexp  points  detect    read  suspect  sus_rows
kite1  lidar       41    20.17   poll         1     0.14    0.20    0.26    0.34      0       0      0    1093     n/a  stream        0       273
kite1  camera      41    20.17   poll         1     0.01    0.01    0.01    0.02      0       0      0     n/a     n/a  stream        0         0
kite1  radar       41    20.00 sensor         1     0.01    0.01    0.02    0.03      0       0      0     n/a     n/a    poll      n/a       n/a
kite1  gps         21    10.00 sensor         1     0.00    0.00    0.01    0.01      0       0      0     n/a     n/a    poll      n/a       n/a
kite1  imu         21    10.05 sensor        10     0.01    0.01    0.01    0.03      0       0      0     n/a     n/a    poll      n/a       n/a
timing: 2.000 simulated s in 0.042 wall s, real-time factor 48.074
  waiting for steps     0.000 s    0.2%
  in sensor polls       0.001 s    1.7%
  in stream reads       0.006 s   14.8%
  our processing        0.035 s   83.2%
...
stream: lidar and camera were read from shared memory with stream(); radar, gps and imu with poll(). suspect counts streamed readings dropped whole, sus_rows the all-zero rows cut off the end of kept lidar scans. Rows left from an earlier scan cannot be detected.
suspect: kite1 lidar dropped 0 readings and cut 273 all-zero rows
```

The fake's times mean nothing. Its streamed camera image is 640 by 360, the configured size, but its polled image is 8 by 4, so the camera's `bytes` differ between the two fake runs for that reason alone. The streamed lidar and camera show `time` `poll` because their readings carry no timestamp.

##### What `suspect` means

When the lidar streams, BeamNGpy reads the whole shared-memory block and does not ask the simulator how many points the scan has. The rows after the scan may be memory that was never written, or rows left from an earlier, longer scan. The bridge checks every streamed reading before it uses it. The rules are in `bridge/kite_bridge/stream_check.py`.

- `sus_rows` (JSON `suspect.rows`), lidar only: the all-zero rows cut off the end of lidar scans that were kept. An all-zero row is a row whose bytes are all zero. The bridge cuts every all-zero row after the last row that has any non-zero byte, and the matching rows of any other array in the reading with the same row count, such as the colours. The count is summed over reads, so memory read twice counts twice. A scan counted under `unexp` adds nothing to `sus_rows`. The `points` column counts the rows that were kept.
- `suspect` (JSON `suspect.readings`): streamed readings dropped whole. Nothing of such a reading is used or counted under `polls`, `empty`, `new` or `bytes`. Its read time still counts. A lidar reading is dropped when its `pointCloud` is not an array of rows with at least 3 columns, when its byte count does not fit 4- or 8-byte floats for its shape, or when a kept value is not finite (`nan` or `inf`). A camera reading is dropped when any image in it is not the configured `resolution`, 640 by 360, or has no size. `suspect.first` holds the first reason, and the `suspect:` line prints it.
- If a camera `stream()` returns None, the bridge counts it under `empty` and takes it as no frame yet. Whether BeamNGpy ever does this is not known. A camera `stream()` result that is not a dict of images counts under `unexp`, with `first: camera stream() returned <type>, not a dict of images`.
- A reading whose memory has not changed since the last read is not suspect. It is not new either: `new` compares the content, as before.

What the bridge cannot detect:

- Rows left from an earlier, longer scan. They are ordinary points of the right shape, and the streamed reading says nothing about how many rows the new scan wrote, so they are kept and counted in `points`. If `points` with `--stream` is clearly higher than without it, stale rows are the likely reason.
- An all-zero row in the middle of a scan. It is kept and not counted.
- A camera image of the right size that holds an old frame or was never drawn into, such as an all-black image.

Look at these numbers:

- `per_poll_ms` and `mean_ms` for lidar and camera in the two runs. In your earlier runs a poll took about 21 ms. The stream run shows what a streamed read costs, including the bridge's check.
- `in stream reads` against `in sensor reads` from the poll run, and the real-time factor of both runs.
- `suspect` against `polls`. If every streamed reading is dropped, streaming does not deliver usable data, and the `suspect:` line says why.
- `sus_rows` and `points`. Compare `points` with the poll run (see the list above).
- `new_per_s` for lidar and camera in both runs. Fewer new readings with `--stream` means the shared memory changed less often than the polls returned new data.
- The `unexpected format:` lines. A stream call that raises prints `first: stream raised <type>: <message>`.

Send back both printed outputs and both `sensors.json` files.

#### Letting the simulator run on its own

In your earlier runs one car reached about 0.1 times real time, and BeamNG.tech felt slow during the runs but smooth on its own. By default the scripts run the simulator in lock-step (`--mode lockstep`). The simulator waits after each step of 1/60 s until the script asks for the next one, and the script reads the sensors between steps. `--mode free` tests what happens when the simulator does not wait.

With `--mode free`:

- The bridge switches the simulator from lock-step to running on its own and resumes it. The script takes no steps.
- It attaches a `Timer` sensor to our car (`kite1`). The Timer gives simulated seconds since the scenario started and does not advance while the simulator is paused. It is a clock only. Our car's position from the simulator is still ground truth and is still kept apart from sensor data.
- It polls each sensor at its `--rates` rate per wall second, not per simulated second. The rule is below.
- It polls the Timer in its own request, at the higher of the GPS and IMU rates. With the default rates that is 10 per wall second.
- The run ends at the first Timer reading that is at least `--seconds` past the run's first Timer reading. `--seconds` therefore still counts simulated seconds.
- If that has not happened after `--seconds` × 20 wall seconds, the run stops with an error. For the default of 30 seconds the limit is 600 wall seconds. A Timer that stands still, for example because the simulator is paused, reaches this limit. The run does not hang.

Nobody has run free mode against the real simulator yet. Your run decides whether it works, and only your run can show a real-time factor. This mode claims no target.

Run the script twice with the same rates, once in each mode:

```powershell
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --out recordings/lockstep
python bridge/spike_sensors.py --home "<your BeamNG.tech folder>" --mode free --out recordings/free
```

During the free run, watch the BeamNG.tech window. Say in your report whether it feels smooth, as it does when BeamNG.tech runs with traffic and no script, or slow, as in the lock-step runs.

`--stream` and `--dump-raw N` work in free mode too. To see both changes together, add a third run with `--mode free --stream --out recordings/free-stream`.

##### The wall-clock schedule

A sensor at rate `r` is due at `k / r` wall seconds after the run starts, for k = 0, 1, 2 and so on. The script works in passes. Each pass reads the wall clock once. It polls the Timer if the Timer is due, then polls each sensor that is due, once, in the usual order. It then sleeps until the next due time. A polled sensor's next due time is the first `k / r` after the start of the pass.

The polls themselves take time, about 21 ms each in your earlier runs. When a pass runs late, each sensor that is due is polled once, and the due times it missed are skipped. They are not made up later in a burst. For example, at 20 polls per second a sensor is due every 0.05 s. If a pass takes 0.2 s, the sensor is polled once at the next pass and is next due at the first multiple of 0.05 s after that pass started. So `polls` can be lower than the rate times the wall seconds. That gap shows that the polls cannot keep up.

With polls that take no time, a run of W wall seconds gives each sensor `ceil(W × r)` polls. On the fake simulator, which runs at real time, a 2-second run at the default rates gives 40 polls of lidar, camera and radar and 20 of GPS and IMU. The rule and the exact arithmetic are in `bridge/kite_bridge/rates.py`.

##### Which clock each number uses

| Number | Clock in free mode |
|---|---|
| `poll_hz`, `--rates` | Polls per wall second. |
| `simulated s`, `rate_hz`, `new_per_s`, `kB_per_s` | Timer seconds: the last Timer reading minus the first. |
| `wall s`, `mean_ms` to `max_ms`, `read_s`, `per_poll_ms` | Wall seconds. |
| `real-time factor` | Timer seconds divided by wall seconds. |

The `time` column shows `timer` for a sensor whose readings carry no time of their own. Your earlier runs showed that the lidar, camera and radar readings have no time. Such a reading gets the latest Timer reading as its approximate time. The Timer is polled less often than the lidar, camera and radar, so several of their readings share one Timer value. For those sensors, `rate_hz` follows how often the Timer was polled, and `repeat` counts readings that share a Timer value. Use `new_per_s` for how often their content changed. GPS and IMU keep their own `time`.

The `possible loss` note and the `loss not checkable for gps, imu` line work as under "Reading each sensor at its own rate". The difference is that every sensor counts as read slower than every step, because nothing holds the simulator still between polls. The note says `polls per wall second`.

##### Where the time goes in free mode

There are no steps, so `steps per second` is replaced by a line that gives the Timer's poll count and rate. The `timing:` block splits the wall time from just before the first Timer poll to the end of the last one. These parts add up to the wall time:

- `waiting to poll`: time asleep until the next sensor or the Timer was due. A large share means the bridge kept up with its rates and had time to spare.
- `in Timer polls`: time inside the Timer's poll requests.
- `in sensor reads` (or `in sensor polls` and `in stream reads` with `--stream`): as in lock-step mode.
- `our processing`: the remainder, as in lock-step mode.

The `timing:` line says `simulated s (Timer)`. On the fake simulator the factor is about 1.000 because the fake runs at real time by construction. That number says nothing about BeamNG.tech.

Look at these numbers:

- The real-time factor in both runs. If free mode reads close to 1.0 while lock-step reads about 0.1, the stepping held the simulator back.
- `polls` against the rate times the wall seconds. If `polls` is much lower, the sensor reads cannot keep up with the rates. `waiting to poll` is then near 0 and `in sensor reads` takes most of the wall time.
- `new_per_s` in both runs, for every sensor. Fewer new readings in free mode can mean that the simulator made updates the polls did not collect.

##### What free mode changes in the output and the JSON

With `--seconds 2` on the fake simulator, default rates (the `sensor:` lines are left out, and the second table is cut):

```
cars: 1  mode: free, the Timer polled 21 times at 10 per wall second
car    sensor   polls  rate_hz   time  per_poll  mean_ms  p50_ms  p95_ms  max_ms  empty  repeat  unexp  points  detect
kite1  lidar       40    10.00 sensor         1     0.05    0.05    0.06    0.10      0      20      0    1017     n/a
kite1  camera      40    20.00 sensor         1     0.01    0.01    0.01    0.01      0       0      0     n/a     n/a
kite1  radar       40    20.00 sensor         1     0.01    0.01    0.01    0.01      0       0      0     n/a     n/a
kite1  gps         20    10.00 sensor         1     0.00    0.00    0.01    0.01      0       0      0     n/a     n/a
kite1  imu         20    10.00 sensor        10     0.01    0.01    0.01    0.01      0       0      0     n/a     n/a
timing: 2.000 simulated s (Timer) in 2.001 wall s, real-time factor 1.000
  waiting to poll       1.990 s   99.5%
  in Timer polls        0.000 s    0.0%
  in sensor reads       0.003 s    0.1%
  our processing        0.008 s    0.4%
...
```

The fake's polled lidar, camera and radar carry a time, so they show `sensor` here. The streamed ones do not, so with `--stream` they show `timer`.

After the `rate_hz:` footer line, a `mode free:` line says which clock each number uses. The JSON has:

- `"mode"`: `"free"`, or `"lockstep"` in a lock-step run.
- `"steps"` and `"steps_per_second"`: `null`.
- `"timer"`: `{"polls": ..., "rate_hz": ...}`.
- Under `timing`: `idle_s` (waiting to poll) and `timer_reads_s` (in Timer polls) in place of `step_wait_s`.

In `raw_gps_imu.json`, an entry's `step` is `null` in free mode, because no steps are taken. `wall_s` is measured from the start of the run.

If the wall-clock limit is reached, the script pauses the simulator, removes the sensors, closes BeamNG.tech and exits with code 1. It writes no `sensors.json` and prints one line, for example:

```
error: free run: the Timer advanced 0.625 of 1 simulated s in the wall-clock limit of 20 s (--seconds x 20)
```

Send that line back. It means the simulator ran at less than 1/20 of real time, or that it was paused. Ctrl+C works as described under "Stopping a run".

#### Footer lines

The `rate_hz:` explanation line always prints. The other footer lines appear only when they apply:

- `possible loss: <car> <sensor> ...` appears for each lidar, camera or radar row that meets the rule under "Reading each sensor at its own rate".
- `loss not checkable for ...` appears when GPS or IMU is polled slower than every step.
- `<car> <sensor>: polls returned up to N readings...` appears when `per_poll` is above 1.
- `no sensor time for <car> <sensor>, <car> <sensor>: rate_hz there is the polling rate, not the sensor's` is one line that lists every affected sensor, separated by commas. It appears when at least one sensor has `time` of `poll`, `timer` or `mixed`.
- With `--mode free` only: the `mode free:` line, which says which clock each number uses. See "Letting the simulator run on its own".
- `unexpected format: <car> <sensor> N readings, first: ...` appears when `unexp` is above 0. It prints the first message for each sensor, so send those lines back in full. A poll that raised an error reads `first: poll raised <type>: <message>`.
- With `--stream` only: the `stream:` line always, and `suspect: <car> <sensor> dropped N readings and cut M all-zero rows` for each streamed sensor where either count is above 0, with `, first reason: ...` when a reading was dropped. See "Streaming the lidar and camera".

One sensor that fails never stops the run. Its polls count under `unexp`, and the other sensors keep their rows. A run where every sensor fails still exits with code 0, so read the `unexp` column. A poll that takes longer than `--timeout` is the exception: it stops the run as a timeout (see "Stopping a run").

A missing reading is never filled in. An empty poll counts under `empty`, and an unexpected format or a failed poll counts under `unexp`.

### How long it takes

30 simulated seconds plus simulator startup. The same hedge as Run 1 applies.

### What to send back

- The full printed output, including the table, the `timing:` block and every footer line, from both runs under "Compare two runs".
- `recordings/every-step/sensors.json` and `recordings/default-rates/sensors.json`.
- `recordings/raw_gps_imu.json`, from a run with `--dump-raw 50` (see "The raw GPS and IMU dump").
- The printed output and `sensors.json` of both runs under "Streaming the lidar and camera" (`recordings/poll` and `recordings/stream`).
- The printed output and `sensors.json` of both runs under "Letting the simulator run on its own" (`recordings/lockstep` and `recordings/free`), and whether the BeamNG.tech window felt smooth during the free run.
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

The five `sensor:` lines once, then three blocks, one for each case. Each block starts with a `spawned` line that lists the cars of that case, then a blank line and the same summary line, table and footer lines as Run 2, then `wrote ...multicar.json`. The summary lines read `cars: 1`, `cars: 2` and `cars: 4`, and the table has one row per sensor per car (`kite1` to `kite4`). Each case also prints the `timing:` block and the second table (see "Where the time goes" and "New readings and bytes" under Run 2). Compare the `steps per second` figure and the real-time factor across the three cases to see whether more cars slow the simulator, and compare `waiting for steps` with `in sensor reads` to see which part grows. With `--stream`, the timing block prints `in sensor polls` and `in stream reads` in place of `in sensor reads`, so compare `waiting for steps` with both. `--dump-raw N` writes `raw_gps_imu.json` with one case per car count and N GPS and N IMU readings per car.

`--rates` applies to every case, with the same default as Run 2. Each case prints `poll_hz` and `req_hz` and the loss notes. To compare with the old behaviour, run it once more with `--rates all=60`. `--stream` also applies to every case, with the same extra columns and lines as Run 2. So does `--mode free`: each case runs free, prints the Timer line and the free-mode timing block, and stops at the wall-clock limit if its Timer falls behind. With more cars, compare the real-time factor across the cases in both modes.

The script rewrites `multicar.json` after each case, so a run that fails in case 2 still leaves the result of case 1.

### How long it takes

The simulator starts three times, and each case runs 30 simulated seconds. Expect about three times the startup time of Run 1 plus three times the stepping time. For a shorter first try, add `--seconds 10`.

### What to send back

- The full printed output of all three cases.
- `recordings/multicar.json`.
- `recordings/raw_gps_imu.json` if you ran with `--dump-raw`.
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

If you stop the run with Ctrl+C, the script closes the recording before it closes BeamNG.tech, so the file keeps every step logged before you pressed it. It does not print `wrote ...` in that case. The last line is `interrupted: BeamNG.tech closed`. The close call that writes out the file has not run against an installed Rerun yet, so check that the file opens.

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

A failed run exits with code 1 and ends with one of these lines:

```
error: <message>
error: <type>: <message>
timed out after <s> s: BeamNG.tech closed
timed out after <s> s: BeamNG.tech may still be running
```

A failure the script checks for prints `error: <message>` and no traceback. An unexpected failure prints a traceback, then `error: <type>: <message>`. A timeout prints no traceback. A run stopped with Ctrl+C exits with code 130 and ends with `interrupted: BeamNG.tech closed` or `interrupted: BeamNG.tech may still be running`. `may still be running` means the script could not end BeamNG.tech, so close it by hand. Send a timeout back like any other failure, with the command and the output before the last line. The output may contain a folder path from your machine. The repo is public, so do not paste such text into a pull request, issue or comment. Send it to the director only.

## Where output may go

Everything these scripts write is BeamNG-generated data: the JSON files and the `.rrd` recordings. It must never reach the public repo. The scripts enforce three things:

- Each JSON file carries a top-level `"provenance"` field that says the file is simulator output and must not be committed. The text is the `PROVENANCE` constant in `bridge/kite_bridge/output_guard.py`. A `.rrd` recording logs the same text once at the start, under `meta/provenance`. Anyone who opens a stray file can see where it came from.
- `--out` may be `recordings/` (the default), `scratch/` or any folder outside the repo, for example on another drive. Both folders inside the repo are ignored by Git.
- Any other `--out` inside the repo is refused. For example `--out docs/x` prints a usage error that names `--out` and exits with code 2. The script has not started the simulator or written a file at that point. Run the script again with an allowed folder. The check follows `..` and symlinks, so `--out recordings/../docs` is refused too.

A relative `--out` is taken from the folder you run the script in, so run from the repo root as in the rest of this guide.

Never commit these files, never copy them into another folder in the repo, and never post them publicly. A screenshot goes in the repo only under `docs/media/`, and only if you chose it. Simulator error output is for the director only; do not paste it into a pull request, issue or comment.
