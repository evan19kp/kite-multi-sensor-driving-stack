# bridge

Python code that talks to the BeamNG.tech simulator through BeamNGpy. Built so far: a simulator
interface, a real adapter, a fake simulator, three spike scripts that measure the simulator's
connection, sensors and multi-car behaviour, and a Rerun lidar demo script. Planned, not built
yet: the local link to the core. No result from the real simulator is recorded here; Evan runs
the spikes on it, and nothing here has run against an installed Rerun yet.

## Layout

- `kite_bridge/sim_api.py`: the small `Simulator` interface the scripts use.
- `kite_bridge/beamng_adapter.py`: the real simulator. The only module that imports `beamngpy`,
  and only inside methods. None of its BeamNGpy calls has run against BeamNGpy 1.36 yet. Calls
  whose name, arguments or result format are guesses carry an `# ASSUMPTION:` comment.
- `kite_bridge/readings.py`: turns a BeamNGpy poll result into plain data: a reading for any
  sensor, the lidar points or the camera image. An empty poll, including a lidar scan with no
  points, is no reading; a radar array with no rows is a reading with 0 detections. It never
  truth-tests a poll, so numpy arrays are safe, and it states its input domain in its docstring.
  It does not import `beamngpy` or numpy, so tests drive it with plain dicts, lists and small
  array-like classes. Its format guesses are marked ASSUMPTION too.
- `kite_bridge/sensor_settings.py`: the one table of every setting the adapter passes to a
  sensor, including `is_streaming=True` for lidar and camera with `--stream`, and the `sensor:`
  lines the scripts print from it. It imports only `sim_api`.
- `kite_bridge/stream_check.py`: the one place that checks a streamed lidar or camera reading
  and decides what is dropped and counted under `suspect`. Standard library only.
- `kite_bridge/fake_sim.py`: a seeded synthetic simulator with configurable sensor rates,
  dropouts, repeated readings, latency, buffered GPS and IMU readings, a streamed lidar memory
  block that keeps stale and zero rows after a shorter scan, a free-running mode whose simulated
  time advances with its clock (real time without one) and whose Timer stands still while paused, and, with a
  `FakeClock`, a fake cost per request, per streamed read and per step (no real waiting). All
  tests use it.
- `kite_bridge/measure.py`: pure measurement functions (effective rate, latency statistics,
  empty and repeated readings, steps per second, real-time factor, the processing remainder and
  new-reading counts). Its docstring states the input domain.
- `kite_bridge/rates.py`: parses `--rates` (with separate lock-step and free-mode defaults) and
  decides which steps each sensor is polled on, and in free mode when each is next due on the
  wall clock, which reads a pass makes, and how many due times a late read passed over, with
  exact rational arithmetic. Standard library only.
- `kite_bridge/raw_data.py`: the one place that counts the bytes of a raw poll and fingerprints
  it, and that turns a raw reading into plain data for the dump. Standard library only.
- `kite_bridge/common.py`: command-line parsing, the step-and-poll loop and the one shutdown path
  shared by the scripts.
- `kite_bridge/output_guard.py`: the provenance text every written file carries and the `--out`
  check. It imports nothing outside the standard library.
- `kite_bridge/rerun_view.py`: the only module that imports `rerun`, and only inside a method.
  It holds the entity paths, the pure distance-to-colour function, `log_step` (lock-step),
  `log_reading` (free mode) and the recorders.
  Its Rerun calls are guesses until the first real run and carry `# ASSUMPTION:` comments.
- `spike_connect.py`, `spike_sensors.py`, `spike_multicar.py`: the spike scripts.
- `spike_lidar_rerun.py`: Demo 1, the Rerun lidar demo.
- `requirements.txt`: `beamngpy==1.36` and `rerun-sdk`. Tests need neither.

## Scripts

All four take `--home <simulator folder>` (required unless `--fake`), `--headless`,
`--seconds` (default 30), `--traffic` (default 8), `--out` (default `recordings/`, see below), `--model`,
`--config`, `--list-models`, `--timeout` (default 300) and `--fake`. `--fake` runs against the
fake simulator and needs no simulator. `spike_sensors.py` and `spike_multicar.py` also take
`--rates` (polls per simulated second for each sensor, default `lidar=20,camera=20,radar=20,gps=10,imu=10`;
with `--mode free` reads per wall second, default `lidar=20,camera=40,radar=10,gps=10,imu=10`),
`--dump-raw N` (default 0), `--stream` (default off) and `--mode lockstep|free` (default
`lockstep`). `spike_lidar_rerun.py` takes `--rates`, `--mode` (default `free`) and
`--stream`/`--no-stream` (default on with `--mode free`); see "Rerun lidar demo".

`--timeout` is the longest wait in seconds for any one simulator call, map loading included; it
must be above 0. Every run ends BeamNG.tech through one shutdown path. After a normal run or an
ordinary error it removes the sensors, then closes BeamNG.tech; with `--mode free` it pauses the
simulator before it removes them. If the pause fails, the script prints `warning: could not
pause: <error>; sensors not removed` whatever the run's outcome, skips the removal and still
closes BeamNG.tech. After Ctrl+C or a timeout it closes BeamNG.tech without
pausing or removing the sensors. If closing raises or takes more than 20 s, or
Ctrl+C is pressed again during shutdown, it ends the process BeamNGpy launched, and only that
process. Ctrl+C prints `interrupted: BeamNG.tech closed` and exits with code 130. A timeout
prints `timed out after <s> s: BeamNG.tech closed` and exits with code 1. Neither prints a
traceback. A poll that times out stops the run; it is not counted under `unexp`.

```
python bridge/spike_sensors.py --fake --seconds 2 --out scratch/example
```

The car is the Cherrier Tograc qE by default: `--model vivace --config
vehicles/vivace/tograc_qE.pc`. `--config=` (nothing after the equals sign) spawns the model's default configuration instead.
The `spawned` line prints the model and the configuration. `--list-models` prints the
simulator's vehicle model names, sorted, one per line, and exits without spawning anything.

`spike_connect.py` prints three version lines: the BeamNGpy package version, the BeamNGpy
protocol version and `BeamNG.tech version: not reported by BeamNGpy 1.36`.

`--seconds` rounds to the nearest whole step (`spike_connect.py`: the nearest whole second), at
least one.

Each sensor row says where its timestamps came from. `time` is `sensor` when the readings carried
their own time, `poll` when they had none and the poll time was used (then `rate_hz` is the
polling rate, not the sensor's, and the table says so), and `mixed` for both. A lidar reading
without `pointCloud` is counted under `unexp` and printed with its keys, not as an empty poll. A
poll that raises, or returns a type outside the input domain in `readings.py`, is counted under
`unexp` for that sensor, with the first message printed in the footer; the run goes on with the
other sensors. A radar poll that returns an array is one reading, and the `detect` column is the
mean row count over those arrays, including arrays with 0 rows. The bridge does not read the
columns. A radar that returned no array shows `n/a`. A lidar scan with no points counts under
`empty`, like any poll that returned None or an empty dict, list or bytes. A radar array with
zero rows does not count under `empty`. It is one reading with 0 detections.

Before the run every script prints one `sensor:` line per sensor with every setting the adapter
passes, taken from `kite_bridge/sensor_settings.py`. `spike_sensors.py` writes the same table
into `sensors.json` as `sensor_settings`. The camera renders colour only.

The two measuring scripts poll each sensor at its `--rates` rate, in polls per simulated second,
at most 60 (the step rate). `--rates all=60` polls every sensor after every step; `all=RATE` sets
every sensor and a named sensor overrides it wherever it is written. A rate that is not a number
above 0 and at most 60 is a usage error (exit code 2). A sensor at rate `r` is polled after step 1
and after each step `k` where `floor(k * r / 60)` rises, which gives `floor(N * r / 60) + 1` polls
in `N` steps (`N` at 60); `kite_bridge/rates.py` has the rule and the parsing. Each row records its
`configured_rate_hz` and `requested_update_hz` (1 divided by the `requested_update_time` in
`sensor_settings.py`, `null` for GPS and IMU, which pass none), and a `possible_loss` note prints
for a lidar, camera or radar read slower than every step that gave more than one reading fewer
than its requested rate. GPS and IMU buffer readings and their capacity is not known, so for them
the check is to compare `new` between two runs (`docs/SPIKE-0B.md`, "Compare two runs"). `rate_hz`
is the rate of distinct reading timestamps seen at poll time. It counts at most one new reading
per poll, so it is limited to about the sensor's poll rate, even for a sensor that runs faster. It does not try
to recover the sensor's own rate. Instead, the `per_poll` column is the most readings one poll
returned, and the JSON field `readings_per_poll_max` has the same value. When it is above 1, the table prints a line for that
sensor saying so.

`spike_sensors.py` and `spike_multicar.py` also print and write, per run, the wall time split
into waiting for steps, in sensor reads and in our own processing, the simulated and wall seconds
and the real-time factor (simulated over wall seconds; `n/a` and `null` when the wall time is 0),
and per sensor the read time in total and per poll, the new readings per simulated second and the
raw bytes per simulated second. `spike_connect.py` polls no sensor, so it prints none of them;
`spike_lidar_rerun.py` prints them in its default free mode. `--dump-raw N` on the two measuring scripts writes the first
N raw GPS and the first N raw IMU readings of each car to `raw_gps_imu.json` in `--out`, each with
its poll's wall time and step count. `docs/SPIKE-0B.md` explains each number.

`--stream` on the two measuring scripts creates the lidar and camera with `is_streaming=True` and
reads them from shared memory with `stream()`, which sends no request; radar, GPS and IMU are
still polled, at the same `--rates`. Every streamed reading goes through `stream_check.py` first.
All-zero rows after the last non-zero row of a lidar scan are cut and counted under `sus_rows`.
A lidar reading of the wrong shape or with a value that is not finite, and a camera reading with
an image that is not the configured resolution, are dropped whole and counted under `suspect`,
with the first reason printed. A camera `stream()` result that is not None and not a dict counts
under `unexp`. Rows left from an earlier, longer scan cannot be told from points and are kept.
Streamed reads are timed as `in stream reads`, apart from `in sensor polls`. That time includes
the bridge's check, which reads every byte of a lidar scan; the bridge does not time it alone. The
JSON records `"stream"`, `timing.stream_reads_s` and each row's `read` and `suspect`. Without
`--stream` the output is as before. Streaming has not run against the real simulator.

`--mode free` on the two measuring scripts lets the simulator run on its own instead of in
lock-step. The adapter attaches a `Timer` sensor to the first car, switches the simulator to
non-deterministic and resumes it. The script then reads each sensor at its `--rates` rate per
wall second. A sensor at rate `r` is due at `k / r` wall seconds after the start. Each pass reads
every due streamed sensor, then at most one due poll (a sensor that is not streamed, or the
Timer): the most overdue, the Timer first on a tie, then the sensor polled longest ago
(`kite_bridge/rates.py`, `pass_reads`). So a due lidar or camera read waits for at most the one
poll already running. A read moves its sensor to the first `k / r` after the read started, so a
late read skips the times it missed instead of making them up (`next_due`). A `schedule` table
prints, per sensor and for the Timer, the reads that started 2 ms or more late, the worst
lateness in ms and the due times skipped; a `skipped:` line names each sensor that missed any.
The Timer is polled first and in its own request, at the higher of the GPS and IMU rates. The run ends when the
Timer has advanced `--seconds` since its first reading, or fails with one `error:` line (exit
code 1) after `--seconds` x 20 wall seconds, which a Timer that stands still also reaches. A
Timer reading that is not a number of seconds from 0 up fails the run the same way. It is not
counted under `unexp` like a bad sensor reading, because the Timer decides when the run ends.
Simulated seconds, the real-time factor (Timer seconds over wall seconds) and the per-second
counts use the Timer; `poll_hz` is per wall second. A reading without its own time gets the
latest Timer reading and shows `time` `timer`. The timing splits the wall time into `waiting to
poll`, `in Timer polls`, the reads and `our processing`. The JSON records `"mode"` in both modes;
in free mode `steps` is `null`, `timer` gives the Timer's poll count, rate and `schedule`, each
sensor row has `schedule` (`late`, `worst_late_ms`, `skipped`), and a dump entry's `step` is `null`. `--stream` and `--dump-raw` work in both modes. Free mode has not run
against the real simulator, and its real-time factor on the fake is the fake's own speed.

Output goes only to `--out`, `recordings/` by default, which is gitignored. Every JSON file has a
top-level `"provenance"` field that marks the file as simulator output not to commit. The text is
the `PROVENANCE` constant in `bridge/kite_bridge/output_guard.py`.

A script refuses an `--out` that resolves inside the repo unless it is under `recordings/` or
`scratch/`, both gitignored. It prints a usage error naming `--out` and exits with code 2 before
it starts the simulator or writes anything. A folder outside the repo is allowed. The check
resolves `..` and symlinks first, so `--out recordings/../docs` is refused. A relative `--out`
is taken from the folder you run the script in. The repo is the folder with `AGENTS.md` above the
script; for a worktree under `.worktrees/` that includes the main checkout around it.

Ground truth (the car's true position) is written in its own file, `connect.json`, apart from
sensor data, and scores results but never drives anything.

## Rerun lidar demo

`spike_lidar_rerun.py` drives our car in traffic mode with AI traffic and logs on one `sim_time`
timeline. It takes the common options above plus `--mode`, `--stream`/`--no-stream`, `--rates`
and `--save NAME.rrd`.

By default it runs `--mode free --stream`: `common.measure_sensors` reads the sensors on the
free-mode schedule (streamed lidar and camera first, at most one poll per pass, free-mode
default rates) and passes each reading to `rerun_view.log_reading`. Each lidar scan's
`pointCloud` goes under `world/ego/lidar` and each camera `colour` image under `camera/image`, at
its Timer time, as BeamNGpy gave them: no distance colours and no ground truth, because reading a
position is a request that would hold up the lidar. The script then prints the measuring tables,
the `schedule` table included, and how many scans and images it logged. `--no-stream` polls the
lidar and camera too.

`--mode lockstep` is the demo as before; it takes neither `--stream` nor `--rates` (usage error,
exit code 2). Once per simulator step it logs lidar points coloured by distance from our car
(blue near, red at 50 m and beyond) under `world/ego/lidar` and the camera image under
`camera/image`. Our position and the traffic cars' positions come from simulator state, so they
are ground truth, shown for display only under `ground_truth/ego/position` and
`ground_truth/other_cars`; hide the `ground_truth/` root in the viewer to see sensor data alone.
When the lidar or the camera gives no reading on a step, nothing is logged for that sensor and the
script prints how many steps that was.

- Without `--save` it spawns a Rerun viewer.
- With `--save` it writes `<--out>/NAME.rrd` and opens no viewer; open the file later with
  `rerun <file>`. The name must end in `.rrd` and stay inside `--out` (no absolute path, drive or
  `..`), else the script exits with a usage error. After Ctrl+C the recording is closed before
  BeamNG.tech, so the file keeps everything logged until then.
- With `--fake` it uses the fake simulator and a recorder that keeps nothing: no viewer, no
  Rerun import and no file, even with `--save`.

```
python bridge/spike_lidar_rerun.py --fake --seconds 2 --out scratch/example
```

In free mode the fake runs on the real clock, so this takes about 2 wall seconds. Nothing here
has run against BeamNG.tech or an installed Rerun, and no real-time factor is claimed.

`.rrd` files are recordings: they go only under `--out` and are never committed. A recording
logs the same provenance text once as its first entry, as a text log under `meta/provenance`.
That call has not run against an installed Rerun yet.

## Tests

```
python -m unittest discover -s bridge -v
```

Standard library only; no simulator, network or installed package is used.
