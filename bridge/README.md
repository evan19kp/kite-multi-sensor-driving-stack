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
  sensor, and the `sensor:` lines the scripts print from it. It imports nothing.
- `kite_bridge/fake_sim.py`: a seeded synthetic simulator with configurable sensor rates,
  dropouts, repeated readings and latency. All tests use it.
- `kite_bridge/measure.py`: pure measurement functions (effective rate, latency statistics,
  empty and repeated readings, steps per second). Its docstring states the input domain.
- `kite_bridge/common.py`: command-line parsing and the step-and-poll loop shared by the scripts.
- `kite_bridge/rerun_view.py`: the only module that imports `rerun`, and only inside a method.
  It holds the entity paths, the pure distance-to-colour function, `log_step` and the recorders.
  Its Rerun calls are guesses until the first real run and carry `# ASSUMPTION:` comments.
- `spike_connect.py`, `spike_sensors.py`, `spike_multicar.py`: the spike scripts.
- `spike_lidar_rerun.py`: Demo 1, the Rerun lidar demo.
- `requirements.txt`: `beamngpy==1.36` and `rerun-sdk`. Tests need neither.

## Scripts

All four take `--home <simulator folder>` (required unless `--fake`), `--headless`,
`--seconds` (default 30), `--traffic` (default 8), `--out` (default `recordings/`), `--model`,
`--config`, `--list-models` and `--fake`. `--fake` runs against the fake simulator and needs no
simulator.

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

The scripts poll each sensor once per step, 60 times per simulator second. `rate_hz` is the rate
of distinct reading timestamps seen at poll time. It counts at most one new reading per poll, so
it is limited to about 60 Hz, the poll rate, even for a sensor that runs faster. It does not try
to recover the sensor's own rate. Instead, the `per_poll` column is the most readings one poll
returned, and the JSON field `readings_per_poll_max` has the same value. When it is above 1, the table prints a line for that
sensor saying so.

Output goes only to `--out`, which is gitignored as `recordings/`. Ground truth (the car's true
position) is written in its own file, `connect.json`, apart from sensor data, and scores results
but never drives anything.

## Rerun lidar demo

`spike_lidar_rerun.py` drives our car in traffic mode with AI traffic and, once per simulator
step, logs on one `sim_time` timeline: lidar points coloured by distance from our car (blue near,
red at 50 m and beyond) under `world/ego/lidar` and the camera image under `camera/image`. Our
position and the traffic cars' positions come from simulator state, so they are ground truth,
shown for display only under `ground_truth/ego/position` and `ground_truth/other_cars`; hide the
`ground_truth/` root in the viewer to see sensor data alone. When the lidar or the camera gives no
reading on a step, nothing is logged for that sensor and the script prints how many steps that
was. It takes the common options above plus `--save NAME.rrd`.

- Without `--save` it spawns a Rerun viewer.
- With `--save` it writes `<--out>/NAME.rrd` and opens no viewer; open the file later with
  `rerun <file>`. The name must end in `.rrd` and stay inside `--out` (no absolute path, drive or
  `..`), else the script exits with a usage error.
- With `--fake` it uses the fake simulator and a recorder that keeps nothing: no viewer, no
  Rerun import and no file, even with `--save`.

```
python bridge/spike_lidar_rerun.py --fake --seconds 2 --out scratch/example
```

`.rrd` files are recordings: they go only under `--out` and are never committed.

## Tests

```
python -m unittest discover -s bridge -v
```

Standard library only; no simulator, network or installed package is used.
