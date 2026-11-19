# bridge

Python code that talks to the BeamNG.tech simulator through BeamNGpy. Built so far: a simulator
interface, a real adapter, a fake simulator and three spike scripts that measure the simulator's
connection, sensors and multi-car behaviour. Planned, not built yet: the Rerun viewer and the
local link to the core. No result from the real simulator is recorded here; Evan runs the spikes
on it.

## Layout

- `kite_bridge/sim_api.py`: the small `Simulator` interface the scripts use.
- `kite_bridge/beamng_adapter.py`: the real simulator. The only module that imports `beamngpy`,
  and only inside methods. Calls not verified against BeamNGpy 1.36 are marked ASSUMPTION.
- `kite_bridge/fake_sim.py`: a seeded synthetic simulator with configurable sensor rates,
  dropouts, repeated readings and latency. All tests use it.
- `kite_bridge/measure.py`: pure measurement functions (effective rate, latency statistics,
  empty and repeated readings, steps per second). Its docstring states the input domain.
- `kite_bridge/common.py`: command-line parsing and the step-and-poll loop shared by the scripts.
- `spike_connect.py`, `spike_sensors.py`, `spike_multicar.py`: the spike scripts.
- `requirements.txt`: `beamngpy==1.36` and `rerun-sdk`. Tests need neither.

## Scripts

All three take `--home <simulator folder>` (required unless `--fake`), `--headless`,
`--seconds` (default 30), `--traffic` (default 8), `--out` (default `recordings/`), `--model` and
`--fake`. `--fake` runs against the fake simulator and needs no simulator.

```
python bridge/spike_sensors.py --fake --seconds 2 --out scratch/example
```

Without `--model`, a script lists the vehicle models the simulator reports and picks the single
one whose name contains "tograc" or "cherrier"; with none or several it stops and asks for
`--model`.

Output goes only to `--out`, which is gitignored as `recordings/`. Ground truth (the car's true
position) is written in its own file, `connect.json`, apart from sensor data, and scores results
but never drives anything.

## Tests

```
python -m unittest discover -s bridge -v
```

Standard library only; no simulator, network or installed package is used.
