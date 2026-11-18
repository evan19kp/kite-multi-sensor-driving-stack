# Project overview

Kite is a modular autonomous-driving software stack, developed and tested in the BeamNG.tech simulator. It is an independent, non-commercial student project. It is at the setup stage: the only stack code so far is the C++ core skeleton.

## Planned stages

1. Perception
2. Tracking and own-state estimation
3. Prediction
4. Planning
5. Control

All of these are planned; none is built yet.

## Folder map

| Folder | Contents | Status |
|---|---|---|
| `core/` | C++17 library and tests, built with CMake and tested with GoogleTest | exists |
| `third_party/` | vendored GoogleTest, changed only by `scripts/vendor_googletest.py` | exists |
| `scripts/` | repo tooling | exists |
| `docs/` | project documents | exists |
| `bridge/` | Python bridge to the simulator and the core | planned |
| `perception/` | Python camera, lidar and radar processing | planned |
| `scenarios/` | scripted test scenarios | planned |
| `eval/` | scoring against simulator ground truth | planned |
| `viewer/` | Rerun visualization of logs | planned |

## Standing principles

- Ground truth scores, never drives.
- Results come from simulation, and every number names the test that produced it.
- Simulator files never enter the repo.
- The code talks to the simulator only through BeamNGpy.
