# Project overview

Kite is a modular autonomous-driving software stack (perception, tracking, prediction, planning, control), developed and tested in simulation in BeamNG.tech. It is an independent, non-commercial student project. Status: setup stage, nothing runs yet.

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
| `sim/` | synthetic scenario generator | exists |
| `tools/` | command-line programs | exists |
| `third_party/` | vendored GoogleTest, changed only by `scripts/vendor_googletest.py` | exists |
| `scripts/` | repo tooling | exists |
| `docs/` | project documents | exists |
| `bridge/` | Python bridge to the simulator and the core | planned |
| `perception/` | Python camera, lidar and radar processing | planned |
| `scenarios/` | scripted test scenarios | planned |
| `eval/` | scoring against simulator ground truth | exists |
| `viewer/` | Rerun visualization of logs | planned |

## Standing principles

- Ground truth scores, never drives.
- Results come from simulation, and every number names the test that produced it.
- Simulator files never enter the repo.
- The code talks to the simulator only through BeamNGpy.
