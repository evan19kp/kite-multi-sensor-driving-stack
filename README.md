# Kite Multi-Sensor Driving Stack

A modular autonomous-driving software stack (perception, tracking, prediction, planning, control), developed and tested in simulation in BeamNG.tech. It is an independent, non-commercial student project. Status: setup stage, nothing runs yet.

## Planned architecture

- Planned stages: sensors through a simulator bridge, then perception, then tracking and own-state estimation, then prediction, then planning, then control.
- Planned core: C++17 built with CMake and tested with GoogleTest.
- Planned bridge: Python, talking to the simulator through BeamNGpy.
- Planned viewer: Rerun for 3D visualization.

## Build and test

The C++ part needs CMake 3.20 or newer, Ninja and a C++17 compiler (GCC, Clang or MSVC). GoogleTest is vendored in `third_party/googletest`, so there is nothing else to install.

With the `dev` preset (Ninja, Debug, output in `build/dev`):

```
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

Without the preset:

```
cmake -S . -B build/dev -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/dev
ctest --test-dir build/dev --output-on-failure
```

Pass `-DKITE_BUILD_TESTS=OFF` to skip the tests. To check the build scripts: `python -m unittest discover -s scripts -v`.

## Simulation only

Any results, once there are some, will come from simulation. This project makes no claims about real vehicles.

## Credits

BeamNG.tech by BeamNG GmbH

This project is not affiliated with or endorsed by BeamNG GmbH.

## Licence

All rights reserved; see LICENSE.
