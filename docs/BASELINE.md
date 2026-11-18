# Baseline

Run these from the repo root after a build with the `dev` preset. `kite_baseline --help` prints the
fixed scenario, the tracker settings and the gate settings.

```
build/dev/tools/kite_baseline
build/dev/tools/kite_baseline --tracker kalman
```

The first command scores the baseline tracker on seeds 1-20. The CTest test
`kite_baseline_doc_matches` fails unless the table below is exactly what it prints.

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 1 | 1398 | 107 | 23 | 1.393 | 87 | 301 |
| 2 | 1418 | 87 | 17 | 1.406 | 73 | 301 |
| 3 | 1413 | 92 | 12 | 1.367 | 79 | 301 |
| 4 | 1424 | 81 | 20 | 1.384 | 68 | 301 |
| 5 | 1427 | 78 | 18 | 1.394 | 62 | 301 |
| 6 | 1417 | 88 | 19 | 1.369 | 68 | 301 |
| 7 | 1419 | 86 | 13 | 1.394 | 82 | 301 |
| 8 | 1420 | 85 | 15 | 1.379 | 72 | 301 |
| 9 | 1414 | 91 | 19 | 1.374 | 76 | 301 |
| 10 | 1404 | 101 | 19 | 1.394 | 91 | 301 |
| 11 | 1411 | 94 | 19 | 1.369 | 73 | 301 |
| 12 | 1425 | 80 | 16 | 1.359 | 72 | 301 |
| 13 | 1409 | 96 | 16 | 1.382 | 83 | 301 |
| 14 | 1410 | 95 | 17 | 1.374 | 79 | 301 |
| 15 | 1423 | 82 | 11 | 1.381 | 72 | 301 |
| 16 | 1409 | 96 | 17 | 1.374 | 85 | 301 |
| 17 | 1407 | 98 | 20 | 1.376 | 89 | 301 |
| 18 | 1413 | 92 | 15 | 1.380 | 79 | 301 |
| 19 | 1402 | 103 | 20 | 1.357 | 97 | 301 |
| 20 | 1424 | 81 | 18 | 1.370 | 64 | 301 |
| mean | 1414.350 | 90.650 | 17.200 | 1.379 | 77.550 | 301.000 |

The baseline is `NearestNeighbourBaseline`, a tracker with no filter that turns each measurement
into a track and links it greedily to the nearest previous track within its gate. The Kalman
tracker below is scored against it on the same scenario, seeds and match gate.

## Kalman tracker

The second command scores `KalmanTracker` on seeds 1-20, with the same scenario and the same 3 m
match gate. The CTest test `kite_baseline_kalman_doc_matches` fails unless the table below is
exactly what it prints.

Parameters:

| parameter | value | how it was chosen |
|---|---|---|
| q (white-noise acceleration density) | 0.01 m^2/s^3 | tuned |
| initial velocity sigma | 10 m/s | tuned |
| measurement sigma | 1.0 m | the sensor's noise spec, a constant in `kite_baseline` |
| chi-square gate | 9.21 | 99 % point of chi-square with 2 degrees of freedom |
| confirmation | 3 hits within a new track's first 5 steps (birth counts as hit 1) | fixed |
| tentative drop | at the 3rd miss, once 3 hits in 5 steps is out of reach | fixed |
| deletion of a confirmed track | 5 consecutive misses | fixed |

Only q and the initial velocity sigma were tuned, on seeds 101-120 only. The table below comes
from one run after the final tuning; seeds 1-20 never informed a parameter. An earlier run at a
superseded commit printed the same table. The grid was q in {0.01, 0.1, 1, 10} m^2/s^3 and
initial velocity sigma in {5, 10, 20} m/s. The rule, fixed before the grid ran, takes the lowest
mean of missed + false + id_switches and breaks ties by the lower rmse_m.
`build/dev/tools/kite_baseline --tracker kalman --seeds 101-120` prints the tuning-seed table for
the chosen values; the CTest test `kite_baseline_kalman_tuning` runs it and checks its mean row
(`ctest -R kite_baseline_kalman_tuning -V` shows the table).

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 1 | 1493 | 12 | 0 | 0.454 | 0 | 301 |
| 2 | 1494 | 11 | 0 | 0.429 | 0 | 301 |
| 3 | 1495 | 10 | 0 | 0.443 | 0 | 301 |
| 4 | 1495 | 10 | 0 | 0.443 | 0 | 301 |
| 5 | 1495 | 10 | 0 | 0.432 | 0 | 301 |
| 6 | 1495 | 10 | 0 | 0.414 | 0 | 301 |
| 7 | 1495 | 10 | 0 | 0.460 | 0 | 301 |
| 8 | 1495 | 10 | 0 | 0.418 | 0 | 301 |
| 9 | 1494 | 11 | 0 | 0.394 | 0 | 301 |
| 10 | 1495 | 10 | 0 | 0.436 | 0 | 301 |
| 11 | 1493 | 12 | 0 | 0.391 | 0 | 301 |
| 12 | 1492 | 13 | 0 | 0.393 | 0 | 301 |
| 13 | 1493 | 12 | 0 | 0.415 | 0 | 301 |
| 14 | 1493 | 12 | 0 | 0.422 | 0 | 301 |
| 15 | 1494 | 11 | 0 | 0.420 | 0 | 301 |
| 16 | 1495 | 10 | 0 | 0.392 | 0 | 301 |
| 17 | 1494 | 11 | 0 | 0.436 | 0 | 301 |
| 18 | 1494 | 11 | 0 | 0.398 | 0 | 301 |
| 19 | 1495 | 10 | 0 | 0.390 | 0 | 301 |
| 20 | 1494 | 11 | 0 | 0.441 | 0 | 301 |
| mean | 1494.150 | 10.850 | 0.000 | 0.421 | 0.000 | 301.000 |

## Comparison

On the mean rows, the Kalman tracker misses 10.850 truths per seed against 90.650 for the
baseline, reports 0.000 false tracks against 17.200 and 0.000 identity switches against 77.550.
Its rmse_m is 0.421 against 1.379.
