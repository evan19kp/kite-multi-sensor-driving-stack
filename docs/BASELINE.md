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

## Hard scenario

`--scenario hard` runs the same 5 vehicles, 30 s, dt 0.1 s, noise 1.0 m, detection probability 0.95
and half-width 100 m as above, plus turning, speed changes and clutter. The constants are in
`tools/baseline_setup.hpp`; the generator documents the motion and the clutter draws in
`sim/include/kite/generator.hpp`. Clutter is false detections, 2 per step on average, written as
ordinary measurements that nothing marks. The trackers and the 3 m scoring gate are the ones above,
with every parameter unchanged. `--scenario straight` is the default and prints the tables above.

```
build/dev/tools/kite_baseline --scenario hard
build/dev/tools/kite_baseline --scenario hard --tracker kalman
```

The baseline tracker on seeds 1-20. The CTest test `kite_baseline_hard_doc_matches` fails unless
the table below is exactly what the first command prints.

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 1 | 1408 | 97 | 600 | 1.394 | 93 | 301 |
| 2 | 1425 | 80 | 599 | 1.423 | 82 | 301 |
| 3 | 1406 | 99 | 637 | 1.364 | 82 | 301 |
| 4 | 1398 | 107 | 587 | 1.376 | 93 | 301 |
| 5 | 1423 | 82 | 612 | 1.397 | 68 | 301 |
| 6 | 1417 | 88 | 666 | 1.391 | 75 | 301 |
| 7 | 1407 | 98 | 662 | 1.357 | 81 | 301 |
| 8 | 1415 | 90 | 623 | 1.387 | 79 | 301 |
| 9 | 1424 | 81 | 636 | 1.383 | 62 | 301 |
| 10 | 1415 | 90 | 528 | 1.382 | 83 | 301 |
| 11 | 1422 | 83 | 625 | 1.347 | 75 | 301 |
| 12 | 1420 | 85 | 638 | 1.362 | 79 | 301 |
| 13 | 1402 | 103 | 595 | 1.367 | 90 | 301 |
| 14 | 1406 | 99 | 630 | 1.406 | 92 | 301 |
| 15 | 1424 | 81 | 620 | 1.411 | 67 | 301 |
| 16 | 1400 | 105 | 594 | 1.365 | 88 | 301 |
| 17 | 1412 | 93 | 631 | 1.370 | 80 | 301 |
| 18 | 1412 | 93 | 623 | 1.340 | 82 | 301 |
| 19 | 1422 | 83 | 661 | 1.378 | 71 | 301 |
| 20 | 1421 | 84 | 640 | 1.377 | 84 | 301 |
| mean | 1413.950 | 91.050 | 620.350 | 1.379 | 80.300 | 301.000 |

The Kalman tracker on seeds 1-20, with the parameters in the table under "Kalman tracker" above. The CTest test
`kite_baseline_hard_kalman_doc_matches` fails unless the table below is exactly what the second
command prints.

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 1 | 1449 | 56 | 107 | 1.212 | 28 | 301 |
| 2 | 1469 | 36 | 99 | 1.174 | 24 | 301 |
| 3 | 1451 | 54 | 107 | 1.067 | 23 | 301 |
| 4 | 1469 | 36 | 72 | 1.013 | 19 | 301 |
| 5 | 1466 | 39 | 82 | 1.024 | 16 | 301 |
| 6 | 1446 | 59 | 113 | 1.193 | 25 | 301 |
| 7 | 1479 | 26 | 70 | 1.104 | 20 | 301 |
| 8 | 1461 | 44 | 90 | 1.206 | 20 | 301 |
| 9 | 1460 | 45 | 102 | 1.110 | 21 | 301 |
| 10 | 1453 | 52 | 118 | 1.180 | 26 | 301 |
| 11 | 1444 | 61 | 139 | 1.308 | 26 | 301 |
| 12 | 1451 | 54 | 103 | 1.176 | 19 | 301 |
| 13 | 1459 | 46 | 111 | 1.245 | 24 | 301 |
| 14 | 1466 | 39 | 88 | 1.230 | 24 | 301 |
| 15 | 1451 | 54 | 108 | 1.063 | 21 | 301 |
| 16 | 1471 | 34 | 74 | 1.226 | 19 | 301 |
| 17 | 1474 | 31 | 80 | 1.224 | 22 | 301 |
| 18 | 1475 | 30 | 99 | 1.182 | 24 | 301 |
| 19 | 1451 | 54 | 115 | 1.290 | 23 | 301 |
| 20 | 1446 | 59 | 103 | 1.163 | 21 | 301 |
| mean | 1459.550 | 45.450 | 99.000 | 1.169 | 22.250 | 301.000 |

### Comparison with the straight scenario

On the hard mean rows, the Kalman tracker matches 1459.550 truths per seed against 1413.950 for
the baseline and misses 45.450 against 91.050. It reports 99.000 false tracks against 620.350 and
22.250 identity switches against 80.300, and its rmse_m is 1.169 against 1.379. It is not worse
than the baseline on any column of these tables. Against the straight tables, the Kalman tracker
is worse on every scored metric. There it missed 10.850, reported 0.000 false tracks and 0.000
identity switches, with an rmse_m of 0.421. The baseline's false tracks rise from 17.200 to 620.350
and its identity switches from 77.550 to 80.300, while its missed count (90.650 to 91.050) and
rmse_m (1.379 in both) barely move.

### Tuning report (no change)

The same grid as the Kalman tracker's tuning above, run on the hard scenario with seeds 101-120
only: q in {0.01, 0.1, 1, 10} m^2/s^3 and initial velocity sigma in {5, 10, 20} m/s. The rule is
the same: the lowest mean of missed + false + id_switches (the cost column), ties by the lower
rmse_m. `kite_baseline --grid` takes seeds from 101 up and refuses any range that includes a
reporting seed. The CTest test `kite_baseline_hard_grid_doc_matches` fails unless the output below
is exactly what this prints, and `kite_baseline_hard_grid_output` pins the selected row.

```
build/dev/tools/kite_baseline --scenario hard --grid 101-120
```

Kalman tuning grid, hard scenario, seeds 101-120, mean per seed; cost is the mean of missed + false + id_switches

| q | velocity_sigma | matched | missed | false | rmse_m | id_switches | steps | cost |
|---|---|---|---|---|---|---|---|---|
| 0.010 | 5.000 | 1461.600 | 43.400 | 92.100 | 1.138 | 20.700 | 301.000 | 156.200 |
| 0.010 | 10.000 | 1462.950 | 42.050 | 91.000 | 1.132 | 20.400 | 301.000 | 153.450 |
| 0.010 | 20.000 | 1462.950 | 42.050 | 92.850 | 1.138 | 20.750 | 301.000 | 155.650 |
| 0.100 | 5.000 | 1477.550 | 27.450 | 55.700 | 1.080 | 13.650 | 301.000 | 96.800 |
| 0.100 | 10.000 | 1478.850 | 26.150 | 57.200 | 1.077 | 13.600 | 301.000 | 96.950 |
| 0.100 | 20.000 | 1478.150 | 26.850 | 58.050 | 1.080 | 13.450 | 301.000 | 98.350 |
| 1.000 | 5.000 | 1491.850 | 13.150 | 6.550 | 0.784 | 1.200 | 301.000 | 20.900 |
| 1.000 | 10.000 | 1492.950 | 12.050 | 6.350 | 0.780 | 1.100 | 301.000 | 19.500 |
| 1.000 | 20.000 | 1492.750 | 12.250 | 7.600 | 0.782 | 1.200 | 301.000 | 21.050 |
| 10.000 | 5.000 | 1494.100 | 10.900 | 1.100 | 0.806 | 0.150 | 301.000 | 12.150 |
| 10.000 | 10.000 | 1494.300 | 10.700 | 2.050 | 0.805 | 0.300 | 301.000 | 13.050 |
| 10.000 | 20.000 | 1494.000 | 11.000 | 3.150 | 0.807 | 0.350 | 301.000 | 14.500 |

selected: q 10.000, velocity_sigma 5.000

The candidate is q 10 m^2/s^3 with initial velocity sigma 5 m/s. On seeds 101-120 of the hard
scenario it gives the numbers in its row: cost 12.150, against 153.450 for today's q 0.01 and
sigma 10 in the same table. These are numbers on the tuning seeds only. The candidate has not been
run on seeds 1-20, and the tables above use today's parameters. The candidate is the largest q in
the grid, so the grid does not show where the cost stops falling. No default changed; the decision
is Evan's.
