# Baseline

Run this from the repo root after a build with the `dev` preset. `kite_baseline --help` prints the
fixed scenario and gate settings.

```
build/dev/tools/kite_baseline
```

The CTest test `kite_baseline_doc_matches` fails unless the table below is exactly what this command
prints.

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
into a track and links it greedily to the nearest previous track within its gate. The Sprint 0A.2
filter will be compared against it with the same command.
