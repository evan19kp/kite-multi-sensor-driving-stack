# Association investigation

Sprint 0A.2.1 measures the Kalman tracker's association and tentative-track rules. It changes no
default and no parameter. `KalmanTracker` without an association argument and `kite_baseline`
give the same output as before. Every run here uses seeds 101-120.

## R21: why measurements get stolen

`KalmanTracker` associates in two stages. Confirmed tracks are assigned first, then tentative
tracks get the measurements left over. The R15 builder chose two stages because one joint
assignment let new tentative tracks take measurements from confirmed tracks and created duplicate
tracks. Nobody measured why. This section measures it.

Hypothesis H1 is tested here, not assumed. It says that a new track's velocity is very uncertain,
with an initial velocity sigma of 10 m/s. So its innovation covariance S is large, its squared
Mahalanobis distance to a measurement is small, and it wins measurements from confirmed tracks
that are physically nearer.

### Command

Run this from the repo root after a build with the `dev` preset:

```
build/dev/tools/kite_association_study
```

It runs `KalmanTracker` with `AssociationMode::joint` on seeds 101-120. Joint association is one
`solve_assignment` over all tracks and all measurements, with the same squared Mahalanobis costs
and the same 9.21 gate as the default `two_stage` mode. The scenario and the Kalman parameters are
the `kite_baseline` ones, from `tools/baseline_setup.hpp`, which both programs include. The
tracker's step record is off by default; the tool turns it on. The record gives each track's
predicted position, innovation covariance S, hits, misses and matched measurement at every step.
The CTest test `kite_association_study_doc_matches` fails unless the tables below are exactly what
the tool prints. `ctest -R kite_association_study_output -V` shows them.

### Labelling rule

Ground truth only labels and classifies; the tracker never sees it.

- A measurement is labelled car C when the nearest truth vehicle at the same step is C, it is
  within the 3 m scoring match gate, and the second-nearest vehicle is at least 1 m farther.
  Otherwise the measurement is ambiguous. Every table states how many measurements were ambiguous.
- A track is labelled with the label of the latest measurement it was started from or updated
  with. A track whose latest measurement was ambiguous has no label.

### Definitions

- **Steal.** At one step, a track that was tentative at the start of the step is matched to a
  measurement labelled C, while a track that was confirmed at the start of the step and is
  labelled C misses that step. If more than one such confirmed track misses, the study uses the
  one physically nearest the measurement, and the lower id on a tie. A measurement is at most one
  steal.
- **Distance.** The Euclidean distance from the measurement to the track's predicted position,
  after predict and before update.
- **Score.** The squared Mahalanobis distance y^T S^-1 y with that track's S. The tracker assigns
  on this cost.
- **det S.** The determinant of that track's S.
- **Classes.** The study checks them in this order:
  - **nearer.** The tentative track is physically nearer than the confirmed track.
  - **H1.** The confirmed track is as near or nearer, the tentative track scores lower, and the
    tentative track has the larger det S.
  - **other.** Anything else. The confirmed track is as near or nearer, and either the tentative
    track does not score lower or its det S is not larger. This run had none.
- **Distance ratio.** Tentative distance divided by confirmed distance. Above 1 means the
  confirmed track was physically nearer.
- **det S ratio.** The tentative track's det S divided by the confirmed track's det S. Above 1
  means the tentative track's S was larger.
- **Median.** The middle value after sorting. For an even count it is the mean of the two middle
  values. The tables also give the min and the max.
- **Duplicate.** A steal at step k is followed by a duplicate when, after any of steps k to k + 5,
  at least two of the tracker's output tracks are labelled with the stolen measurement's car. The
  tracker outputs confirmed tracks only.
- **Confirmed outside gate.** The confirmed track's score was above 9.21, so it could not have
  taken the measurement in either association mode.
- **Tentative age.** The tentative track's hits plus misses before the steal step. Age 1 means the
  track started the step before.
- **Born from same car, other car, ambiguous.** The label of the measurement that started the
  tentative track, compared with the stolen measurement's car C.
- **Born outside every gate.** The measurement that started the tentative track scored above 9.21
  against every track at its step.
- **Born inside a gate.** Some track could have taken that measurement but took another one.

### Tables

Association joint, seeds 101-120: 28590 measurements, 314 ambiguous.

| seed | measurements | ambiguous | steals | H1 | nearer | other | duplicates |
|---|---|---|---|---|---|---|---|
| 101 | 1433 | 18 | 13 | 10 | 3 | 0 | 10 |
| 102 | 1433 | 7 | 3 | 2 | 1 | 0 | 2 |
| 103 | 1426 | 18 | 13 | 8 | 5 | 0 | 10 |
| 104 | 1415 | 15 | 4 | 4 | 0 | 0 | 0 |
| 105 | 1435 | 14 | 10 | 8 | 2 | 0 | 8 |
| 106 | 1413 | 20 | 18 | 12 | 6 | 0 | 16 |
| 107 | 1427 | 15 | 4 | 4 | 0 | 0 | 2 |
| 108 | 1426 | 24 | 18 | 13 | 5 | 0 | 14 |
| 109 | 1427 | 12 | 14 | 10 | 4 | 0 | 12 |
| 110 | 1438 | 14 | 11 | 10 | 1 | 0 | 6 |
| 111 | 1425 | 17 | 12 | 7 | 5 | 0 | 8 |
| 112 | 1435 | 12 | 10 | 5 | 5 | 0 | 8 |
| 113 | 1439 | 19 | 9 | 7 | 2 | 0 | 6 |
| 114 | 1428 | 10 | 9 | 5 | 4 | 0 | 6 |
| 115 | 1421 | 14 | 11 | 6 | 5 | 0 | 8 |
| 116 | 1431 | 18 | 14 | 7 | 7 | 0 | 13 |
| 117 | 1427 | 19 | 14 | 10 | 4 | 0 | 10 |
| 118 | 1432 | 18 | 13 | 10 | 3 | 0 | 12 |
| 119 | 1441 | 17 | 21 | 11 | 10 | 0 | 18 |
| 120 | 1438 | 13 | 9 | 7 | 2 | 0 | 6 |
| total | 28590 | 314 | 230 | 156 | 74 | 0 | 175 |

Association joint, seeds 101-120: 28590 measurements, 314 ambiguous.

| class | steals | confirmed outside gate | tentative scores lower | tentative det S larger | distance ratio median | distance ratio min | distance ratio max | det S ratio median | det S ratio min | det S ratio max |
|---|---|---|---|---|---|---|---|---|---|---|
| H1 | 156 | 4 | 156 | 156 | 1.623 | 1.001 | 3.137 | 27.751 | 3.392 | 102.125 |
| nearer | 74 | 1 | 74 | 74 | 0.597 | 0.068 | 0.981 | 7.466 | 1.552 | 100.381 |
| other | 0 | 0 | 0 | 0 | n/a | n/a | n/a | n/a | n/a | n/a |
| all | 230 | 5 | 230 | 230 | 1.296 | 0.068 | 3.137 | 16.860 | 1.552 | 102.125 |

Association joint, seeds 101-120: 28590 measurements, 314 ambiguous.

| class | steals | tentative age median | tentative age min | tentative age max | born from same car | born from other car | born ambiguous | born outside every gate | born inside a gate |
|---|---|---|---|---|---|---|---|---|---|
| H1 | 156 | 3.000 | 1.000 | 4.000 | 44 | 0 | 112 | 156 | 0 |
| nearer | 74 | 2.000 | 1.000 | 4.000 | 37 | 0 | 37 | 74 | 0 |
| other | 0 | n/a | n/a | n/a | 0 | 0 | 0 | 0 | 0 |
| all | 230 | 2.000 | 1.000 | 4.000 | 81 | 0 | 149 | 230 | 0 |

### What the numbers show

With joint association, tentative tracks took 230 measurements from confirmed tracks of the same
car. The 314 ambiguous measurements out of 28590 carry no label, so they cannot count as steals. 175 of the 230 steals were
followed within 5 steps by two confirmed tracks labelled to the same car. 156 steals fit H1. In
those, the confirmed track was as near or nearer, with a distance ratio median of 1.623 and a
range of 1.001 to 3.137. The tentative track still scored lower and had the larger det S, with a
det S ratio median of 27.751 and a range of 3.392 to 102.125. In 4 of the 156, the confirmed
track's score was outside the gate. The other 74 steals do not fit H1 as stated, because the
tentative track was physically nearer, with a distance ratio median of 0.597. In all 74 the
tentative track also had the larger det S. No steal fell into "other". A measurement outside every
track's gate started each of the 230 stealing tracks. Of those measurements, 81 were labelled to
the same car, 149 were ambiguous and 0 were labelled to another car. The tables do not show why
those measurements fell outside every gate.

## R22: comparison on seeds 101-120

This section runs three association modes on seeds 101-120. It chooses no mode and changes no
default or parameter.

### Modes

- **two_stage** (the default). Confirmed tracks are assigned first, then tentative tracks get the
  measurements left over. The cost of a pair is d², the squared Mahalanobis distance.
- **joint.** One `solve_assignment` over all tracks. The cost is d².
- **joint_likelihood.** One `solve_assignment` over all tracks. The cost is d² + ln(det S / det R),
  where R is the measurement noise covariance. This is twice the negative log-likelihood of the
  measurement under the track's prediction, up to a constant. A track with a larger S pays a larger
  ln term. The derivation is in `core/include/kite/kalman_tracker.hpp`.

Every mode gates on d² <= 9.21 alone. A pair outside the gate is never matched, and the ln term
never keeps a pair inside the gate from matching. The CTest test
`KalmanTracker.EveryModeGatesOnTheSquaredDistanceAlone` checks this in all three modes.

### Commands

Run these from the repo root after a build with the `dev` preset, with `<mode>` set to
`two_stage`, `joint` or `joint_likelihood`:

```
build/dev/tools/kite_baseline --tracker kalman --association <mode> --seeds 101-120
build/dev/tools/kite_association_study --association <mode> --summary
```

The CTest tests `kite_baseline_association_<mode>_doc_matches` and
`kite_association_study_<mode>_summary_doc_matches` fail unless the tables below are exactly what
the tools print. The scoring tables carry no mode name, so their tests also need the `<mode>:`
line above each table. `kite_baseline_association_two_stage_output` also fails unless its mean row
equals the one `kite_baseline_kalman_tuning` checks.

### Definitions

Labels, steals and duplicates are as defined in the R21 section.

- **Reverse steal.** At one step, a track that was confirmed at the start of the step is matched to
  a measurement labelled car C, and the measurement that the track was last started from or updated
  with is labelled car D, and D is not C. Neither measurement is ambiguous. Each track counts at most
  once per step.

### Scoring tables

two_stage:

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 101 | 1494 | 11 | 0 | 0.412 | 0 | 301 |
| 102 | 1494 | 11 | 0 | 0.394 | 0 | 301 |
| 103 | 1493 | 12 | 0 | 0.411 | 0 | 301 |
| 104 | 1493 | 12 | 0 | 0.391 | 0 | 301 |
| 105 | 1495 | 10 | 0 | 0.417 | 0 | 301 |
| 106 | 1494 | 11 | 0 | 0.393 | 0 | 301 |
| 107 | 1493 | 12 | 1 | 0.412 | 0 | 301 |
| 108 | 1494 | 11 | 0 | 0.390 | 0 | 301 |
| 109 | 1493 | 12 | 0 | 0.463 | 0 | 301 |
| 110 | 1495 | 10 | 0 | 0.434 | 0 | 301 |
| 111 | 1494 | 11 | 0 | 0.426 | 0 | 301 |
| 112 | 1495 | 10 | 0 | 0.405 | 0 | 301 |
| 113 | 1495 | 10 | 0 | 0.451 | 0 | 301 |
| 114 | 1493 | 12 | 0 | 0.393 | 0 | 301 |
| 115 | 1495 | 10 | 0 | 0.410 | 0 | 301 |
| 116 | 1494 | 11 | 0 | 0.409 | 0 | 301 |
| 117 | 1493 | 12 | 0 | 0.417 | 0 | 301 |
| 118 | 1495 | 10 | 0 | 0.414 | 0 | 301 |
| 119 | 1495 | 10 | 0 | 0.438 | 0 | 301 |
| 120 | 1495 | 10 | 0 | 0.403 | 0 | 301 |
| mean | 1494.100 | 10.900 | 0.050 | 0.414 | 0.000 | 301.000 |

joint:

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 101 | 1494 | 11 | 46 | 0.437 | 5 | 301 |
| 102 | 1494 | 11 | 5 | 0.395 | 0 | 301 |
| 103 | 1492 | 13 | 23 | 0.442 | 4 | 301 |
| 104 | 1493 | 12 | 0 | 0.392 | 0 | 301 |
| 105 | 1495 | 10 | 21 | 0.419 | 0 | 301 |
| 106 | 1494 | 11 | 66 | 0.414 | 3 | 301 |
| 107 | 1493 | 12 | 6 | 0.410 | 0 | 301 |
| 108 | 1494 | 11 | 46 | 0.448 | 4 | 301 |
| 109 | 1493 | 12 | 51 | 0.486 | 2 | 301 |
| 110 | 1495 | 10 | 34 | 0.434 | 2 | 301 |
| 111 | 1494 | 11 | 21 | 0.443 | 1 | 301 |
| 112 | 1493 | 12 | 21 | 0.415 | 3 | 301 |
| 113 | 1495 | 10 | 27 | 0.486 | 4 | 301 |
| 114 | 1493 | 12 | 13 | 0.416 | 1 | 301 |
| 115 | 1495 | 10 | 50 | 0.433 | 3 | 301 |
| 116 | 1494 | 11 | 41 | 0.488 | 4 | 301 |
| 117 | 1493 | 12 | 42 | 0.456 | 7 | 301 |
| 118 | 1495 | 10 | 99 | 0.435 | 1 | 301 |
| 119 | 1495 | 10 | 48 | 0.486 | 5 | 301 |
| 120 | 1495 | 10 | 19 | 0.403 | 2 | 301 |
| mean | 1493.950 | 11.050 | 33.950 | 0.437 | 2.550 | 301.000 |

joint_likelihood:

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 101 | 1494 | 11 | 0 | 0.415 | 0 | 301 |
| 102 | 1494 | 11 | 0 | 0.394 | 0 | 301 |
| 103 | 1493 | 12 | 0 | 0.414 | 0 | 301 |
| 104 | 1493 | 12 | 0 | 0.391 | 0 | 301 |
| 105 | 1495 | 10 | 0 | 0.417 | 0 | 301 |
| 106 | 1494 | 11 | 10 | 0.402 | 0 | 301 |
| 107 | 1493 | 12 | 1 | 0.412 | 0 | 301 |
| 108 | 1494 | 11 | 6 | 0.444 | 2 | 301 |
| 109 | 1493 | 12 | 21 | 0.479 | 1 | 301 |
| 110 | 1495 | 10 | 0 | 0.436 | 0 | 301 |
| 111 | 1494 | 11 | 5 | 0.433 | 2 | 301 |
| 112 | 1495 | 10 | 0 | 0.406 | 0 | 301 |
| 113 | 1495 | 10 | 0 | 0.451 | 0 | 301 |
| 114 | 1493 | 12 | 2 | 0.411 | 1 | 301 |
| 115 | 1495 | 10 | 23 | 0.425 | 0 | 301 |
| 116 | 1494 | 11 | 8 | 0.416 | 0 | 301 |
| 117 | 1493 | 12 | 13 | 0.440 | 3 | 301 |
| 118 | 1495 | 10 | 0 | 0.414 | 0 | 301 |
| 119 | 1495 | 10 | 6 | 0.445 | 0 | 301 |
| 120 | 1495 | 10 | 0 | 0.403 | 0 | 301 |
| mean | 1494.100 | 10.900 | 4.750 | 0.422 | 0.450 | 301.000 |

### Steal tables

Association two_stage, seeds 101-120: 28590 measurements, 314 ambiguous.

| seed | measurements | ambiguous | steals | duplicates | reverse steals |
|---|---|---|---|---|---|
| 101 | 1433 | 18 | 0 | 0 | 0 |
| 102 | 1433 | 7 | 0 | 0 | 0 |
| 103 | 1426 | 18 | 0 | 0 | 0 |
| 104 | 1415 | 15 | 0 | 0 | 0 |
| 105 | 1435 | 14 | 0 | 0 | 0 |
| 106 | 1413 | 20 | 0 | 0 | 0 |
| 107 | 1427 | 15 | 0 | 0 | 0 |
| 108 | 1426 | 24 | 0 | 0 | 0 |
| 109 | 1427 | 12 | 1 | 0 | 0 |
| 110 | 1438 | 14 | 1 | 0 | 0 |
| 111 | 1425 | 17 | 0 | 0 | 0 |
| 112 | 1435 | 12 | 0 | 0 | 0 |
| 113 | 1439 | 19 | 0 | 0 | 0 |
| 114 | 1428 | 10 | 0 | 0 | 0 |
| 115 | 1421 | 14 | 0 | 0 | 0 |
| 116 | 1431 | 18 | 1 | 0 | 0 |
| 117 | 1427 | 19 | 0 | 0 | 0 |
| 118 | 1432 | 18 | 0 | 0 | 0 |
| 119 | 1441 | 17 | 0 | 0 | 0 |
| 120 | 1438 | 13 | 0 | 0 | 0 |
| total | 28590 | 314 | 3 | 0 | 0 |

Association joint, seeds 101-120: 28590 measurements, 314 ambiguous.

| seed | measurements | ambiguous | steals | duplicates | reverse steals |
|---|---|---|---|---|---|
| 101 | 1433 | 18 | 13 | 10 | 0 |
| 102 | 1433 | 7 | 3 | 2 | 0 |
| 103 | 1426 | 18 | 13 | 10 | 0 |
| 104 | 1415 | 15 | 4 | 0 | 0 |
| 105 | 1435 | 14 | 10 | 8 | 0 |
| 106 | 1413 | 20 | 18 | 16 | 0 |
| 107 | 1427 | 15 | 4 | 2 | 0 |
| 108 | 1426 | 24 | 18 | 14 | 0 |
| 109 | 1427 | 12 | 14 | 12 | 0 |
| 110 | 1438 | 14 | 11 | 6 | 0 |
| 111 | 1425 | 17 | 12 | 8 | 0 |
| 112 | 1435 | 12 | 10 | 8 | 0 |
| 113 | 1439 | 19 | 9 | 6 | 0 |
| 114 | 1428 | 10 | 9 | 6 | 0 |
| 115 | 1421 | 14 | 11 | 8 | 0 |
| 116 | 1431 | 18 | 14 | 13 | 0 |
| 117 | 1427 | 19 | 14 | 10 | 0 |
| 118 | 1432 | 18 | 13 | 12 | 0 |
| 119 | 1441 | 17 | 21 | 18 | 0 |
| 120 | 1438 | 13 | 9 | 6 | 0 |
| total | 28590 | 314 | 230 | 175 | 0 |

Association joint_likelihood, seeds 101-120: 28590 measurements, 314 ambiguous.

| seed | measurements | ambiguous | steals | duplicates | reverse steals |
|---|---|---|---|---|---|
| 101 | 1433 | 18 | 2 | 0 | 0 |
| 102 | 1433 | 7 | 0 | 0 | 0 |
| 103 | 1426 | 18 | 2 | 0 | 0 |
| 104 | 1415 | 15 | 0 | 0 | 0 |
| 105 | 1435 | 14 | 1 | 0 | 0 |
| 106 | 1413 | 20 | 5 | 4 | 0 |
| 107 | 1427 | 15 | 0 | 0 | 0 |
| 108 | 1426 | 24 | 5 | 4 | 0 |
| 109 | 1427 | 12 | 3 | 2 | 0 |
| 110 | 1438 | 14 | 3 | 0 | 0 |
| 111 | 1425 | 17 | 4 | 2 | 0 |
| 112 | 1435 | 12 | 1 | 0 | 0 |
| 113 | 1439 | 19 | 1 | 0 | 0 |
| 114 | 1428 | 10 | 4 | 2 | 0 |
| 115 | 1421 | 14 | 5 | 4 | 0 |
| 116 | 1431 | 18 | 5 | 2 | 0 |
| 117 | 1427 | 19 | 4 | 2 | 0 |
| 118 | 1432 | 18 | 1 | 0 | 0 |
| 119 | 1441 | 17 | 5 | 2 | 0 |
| 120 | 1438 | 13 | 1 | 0 | 0 |
| total | 28590 | 314 | 52 | 24 | 0 |

### What the numbers show

The mean matched count was 1494.100 with two_stage and joint_likelihood and 1493.950 with joint.
The mean false count was 0.050 with two_stage, 33.950 with joint and 4.750 with joint_likelihood,
and the mean id switches were 0.000, 2.550 and 0.450. Tentative tracks stole 3 measurements with
two_stage, 230 with joint and 52 with joint_likelihood. Of those steals, 0, 175 and 24 were
followed by a duplicate. No mode had a reverse steal. All three runs skipped the same 314 ambiguous
measurements out of 28590.

## R23: tentative tracks dropped

This section measures how often the tracker drops a tentative track that a real car started. It
changes no default, no parameter and no drop rule. It uses the default `two_stage` mode and the
`kite_baseline` scenario and Kalman parameters from `tools/baseline_setup.hpp`.

### Commands

Run these from the repo root after a build with the `dev` preset:

```
build/dev/tools/kite_drop_study --monte-carlo
build/dev/tools/kite_drop_study --tuning
```

The CTest tests `kite_drop_study_monte_carlo_doc_matches` and `kite_drop_study_tuning_doc_matches`
fail unless the tables below are exactly what the tool prints. `ctest -R kite_drop_study -V` shows
them.

### The drop rule and the formula

A tentative track is confirmed at its 3rd hit and dropped at its 3rd miss, and the step that
starts it counts as hit 1. So a track born from a car is dropped exactly when at most M - 2 = 1 of
the N - 1 = 4 steps after the birth give it a hit, with M = 3 and N = 5. If the car is detected
at each step with probability p, independently, and the track always gets a detection of its car,
that chance is the sum for k from 0 to M - 2 of C(N - 1, k) p^k (1 - p)^(N - 1 - k). At p = 0.95
it is 0.05^4 + 4 * 0.95 * 0.05^3. The tool computes it from the tracker's M and N. The formula
ignores gating losses and measurements taken by other tracks. The study measures them.

### Monte Carlo

- One vehicle and the `kite_baseline` scenario otherwise. Each seed runs 10 s, which is 101 steps,
  with the detection probability p from the table. The seeds are 100001-120000 for p = 0.95, 120001-140000 for 0.90,
  140001-160000 for 0.80 and 160001-180000 for 0.70, so 20000 seeds each.
- **Birth.** The first tentative track started from the car's measurements, which is the track
  started at the first step that has a measurement. The tool follows only that track through the
  tracker's step record. It is confirmed when the tracker's output has it. It is dropped when the
  next update's record no longer lists it.
- **Run length.** A birth is confirmed or dropped within 5 updates after the birth step. The run
  is long enough unless the car's first measurement comes within the last 5 steps. Births still
  open at the end of the run count as undecided, and seeds in which the car produced no
  measurement at all count as seeds with no measurement. Both columns are in the second table.
- **Misses.** Each miss of a dropped birth is "no measurement" when that step had no measurement,
  "outside gate" when no measurement of that step had a squared Mahalanobis distance of at most
  9.21 to the track, and "inside gate, taken by another track" when one did but the track did not
  get it. The distance uses the step record's predicted position and innovation covariance.
- **Rate.** Births are confirmed plus dropped. The measured rate is dropped over births, with the
  95 % Wilson interval for z = 1.959963984540054. Rates and interval bounds print as percentages
  with 3 decimals.

### Monte Carlo tables

Monte Carlo, 1 vehicle, two_stage, 3 hits in 5 steps, 10.000 s per seed, 20000 seeds per detection probability. Rates and bounds are percentages.

| detection probability | seeds | births | dropped | measured rate (%) | Wilson 95 % low (%) | Wilson 95 % high (%) | formula (%) | formula inside interval |
|---|---|---|---|---|---|---|---|---|
| 0.950 | 100001-120000 | 20000 | 134 | 0.670 | 0.566 | 0.793 | 0.048 | no |
| 0.900 | 120001-140000 | 20000 | 207 | 1.035 | 0.904 | 1.185 | 0.370 | no |
| 0.800 | 140001-160000 | 20000 | 662 | 3.310 | 3.071 | 3.567 | 2.720 | no |
| 0.700 | 160001-180000 | 20000 | 1841 | 9.205 | 8.812 | 9.614 | 8.370 | no |

Monte Carlo, 1 vehicle, two_stage, 3 hits in 5 steps, 10.000 s per seed, 20000 seeds per detection probability. Rates and bounds are percentages.

| detection probability | confirmed | dropped | seeds with no measurement | undecided | misses: no measurement | misses: outside gate | misses: inside gate, taken by another track |
|---|---|---|---|---|---|---|---|
| 0.950 | 19866 | 134 | 0 | 0 | 55 | 133 | 214 |
| 0.900 | 19793 | 207 | 0 | 0 | 256 | 150 | 215 |
| 0.800 | 19338 | 662 | 0 | 0 | 1728 | 124 | 134 |
| 0.700 | 18159 | 1841 | 0 | 0 | 5303 | 108 | 112 |

### Seeds 101-120

The `kite_baseline` scenario, five vehicles, 301 steps, labelled by the R21 rule.

- **Tentative track.** Every track the tracker starts. It is confirmed when the tracker's output
  has it and dropped when the next update's record no longer lists it. A track dropped by the
  final update of a run is unresolved, with any track still tentative at the end.
- **A car's confirmed track.** A track belongs to the car of its latest matched measurement that
  has a label. Matching an ambiguous measurement never changes it, so a confirmed track that takes
  an ambiguous measurement and then coasts still belongs to its car. A car has a confirmed track at
  a step when some track in the step's output belongs to it, and a coasting confirmed track is in
  the output. A track that starts on an ambiguous measurement belongs to no car until it is matched
  to a labelled one. This refines the R21 rule, where a match to an ambiguous measurement cleared
  the track's label, for the tables of this section only. The R21 and R22 tables are unchanged.
- **Real birth lost.** A dropped track whose birth measurement is labelled car C, when no confirmed
  track belonged to C after the birth step's update.
- **Birth to a car with a confirmed track.** A dropped track whose birth measurement is labelled
  car C, when a confirmed track belonging to C existed after the birth step's update. These are the
  duplicates and strays among the labelled births.
- **Ambiguous birth.** A dropped track whose birth measurement had no label. The study cannot say
  whether it belonged to a car, so it is counted apart from the other two.
- **Steps to confirmation.** For a car, the step of its first measurement with a label to the first
  step whose output has a confirmed track belonging to that car. The mean is over the cars that
  were confirmed; the table counts the cars that never were. Cars are those with at least one
  labelled measurement. A car can take fewer than 2 steps only when the track that confirms had
  hits on measurements without the car's label before the car's first labelled measurement. The
  seed 116 mean of 1.800 over 5 cars shows that this happens.

Tentative tracks, two_stage, seeds 101-120: 28590 measurements, 314 ambiguous.

| seed | tentative tracks | confirmed | dropped | unresolved | real births lost | birth to a car with a confirmed track | ambiguous birth |
|---|---|---|---|---|---|---|---|
| 101 | 19 | 5 | 14 | 0 | 0 | 5 | 9 |
| 102 | 11 | 5 | 5 | 1 | 0 | 1 | 4 |
| 103 | 18 | 5 | 13 | 0 | 0 | 3 | 10 |
| 104 | 13 | 5 | 8 | 0 | 0 | 0 | 8 |
| 105 | 13 | 5 | 8 | 0 | 0 | 0 | 8 |
| 106 | 22 | 5 | 17 | 0 | 0 | 4 | 13 |
| 107 | 14 | 5 | 9 | 0 | 0 | 2 | 7 |
| 108 | 21 | 5 | 16 | 0 | 0 | 3 | 13 |
| 109 | 15 | 5 | 10 | 0 | 0 | 4 | 6 |
| 110 | 15 | 5 | 10 | 0 | 0 | 1 | 9 |
| 111 | 14 | 5 | 9 | 0 | 0 | 1 | 8 |
| 112 | 15 | 5 | 10 | 0 | 0 | 5 | 5 |
| 113 | 19 | 5 | 14 | 0 | 0 | 4 | 10 |
| 114 | 15 | 5 | 10 | 0 | 0 | 1 | 9 |
| 115 | 15 | 5 | 9 | 1 | 0 | 4 | 5 |
| 116 | 18 | 5 | 13 | 0 | 0 | 4 | 9 |
| 117 | 20 | 5 | 15 | 0 | 0 | 4 | 11 |
| 118 | 17 | 5 | 11 | 1 | 0 | 4 | 7 |
| 119 | 22 | 5 | 17 | 0 | 0 | 6 | 11 |
| 120 | 17 | 5 | 12 | 0 | 0 | 1 | 11 |
| total | 333 | 100 | 230 | 3 | 0 | 57 | 173 |

Tentative tracks, two_stage, seeds 101-120: 28590 measurements, 314 ambiguous.

| seed | measurements | ambiguous | cars | cars never confirmed | mean steps, first measurement to confirmation |
|---|---|---|---|---|---|
| 101 | 1433 | 18 | 5 | 0 | 2.200 |
| 102 | 1433 | 7 | 5 | 0 | 2.000 |
| 103 | 1426 | 18 | 5 | 0 | 2.400 |
| 104 | 1415 | 15 | 5 | 0 | 2.200 |
| 105 | 1435 | 14 | 5 | 0 | 2.000 |
| 106 | 1413 | 20 | 5 | 0 | 2.200 |
| 107 | 1427 | 15 | 5 | 0 | 2.200 |
| 108 | 1426 | 24 | 5 | 0 | 2.200 |
| 109 | 1427 | 12 | 5 | 0 | 2.200 |
| 110 | 1438 | 14 | 5 | 0 | 2.000 |
| 111 | 1425 | 17 | 5 | 0 | 2.200 |
| 112 | 1435 | 12 | 5 | 0 | 2.000 |
| 113 | 1439 | 19 | 5 | 0 | 2.000 |
| 114 | 1428 | 10 | 5 | 0 | 2.400 |
| 115 | 1421 | 14 | 5 | 0 | 2.000 |
| 116 | 1431 | 18 | 5 | 0 | 1.800 |
| 117 | 1427 | 19 | 5 | 0 | 2.400 |
| 118 | 1432 | 18 | 5 | 0 | 2.000 |
| 119 | 1441 | 17 | 5 | 0 | 2.000 |
| 120 | 1438 | 13 | 5 | 0 | 2.000 |
| total | 28590 | 314 | 100 | 0 | 2.120 |

### What the numbers show

The formula lies outside the 95 % Wilson interval at all four detection probabilities. At p = 0.95
the tracker dropped 134 of 20000 births, 0.670 % with an interval of 0.566 to 0.793 %, against
0.048 % from the formula. Of the 402 misses of those 134 dropped births, 55 were steps with no
measurement, 133 were outside the gate and 214 were inside the gate but taken by another track.
As p falls to 0.70 the no-measurement misses rise from 55 to 5303 while the outside-gate misses
stay between 108 and 150. The tables do not show why another track took those measurements. On
seeds 101-120 the tracker started 333 tentative tracks, confirmed 100 and dropped 230. Of the
dropped, none was a real birth lost, 57 were births to a car that already had a confirmed track
and 173 had an ambiguous birth label. Every one of the 100 cars was confirmed, a mean of 2.120
steps after its first labelled measurement.

## Hard scenario (seeds 101-120)

This section runs the three association modes and the tentative-track study on the hard scenario
of `docs/BASELINE.md`: the `kite_baseline` scenario with turning, speed changes and clutter. It
chooses no mode and changes no default, no parameter and no table above. The scenario and the
Kalman parameters come from `tools/baseline_setup.hpp`, as before. Every run uses seeds 101-120,
never the reporting seeds 1-20.

### Commands

Run these from the repo root after a build with the `dev` preset, with `<mode>` set to `two_stage`,
`joint` or `joint_likelihood`:

```
build/dev/tools/kite_baseline --scenario hard --tracker kalman --association <mode> --seeds 101-120
build/dev/tools/kite_association_study --scenario hard --association <mode> --summary
build/dev/tools/kite_drop_study --scenario hard --tuning
```

`--scenario straight` is the default of both study tools and prints what they printed before the
flag existed; `kite_association_study_scenario_straight_matches_default`,
`kite_association_study_scenario_straight_summary_matches_default` and
`kite_drop_study_scenario_straight_matches_default` compare the two outputs byte for byte.
`kite_drop_study --monte-carlo` takes `--scenario straight` only and exits 2 with `--scenario
hard`. The Monte Carlo study of R23 is one vehicle on seeds from 100001, so the hard scenario's
output of the drop study is the tentative-track table on seeds 101-120 below.

The CTest tests `kite_baseline_hard_association_<mode>_doc_matches`,
`kite_association_study_hard_<mode>_summary_doc_matches` and
`kite_drop_study_hard_tuning_doc_matches` fail unless the tables below are exactly what the tools
print. The scoring tables carry no mode name, so their tests also need the `hard <mode>:` line above
each table. `ctest -R "hard" -V` shows the outputs.

### Labels

Labels are the R21 rule, with one more value. Ground truth only labels and classifies; no tracker
sees it, and the generator marks no measurement as clutter.

- **Car C.** The nearest truth vehicle is C, it is within 3 m, and the second-nearest vehicle is at
  least 1 m farther.
- **Clutter.** No truth vehicle is within 3 m.
- **Ambiguous.** A truth vehicle is within 3 m but the second-nearest vehicle is less than 1 m
  farther than the nearest.

The label comes only from the distance to the truth vehicles, in `tools/study_common.hpp`. So a
false detection within 3 m of a car is labelled as that car, or ambiguous, and a real detection whose
noise put it more than 3 m from its car is labelled clutter. A measurement with no car label,
clutter or ambiguous, is never a steal and never counts in a reverse steal. Every table below states
its clutter and ambiguous counts. The straight tables above count both together as ambiguous.

### Definitions

Steals, duplicates, the steal columns and reverse steals are as defined in the R21 and R22
sections. A reverse steal needs two measurements with car labels, so a clutter measurement counts
like an ambiguous one there.

- **Duplicate tentative track.** Number the steps of a run from 0. Every track the tracker starts is
  tentative at its birth step. A track is a duplicate when its birth measurement is labelled car C
  and some earlier-started track also has a birth measurement labelled C with a birth step at most 5
  steps before this track's. Two births in the same step are 0 steps apart. The earlier track may
  have been dropped, confirmed or still tentative. The count is of the later tracks, so births of
  one car at steps 0 and 5 make one duplicate, and births at steps 0, 3 and 6 make two. A track
  born from a clutter or ambiguous measurement is never a duplicate.
- **Clutter birth.** A dropped track whose birth measurement is labelled clutter. The tables of the
  R23 section count these with the ambiguous births; here they are apart.

### Scoring tables

hard two_stage:

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 101 | 1465 | 40 | 91 | 1.142 | 20 | 301 |
| 102 | 1468 | 37 | 87 | 1.235 | 20 | 301 |
| 103 | 1457 | 48 | 119 | 1.101 | 21 | 301 |
| 104 | 1471 | 34 | 79 | 1.078 | 17 | 301 |
| 105 | 1454 | 51 | 116 | 1.161 | 23 | 301 |
| 106 | 1456 | 49 | 97 | 1.193 | 24 | 301 |
| 107 | 1461 | 44 | 93 | 1.201 | 23 | 301 |
| 108 | 1460 | 45 | 100 | 1.270 | 25 | 301 |
| 109 | 1457 | 48 | 114 | 1.137 | 24 | 301 |
| 110 | 1476 | 29 | 57 | 1.068 | 16 | 301 |
| 111 | 1469 | 36 | 72 | 1.081 | 19 | 301 |
| 112 | 1471 | 34 | 72 | 1.112 | 19 | 301 |
| 113 | 1478 | 27 | 86 | 1.029 | 18 | 301 |
| 114 | 1455 | 50 | 114 | 1.263 | 26 | 301 |
| 115 | 1445 | 60 | 97 | 1.208 | 20 | 301 |
| 116 | 1472 | 33 | 65 | 0.918 | 12 | 301 |
| 117 | 1483 | 22 | 34 | 0.838 | 11 | 301 |
| 118 | 1446 | 59 | 121 | 1.238 | 27 | 301 |
| 119 | 1458 | 47 | 92 | 1.173 | 19 | 301 |
| 120 | 1457 | 48 | 114 | 1.189 | 24 | 301 |
| mean | 1462.950 | 42.050 | 91.000 | 1.132 | 20.400 | 301.000 |

With two_stage the mean row has 1462.950 matched, 42.050 missed and 91.000 false tracks per seed,
an rmse_m of 1.132 and 20.400 id switches. Id switches run from 11 on seed 117 to 27 on seed 118.

hard joint:

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 101 | 1482 | 23 | 161 | 1.025 | 43 | 301 |
| 102 | 1485 | 20 | 128 | 1.031 | 38 | 301 |
| 103 | 1486 | 19 | 160 | 0.975 | 35 | 301 |
| 104 | 1485 | 20 | 150 | 0.915 | 34 | 301 |
| 105 | 1480 | 25 | 182 | 1.059 | 45 | 301 |
| 106 | 1479 | 26 | 123 | 1.030 | 41 | 301 |
| 107 | 1484 | 21 | 187 | 1.077 | 45 | 301 |
| 108 | 1472 | 33 | 178 | 1.092 | 49 | 301 |
| 109 | 1484 | 21 | 180 | 1.012 | 44 | 301 |
| 110 | 1489 | 16 | 131 | 0.909 | 36 | 301 |
| 111 | 1481 | 24 | 147 | 0.978 | 45 | 301 |
| 112 | 1488 | 17 | 183 | 1.003 | 45 | 301 |
| 113 | 1493 | 12 | 142 | 0.941 | 34 | 301 |
| 114 | 1481 | 24 | 144 | 1.089 | 43 | 301 |
| 115 | 1484 | 21 | 108 | 0.983 | 37 | 301 |
| 116 | 1482 | 23 | 139 | 0.840 | 27 | 301 |
| 117 | 1492 | 13 | 112 | 0.848 | 27 | 301 |
| 118 | 1475 | 30 | 156 | 1.146 | 46 | 301 |
| 119 | 1482 | 23 | 171 | 1.001 | 42 | 301 |
| 120 | 1479 | 26 | 140 | 1.034 | 42 | 301 |
| mean | 1483.150 | 21.850 | 151.100 | 1.000 | 39.900 | 301.000 |

With joint the mean row has 1483.150 matched, 21.850 missed and 151.100 false tracks per seed, an
rmse_m of 1.000 and 39.900 id switches. Id switches run from 27 on seeds 116 and 117 to 49 on seed
108.

hard joint_likelihood:

| seed | matched | missed | false | rmse_m | id_switches | steps |
|---|---|---|---|---|---|---|
| 101 | 1477 | 28 | 110 | 1.071 | 27 | 301 |
| 102 | 1480 | 25 | 140 | 1.098 | 35 | 301 |
| 103 | 1484 | 21 | 109 | 1.006 | 27 | 301 |
| 104 | 1482 | 23 | 127 | 0.977 | 31 | 301 |
| 105 | 1479 | 26 | 96 | 1.078 | 29 | 301 |
| 106 | 1478 | 27 | 107 | 1.058 | 30 | 301 |
| 107 | 1485 | 20 | 134 | 1.066 | 37 | 301 |
| 108 | 1471 | 34 | 147 | 1.155 | 37 | 301 |
| 109 | 1481 | 24 | 146 | 1.032 | 28 | 301 |
| 110 | 1493 | 12 | 84 | 0.913 | 26 | 301 |
| 111 | 1475 | 30 | 102 | 1.007 | 32 | 301 |
| 112 | 1487 | 18 | 104 | 0.974 | 28 | 301 |
| 113 | 1491 | 14 | 79 | 0.990 | 23 | 301 |
| 114 | 1479 | 26 | 139 | 1.154 | 42 | 301 |
| 115 | 1483 | 22 | 115 | 1.055 | 33 | 301 |
| 116 | 1482 | 23 | 70 | 0.844 | 18 | 301 |
| 117 | 1489 | 16 | 73 | 0.869 | 19 | 301 |
| 118 | 1467 | 38 | 142 | 1.162 | 41 | 301 |
| 119 | 1481 | 24 | 113 | 1.058 | 29 | 301 |
| 120 | 1474 | 31 | 135 | 1.091 | 35 | 301 |
| mean | 1480.900 | 24.100 | 113.600 | 1.033 | 30.350 | 301.000 |

With joint_likelihood the mean row has 1480.900 matched, 24.100 missed and 113.600 false tracks per
seed, an rmse_m of 1.033 and 30.350 id switches. Id switches run from 18 on seed 116 to 42 on seed
114.

### Steal tables

Association two_stage, hard scenario, seeds 101-120: 40655 measurements, 12357 clutter, 23 ambiguous.

| seed | measurements | clutter | ambiguous | steals | duplicates | reverse steals |
|---|---|---|---|---|---|---|
| 101 | 2039 | 616 | 3 | 49 | 43 | 0 |
| 102 | 2063 | 633 | 0 | 43 | 38 | 0 |
| 103 | 2012 | 615 | 0 | 49 | 37 | 0 |
| 104 | 2032 | 617 | 0 | 42 | 35 | 0 |
| 105 | 2081 | 653 | 0 | 53 | 45 | 0 |
| 106 | 2009 | 591 | 0 | 52 | 47 | 0 |
| 107 | 2060 | 643 | 2 | 53 | 43 | 2 |
| 108 | 1985 | 590 | 0 | 61 | 46 | 0 |
| 109 | 2021 | 590 | 0 | 52 | 44 | 0 |
| 110 | 2013 | 594 | 0 | 41 | 34 | 0 |
| 111 | 2019 | 610 | 4 | 43 | 40 | 0 |
| 112 | 2042 | 622 | 0 | 48 | 36 | 2 |
| 113 | 2029 | 603 | 0 | 38 | 34 | 0 |
| 114 | 2020 | 601 | 0 | 61 | 49 | 0 |
| 115 | 2017 | 605 | 14 | 48 | 42 | 0 |
| 116 | 2045 | 627 | 0 | 28 | 20 | 0 |
| 117 | 2043 | 652 | 0 | 23 | 18 | 0 |
| 118 | 2023 | 620 | 0 | 62 | 54 | 0 |
| 119 | 2042 | 632 | 0 | 48 | 36 | 0 |
| 120 | 2060 | 643 | 0 | 56 | 48 | 0 |
| total | 40655 | 12357 | 23 | 950 | 789 | 4 |

With two_stage, tentative tracks stole 950 measurements from confirmed tracks of the same car, and
789 of the steals were followed within 5 steps by two confirmed tracks labelled to the same car.
There were 4 reverse steals, 2 on seed 107 and 2 on seed 112. The 12357 clutter and 23 ambiguous
measurements out of 40655 carry no car label, so they cannot count as steals.

Association joint, hard scenario, seeds 101-120: 40655 measurements, 12357 clutter, 23 ambiguous.

| seed | measurements | clutter | ambiguous | steals | duplicates | reverse steals |
|---|---|---|---|---|---|---|
| 101 | 2039 | 616 | 3 | 69 | 65 | 1 |
| 102 | 2063 | 633 | 0 | 67 | 66 | 0 |
| 103 | 2012 | 615 | 0 | 67 | 62 | 0 |
| 104 | 2032 | 617 | 0 | 67 | 64 | 0 |
| 105 | 2081 | 653 | 0 | 79 | 73 | 0 |
| 106 | 2009 | 591 | 0 | 77 | 72 | 0 |
| 107 | 2060 | 643 | 2 | 79 | 75 | 4 |
| 108 | 1985 | 590 | 0 | 93 | 89 | 0 |
| 109 | 2021 | 590 | 0 | 82 | 76 | 0 |
| 110 | 2013 | 594 | 0 | 61 | 54 | 0 |
| 111 | 2019 | 610 | 4 | 76 | 72 | 0 |
| 112 | 2042 | 622 | 0 | 68 | 68 | 1 |
| 113 | 2029 | 603 | 0 | 67 | 65 | 0 |
| 114 | 2020 | 601 | 0 | 89 | 85 | 0 |
| 115 | 2017 | 605 | 14 | 69 | 66 | 0 |
| 116 | 2045 | 627 | 0 | 52 | 49 | 0 |
| 117 | 2043 | 652 | 0 | 48 | 44 | 0 |
| 118 | 2023 | 620 | 0 | 79 | 74 | 0 |
| 119 | 2042 | 632 | 0 | 71 | 69 | 1 |
| 120 | 2060 | 643 | 0 | 79 | 72 | 0 |
| total | 40655 | 12357 | 23 | 1439 | 1360 | 7 |

With joint, tentative tracks stole 1439 measurements from confirmed tracks of the same car, and
1360 of the steals were followed within 5 steps by two confirmed tracks labelled to the same car.
There were 7 reverse steals, 4 on seed 107 and 1 each on seeds 101, 112 and 119. The 12357 clutter
and 23 ambiguous measurements out of 40655 carry no car label, so they cannot count as steals.

Association joint_likelihood, hard scenario, seeds 101-120: 40655 measurements, 12357 clutter, 23 ambiguous.

| seed | measurements | clutter | ambiguous | steals | duplicates | reverse steals |
|---|---|---|---|---|---|---|
| 101 | 2039 | 616 | 3 | 57 | 53 | 0 |
| 102 | 2063 | 633 | 0 | 65 | 61 | 0 |
| 103 | 2012 | 615 | 0 | 61 | 54 | 0 |
| 104 | 2032 | 617 | 0 | 57 | 54 | 0 |
| 105 | 2081 | 653 | 0 | 58 | 53 | 0 |
| 106 | 2009 | 591 | 0 | 63 | 58 | 0 |
| 107 | 2060 | 643 | 2 | 59 | 52 | 5 |
| 108 | 1985 | 590 | 0 | 74 | 71 | 0 |
| 109 | 2021 | 590 | 0 | 65 | 62 | 0 |
| 110 | 2013 | 594 | 0 | 48 | 45 | 0 |
| 111 | 2019 | 610 | 4 | 55 | 49 | 0 |
| 112 | 2042 | 622 | 0 | 55 | 51 | 1 |
| 113 | 2029 | 603 | 0 | 51 | 46 | 0 |
| 114 | 2020 | 601 | 0 | 77 | 74 | 0 |
| 115 | 2017 | 605 | 14 | 64 | 60 | 2 |
| 116 | 2045 | 627 | 0 | 37 | 32 | 0 |
| 117 | 2043 | 652 | 0 | 30 | 26 | 0 |
| 118 | 2023 | 620 | 0 | 71 | 65 | 0 |
| 119 | 2042 | 632 | 0 | 56 | 52 | 0 |
| 120 | 2060 | 643 | 0 | 69 | 60 | 0 |
| total | 40655 | 12357 | 23 | 1172 | 1078 | 8 |

With joint_likelihood, tentative tracks stole 1172 measurements from confirmed tracks of the same
car, and 1078 of the steals were followed within 5 steps by two confirmed tracks labelled to the
same car. There were 8 reverse steals, 5 on seed 107, 2 on seed 115 and 1 on seed 112. The 12357
clutter and 23 ambiguous measurements out of 40655 carry no car label, so they cannot count as
steals.

### Tentative tracks

Tentative tracks, two_stage, hard scenario, seeds 101-120: 40655 measurements, 12357 clutter, 23 ambiguous.

| seed | tentative tracks | confirmed | dropped | unresolved | real births lost | birth to a car with a confirmed track | clutter birth | ambiguous birth | duplicate tentative tracks |
|---|---|---|---|---|---|---|---|---|---|
| 101 | 650 | 25 | 618 | 7 | 0 | 33 | 585 | 0 | 13 |
| 102 | 664 | 26 | 628 | 10 | 0 | 35 | 593 | 0 | 10 |
| 103 | 649 | 26 | 618 | 5 | 0 | 28 | 590 | 0 | 4 |
| 104 | 652 | 22 | 620 | 10 | 0 | 29 | 591 | 0 | 6 |
| 105 | 693 | 29 | 653 | 11 | 0 | 28 | 625 | 0 | 5 |
| 106 | 627 | 28 | 593 | 6 | 0 | 32 | 561 | 0 | 11 |
| 107 | 680 | 28 | 635 | 17 | 0 | 34 | 601 | 0 | 9 |
| 108 | 629 | 30 | 589 | 10 | 0 | 38 | 551 | 0 | 11 |
| 109 | 624 | 28 | 591 | 5 | 0 | 35 | 556 | 0 | 12 |
| 110 | 616 | 21 | 586 | 9 | 0 | 23 | 563 | 0 | 6 |
| 111 | 628 | 24 | 591 | 13 | 0 | 19 | 572 | 0 | 6 |
| 112 | 647 | 24 | 614 | 9 | 0 | 28 | 586 | 0 | 3 |
| 113 | 620 | 23 | 588 | 9 | 0 | 18 | 570 | 0 | 8 |
| 114 | 642 | 31 | 605 | 6 | 0 | 39 | 566 | 0 | 11 |
| 115 | 651 | 25 | 619 | 7 | 0 | 42 | 577 | 0 | 15 |
| 116 | 645 | 19 | 620 | 6 | 0 | 26 | 594 | 0 | 9 |
| 117 | 662 | 16 | 636 | 10 | 0 | 15 | 621 | 0 | 3 |
| 118 | 658 | 32 | 622 | 4 | 0 | 27 | 595 | 0 | 10 |
| 119 | 659 | 24 | 629 | 6 | 0 | 24 | 605 | 0 | 11 |
| 120 | 674 | 29 | 630 | 15 | 0 | 25 | 605 | 0 | 7 |
| total | 12970 | 510 | 12285 | 175 | 0 | 578 | 11707 | 0 | 170 |

Tentative tracks, two_stage, hard scenario, seeds 101-120: 40655 measurements, 12357 clutter, 23 ambiguous.

| seed | measurements | clutter | ambiguous | cars | cars never confirmed | mean steps, first measurement to confirmation |
|---|---|---|---|---|---|---|
| 101 | 2039 | 616 | 3 | 5 | 0 | 2.000 |
| 102 | 2063 | 633 | 0 | 5 | 0 | 2.200 |
| 103 | 2012 | 615 | 0 | 5 | 0 | 1.800 |
| 104 | 2032 | 617 | 0 | 5 | 0 | 2.200 |
| 105 | 2081 | 653 | 0 | 5 | 0 | 2.000 |
| 106 | 2009 | 591 | 0 | 5 | 0 | 2.000 |
| 107 | 2060 | 643 | 2 | 5 | 0 | 2.000 |
| 108 | 1985 | 590 | 0 | 5 | 0 | 2.000 |
| 109 | 2021 | 590 | 0 | 5 | 0 | 2.000 |
| 110 | 2013 | 594 | 0 | 5 | 0 | 2.000 |
| 111 | 2019 | 610 | 4 | 5 | 0 | 2.000 |
| 112 | 2042 | 622 | 0 | 5 | 0 | 2.000 |
| 113 | 2029 | 603 | 0 | 5 | 0 | 2.000 |
| 114 | 2020 | 601 | 0 | 5 | 0 | 2.000 |
| 115 | 2017 | 605 | 14 | 5 | 0 | 2.000 |
| 116 | 2045 | 627 | 0 | 5 | 0 | 2.000 |
| 117 | 2043 | 652 | 0 | 5 | 0 | 2.200 |
| 118 | 2023 | 620 | 0 | 5 | 0 | 2.000 |
| 119 | 2042 | 632 | 0 | 5 | 0 | 2.200 |
| 120 | 2060 | 643 | 0 | 5 | 0 | 2.000 |
| total | 40655 | 12357 | 23 | 100 | 0 | 2.030 |

On seeds 101-120 the tracker started 12970 tentative tracks, confirmed 510, dropped 12285 and left
175 unresolved. Of the 12285 dropped, none was a real birth lost, 578 were births to a car that
already had a confirmed track, 11707 had a clutter birth label and none had an ambiguous one. 170
of the 12970 tracks were duplicate tentative tracks of one car. Per seed the count runs from 3
(seeds 112 and 117) to 15 (seed 115).

Each seed has 5 cars, and all 100 cars were confirmed, a mean of 2.030 steps after the car's first
labelled measurement. The per-seed mean runs from 1.800 on seed 103 to 2.200 on seeds 102, 104, 117
and 119.
