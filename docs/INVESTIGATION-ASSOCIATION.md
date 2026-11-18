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
