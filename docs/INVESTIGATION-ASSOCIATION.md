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
car. The study skipped the 314 ambiguous measurements out of 28590. 175 of the 230 steals were
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
