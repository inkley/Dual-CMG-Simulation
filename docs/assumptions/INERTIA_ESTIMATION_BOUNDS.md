# Rotor-inertia estimation error during disturbed operation

## Provisional development requirement

Target **absolute per-rotor relative inertia-estimation error <=5%** for the
fixed unequal-inertia plant and disturbed-hold family tested here. Treat this
as a conservative calibration/model-knowledge target supported by discrete
screening, not a certified continuous uncertainty bound or the largest allowed
error. No production estimate is automatically replaced by truth.

All nine {-5%,0,+5%} error pairs with negative bias, and the four +/-5% corners
plus exact knowledge with positive bias, passed the unchanged roll/momentum
criteria: **14/14 selected cases**. Worst peak error was 1.514506 degrees and
worst RMS error was 0.835458 degrees. Positive-bias edge midpoints were not
tested. Intermediate error pairs, arbitrary phases, other periods, speed-sensor
bias, concurrent actuator uncertainty, and disturbed hybrid maneuvers are not
covered by this calibration target.

The prior -9.09%/+11.11% estimation case remains a documented failure at
2.039360 degrees peak error. Exact knowledge is not necessary for every tested
case, but using nominal rotor inertia without bounding estimation error is not
supported. Passing selected larger-error points would not invalidate that
failure or establish monotonic performance as error grows.

## Completed results (2026-09-22)

| Selected error pair, relative to truth | Bias (Nm) | Peak error (deg) | Result |
|---|---|---|---|
| +5% / -5% (worst peak within the selected +/-5% set) | -0.0005 | 1.514506 | Pass |
| +5% / +5% (worst RMS within that set: 0.835458 deg) | -0.0005 | 1.308532 | Pass |
| -7.5% / +7.5% | -0.0005 | 1.622562 | Pass |
| -10% / +10% | -0.0005 | 2.009986 | Fail: peak error |

All runs completed 200 seconds with no recorded actuator-limit flags. Within
the selected +/-5% set, maximum final-second error was 0.10679 degrees,
maximum rotor-x excursion was 0.11986 N m s (allowance 0.152651), maximum gimbal
angle was 48.93 degrees, and maximum pitch/yaw excursions were 0.06598/0.05806
degrees. Cross-axis excursions are reported, not assigned a new pass criterion.

Half-step repeats of the worst-peak +/-5% case and the -10%/+10% failure retained
their classifications. Maximum common-grid roll changes were 6.77e-11 and
1.34e-9 degrees, respectively. The 18 saved runs comprise 14 distinct target-set
points, two outside-target probes, and two resolution repeats; they are not
18 independent uncertainty cases. Sixteen runs pass and the two runs at the
same -10%/+10% point fail.

The 7.5% pass and 10% failure are observations along **one opposite-error
direction**, not a proof of a single monotonic threshold between them. The
provisional +/-5% target deliberately avoids claiming that threshold.

### Development disposition

The estimation-error bounding study is complete for this fixed plant and
15-second disturbed-hold family. Retain +/-5% as a provisional model-knowledge
requirement, with its discrete-screening caveat. Do not claim the entire nominal
disturbance envelope is now mismatch-robust. Before relying on the requirement
in broader missions, test additional frequencies/phases and combined sensing or
actuator errors, or evaluate uncertainty-aware allocation near poorly conditioned
steering. No hardware calibration method or estimator has been implemented.

## Definition and fixed experiment

Per-rotor error is (estimated inertia / true inertia) - 1. The physical plant
is held at the previously studied +10%/-10% rotor inertias relative to nominal;
installed vehicle inertia and mass are unchanged. Rotor speeds, gimbal dynamics,
controller gains, allocator damping, and pitch correction remain fixed. Speed
and angle sensing are ideal. Only allocator inertia estimates are varied.

The previous nominal-estimate failure corresponds to **-9.09%/+11.11% errors
relative to truth**, not symmetric +/-10% estimation error. Exact inertia
knowledge passed the same true plant. Neither result establishes a universal
calibration threshold, especially near sensitive steering configurations.

The initial screen uses all nine combinations of -5%, 0%, and +5% per rotor,
under the previously failing negative-bias disturbance. This distinguishes
common, differential, and one-rotor-only estimation errors. Additional signed
bias checks and boundary points are recorded separately as they are tested.

Each run starts at the same initialized 45-degree hold and lasts 200 seconds.
The sinusoid is 0.006 Nm, period 15 s, zero onset phase, active from 10 to 190 s
with 2-second ramps. Constant bias acts throughout the run. Criteria remain
peak roll error <=2 degrees, load-window RMS <=1 degree, final-second error
<=0.5 degrees, complete horizon, no recorded actuator-limit flags, and rotor-x
momentum excursion within the original reserve allowance. The same 85-degree
diagnostic gimbal guard and physical speed guard apply.

This is deterministic finite-duration screening of one plant and load family,
not a proof for every intermediate error pair, frequency, phase, initial
momentum, or hybrid mission. The scalar aggregate momentum budget remains a
planning check rather than proof of full coupled allocation feasibility.

## Reproduction

```matlab
SWEEP_DISTURBANCE_INERTIA_ESTIMATES; % negative-bias 3x3 grid, errors relative to truth
% Explicit additional pairs, bias, maximum solver step:
% SWEEP_DISTURBANCE_INERTIA_ESTIMATES([-.05,.05],.0005,.01);
REPORT_INERTIA_ESTIMATION_BOUNDS;
```

Each point has an individual MAT file containing the true plant, controller
configuration, trajectories, and criteria-derived summary under
`Working Results/inertia_estimation_bounds`. The report checks estimate/truth
consistency and applied loads, and aggregates saved points without discarding
failures. Production estimates and controller settings are not changed.
