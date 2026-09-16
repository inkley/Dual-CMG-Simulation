# Gimbal and thruster actuator-parameter uncertainty

## Experiment

Nine full +45-degree plane/+45-degree heading roll-turn-surge simulations use
unchanged gains, geometry, installed mass, rotor momentum and capture criteria.
These are deterministic exploratory cases, not measured hardware tolerances,
an exhaustive uncertainty grid, or a probabilistic reliability estimate.

| Case | Changed physical response relative to nominal |
|---|---|
| nominal | All multipliers 1 |
| gimbal_fast | Both rate-servo time constants x0.8 |
| gimbal_slow | Both time constants x1.2; acceleration bound x0.8 |
| gimbal_unequal | Rate-servo time constants x[0.8,1.2] |
| vrt_slow | Both force time constants x1.2; slew bound x0.8 |
| vrt_low_gain | Both lateral-thruster force gains x0.9 |
| vrt_unequal | Force time constants x[0.8,1.2]; gains x[0.9,1.1] |
| aft_slow | Aft time constant x1.2; slew bound x0.8 |
| combined | Unequal gimbal lags, acceleration x0.8, unequal VRT lags/gains, VRT slew x0.8, aft lag x1.2/slew x0.8 |

`ACTUATOR_PLANT_CONFIG` applies perturbations after nominal allocation.
VRT force gain multiplies the nominal force request before the physical force
cap and finite response, without changing allocation geometry or controller
knowledge. Scalar VRT lag retains its prior behavior; column-vector lags
enable unequal module response. Actual force states still generate vehicle
loads at the unchanged mount locations. Gimbal acceleration uncertainty also
changes the actuator's stopping-distance envelope. No sensor bias is combined
with this study, and no controller gains are retuned.

The mission's limit reporting now additionally includes physical lateral
thruster command clipping and force-slew saturation; allocation saturation
alone would miss these plant-side effects. Flags are sampled diagnostics,
not a guarantee against arbitrarily brief between-sample limit activity.

## Scope boundaries

This is the existing low-order actuator model. It does not add motor
torque-speed or thermal physics, backlash, deadband, transport delay, vortex
pulse physics, thrust-direction errors or propeller shaft-reaction torque.
Gimbal angle/rate and maximum thrust bounds remain nominal; the selected
acceleration/slew bounds and response parameters are perturbed. Results apply
to the tested finite positive oblique mission, not arbitrary sustained operation
or all heading directions. Completion is cruising, not vehicle stopping.

## Reproduce

Run `VERIFY_ACTUATOR_UNCERTAINTY`, then `ACTUATOR_UNCERTAINTY_SWEEP`.
Summary CSV/MAT files are in `Working Results/actuator_uncertainty`.
Detailed mission histories and figures are saved in `actuator_<case>`
subfolders of the existing mission output directory, preserving baseline
results. Case definitions are in `ACTUATOR_UNCERTAINTY_CASES.m`.

For the combined-case numerical refinement, run:

```matlab
RUN_ROLL_TURN_SURGE(struct('actuatorCase',9,'sampleTime',.025,'maxStep',.005))
VERIFY_ACTUATOR_UNCERTAINTY_RESULTS
```

## Results

All nine cases completed and passed the existing mission screen without
retuning. There were no recorded actuator-limit flags or sampled roll
infeasibility. Maximum final heading error was 0.4630 degrees; maximum
surge-plane error was 0.6418 degrees; maximum cross-track displacement was
0.000611 m; maximum out-of-plane displacement was 0.01902 m. The common
10% lateral-force reduction produced the largest heading/plane errors and
delayed surge activation to 39.35 s (38.10 s nominal).

Final surge speed remained 0.49956--0.49959 m/s. Slower aft response increased
peak speed from 0.5314 to 0.5382 m/s and peak thrust from 3.5220 to 3.5769 N.
Maximum gimbal angle across the nine cases was 62.949 degrees.

The combined case also passed with supervisor sample time and maximum ODE
step halved. Final heading and peak surge-plane errors changed by less than
0.001 degree; surge activation moved from 38.40 to 38.375 s. This is a
representative numerical sensitivity check, not exhaustive convergence proof.

The combined perturbation did not produce the worst tracking errors: some
effects offset each other in this particular trajectory. Do not infer that
the combined case bounds every parameter combination. Reduced acceleration
and slew bounds remained inactive in these recorded histories, so the result
does not characterize maneuver behavior under actuator saturation.

## Trello handoff

Complete: Evaluate gimbal and thruster actuator-parameter uncertainty for the
finite +45/+45 roll-turn-surge demonstrator. Nine nominal/perturbed cases
passed without gain retuning. Retain the stated low-order actuator and
finite-scenario scope rather than claiming general hardware robustness.
