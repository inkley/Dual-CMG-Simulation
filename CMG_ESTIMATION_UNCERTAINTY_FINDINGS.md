# Flywheel-speed bias and rotor-inertia uncertainty

## Scope and reproducibility

Run `VERIFY_CMG_ESTIMATION`, `CMG_ESTIMATION_UNCERTAINTY_SWEEP`,
`CMG_ESTIMATION_UNCERTAINTY_SWEEP(.005)`, then
`VERIFY_CMG_UNCERTAINTY_RESULTS` from the Simulation directory.
Results, full histories, configuration snapshots and a summary figure are saved
under `Working Results/estimation_uncertainty/` (ignored by Git).
The sweep loads the existing single VFR and symmetric dual VFR MAT baselines;
regenerating those baselines with different parameters changes these results.

The experiment is a five-second, from-rest 90-degree roll with fixed gains,
not a roll-turn-surge robustness demonstration. No thruster/propulsion assistance
is enabled. Nine single and 27 dual cases were evaluated (including repeated
nominal points across dual pattern groups). These are deterministic scenario
counts, not statistical success probabilities.

## Truth versus estimate

`CONTROL` now optionally supplies an estimated state and rotor inertia to
`CMG_ALLOCATE`, while `CMG` and vehicle dynamics retain the physical state and
true rotor inertias. Without `config.estimation`, behavior is unchanged.

Measured speed = true speed * (1 + scale bias) + additive offset.
The sweep uses scale biases -5%, 0, +5%, with zero additive offset.
Positive scale bias overestimates the magnitude of either signed spin speed.
True rotor spin-axis inertia is nominal * (1 + error), with errors -10%, 0,
+10%; the controller retains nominal inertia. Dual patterns are equal errors
on both rotors, opposite errors, and error on rotor 2 only. Scale bias and
inertia-error signs vary independently within each pattern, but this is not
the exhaustive four-independent-parameter uncertainty box.

These are exploratory stress ranges, NOT validated sensor or manufacturing
tolerances. Installed vehicle mass/inertia is held fixed: this isolates the
rotor angular-momentum model, not the effects of a physically resized rotor.
Gimbal angles and body-state measurements are ideal. Actual spin state is not
perturbed by measurement bias. Constant-speed dual operation does not model a
biased flywheel speed servo. Additive offsets are supported and unit-tested,
but their trajectories, noise, delay and bias drift were not swept.

## Results

Development roll screening required a full five-second trajectory, final roll
error <=1.8 degrees, and 2%-band settling by 4 seconds. This is a declared
development screen, not a hardware or publication acceptance standard.
Actuator limits and cross-axis behavior are reported separately.

| Metric | Single (9 cases) | Dual (27 cases) |
|---|---:|---:|
| Roll-screen passes | 6 | 23 |
| Worst 2%-band settling time | 4.690 s | 4.820 s |
| Maximum absolute final roll error | 1.387 deg | 1.517 deg |
| Maximum absolute pitch | 1.189 deg | 0.863 deg |
| Maximum absolute yaw | 0.410 deg | 0.430 deg |
| Maximum absolute gimbal angle | 81.776 deg | 23.764 deg |
| Maximum actual rotor speed | 1575.92 rpm | 1200 rpm |

All cases completed and met the final roll-error criterion. Seven missed the
four-second settling target; do not describe the entire sweep as passing.
The single cases retain brief gimbal-acceleration saturation (two recorded
samples each at 0.01-second output spacing). Dual cases have no recorded
actuator-limit flags. No actual angle or flywheel-speed stop event occurred.

Equal dual errors preserve symmetry but alter roll-loop response. Differential
true inertia errors introduce cross-axis reactions; cancellation is conditional
on symmetry, not an unconditional dual-CMG property. Slower settling is the
principal roll-screen failure here, not momentum exhaustion or a hard stop.

Across all cases, the worst time-weighted physical unconstrained roll-inverse
RMSE is 0.016169 N m. This includes estimation mismatch and allocator damping,
but excludes finite-gimbal response loss. `control.allocationError` is evaluated
against physical truth, not solely the estimated allocation equation. Do not
interpret every nonzero moment error as actuator saturation.

## Verification and boundaries

Halving MaxStep from 0.01 to 0.005 s preserved pass/fail outcomes and changed
reported attitude/gimbal metrics by less than 2e-11 degrees. Tight adaptive
tolerances can already enforce smaller steps; this does not establish model
accuracy. Zero-error estimation reproduces the original CONTROL derivative
exactly in the unit check. Plant-truth isolation and signed bias/offset behavior
are also tested.

Physical angle/speed terminal events prevent interpreting post-stop trajectories.
Allocator speed-limit logic uses measured speed, so it is not an independent
true-speed hardware protection model. The existing model does not introduce
extra gimbal structural inertia, rotor transverse inertia, motor torque-speed
limits, or a full mechanical redesign in this uncertainty test.

## Trello handoff

Complete: Evaluate flywheel-speed measurement bias and rotor-inertia uncertainty
for the single/dual 90-degree roll baseline.

Retain: Apply representative uncertainty corners to the complete oblique
roll-turn-surge mission, including activation delays and cross-axis drift.
Baseline roll robustness alone does not validate that mission.
