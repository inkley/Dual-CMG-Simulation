# IEEE JOE simulation and comparison matrix — study plan v1

This defines the intended paper scope and evidence plan, not IEEE submission
requirements or a statement that all tests have been completed. Freeze final
parameter sets and screening criteria before generating the publication batch.
Existing tests are development evidence; rerun selected cases from one code
revision for the manuscript. Do not silently discard failed cases.

## Central claim and scope

Within the stated six-DOF vehicle and finite-actuator model, CMG roll control
can reorient the plane of fixed lateral thrusters and enable a staged
roll–turn–surge maneuver from rest. Compare single and dual CMGs in tracking,
cross-axis reaction, actuator demand and mechanical-energy accounting.
Do not claim global controllability, optimal control, demonstrated electrical
efficiency, hardware validation or universal robustness.

Core: model verification; single/dual roll; rotor sizing and energy tradeoffs;
nominal hybrid mission; bounded uncertainty; bounded roll-disturbance tests.
Optional: sensor-derived feedforward, full station keeping, waypoint stopping,
hardware design and faster-turn optimization. These are not prerequisites for
finishing this scoped simulation study. Sufficiency for publication remains a
research/advisor judgment, not a guarantee of acceptance.

## Main comparison matrix

| ID | Question / cases | Controlled comparison and metrics | Evidence status / remaining action |
|---|---|---|---|
| V1 | Is the model internally consistent? | Torque sign and vector identity; allocation residual; angular-momentum balance with external loads; installed-mass assembly; finite-actuator and supervisor unit tests; finer-step checks | Existing checks available. Rerun at frozen revision; extend any energy-balance claims only after the accounting audit. |
| R1 | What changes between single and dual roll actuation? | +90-degree rest-to-rest request, 5-s observation, same PD gains/initial vehicle state and stated per-module properties; retain installed mass differences. Report 2% settling, overshoot, final error, pitch/yaw, moment RMSE, angle/rate/acceleration limits, spin speed and rotor momentum | Baselines exist. Regenerate together; single uses variable spin while preferred dual uses constant spin, so this compares complete configured architectures, not rotor count alone. |
| R2 | Is the dual advantage due to extra hardware? | Add one matched-total-initial-spin-energy single/dual comparison at common initial speed: sum of dual spin inertias equals single inertia. Recompute rotor mass/tensors consistently using the chosen geometry. Keep mission, gains, bounds and accounting horizon fixed; report resulting mass and demands | Pending rotor-geometry study. Equal stored energy is not equal total mass, electrical power, or universal fairness; show actual as-installed R1 alongside this normalization. |
| G1 | What is the rotor-sizing tradeoff? | Nominal and two bracketing physically consistent geometries, initially 0.8/1.0/1.2 times nominal radius at fixed thickness/density/spin. Recompute mass, axial/transverse inertias and installed properties. Use dual +90-degree roll with unchanged gains/limits | Pending. This is a three-point sensitivity study, not optimization. Include changed stored energy and actuator torque demand; assumed acceleration limits do not imply motors can realize every geometry. |
| E1 | What mechanical energy is required? | Reuse R1/R2/G1 histories; same boundaries and horizon. Separate net/gross vehicle roll work, signed/positive/negative actuator mechanical work where supported, initial/final rotor energy and spin-up bookkeeping | Accounting audit pending. Current ideal input proxy is not independent motor input. Do not label its near-100% ratio electrical efficiency or sum overlapping energy transfers as independent consumption. |
| H1 | Does the staged mission work in different planes/directions? | Existing (plane, turn) pairs (+45,+45), (-45,-45), (0,+30) degrees; same 24-s turn shape and 0.5-m/s surge command. Report stage times, full heading-vector error, plane error, cross/out-of-plane position, speed error/overshoot, forces and limits | Three development cases passed. Regenerate as main mission evidence. End state is cruising, not stopped at a waypoint. |
| H2 | Why use capture-based sequencing and shaping? | One +45/+45 mission with nominal supervisor versus the same shaped reference and predetermined phase transitions at 5.25 and 38.10 s. Repeat both at the existing high-mass/high-inertia corner; no gain retuning. Separately retain the earlier step-versus-shaped oblique turn as a diagnostic | Fixed-time ablation pending; it may pass or fail. Report outcomes without assuming gating superiority. Existing step-turn failures concern tracking/gating, not demonstrated momentum exhaustion. |
| U1 | How sensitive is the result to imperfect parameters? | Existing rotor-estimation roll sweep (36 cases), installed-mass mission study (9), actuator-response mission study (9). Fixed gains; preserve failures. Extend estimator error to +45/+45 mission using nominal plus eight corners: common/opposite rotor pattern × speed bias +/-5% × true spin inertia +/-10% | Existing studies available; full-mission estimator extension pending. These are scenario samples, not an exhaustive uncertainty box or success probabilities. Do not combine all uncertainties and claim worst-case coverage. |
| D1 | Can the CMGs reject bounded roll disturbances? | Dual roll hold with sinusoidal external K only: zero-load reference plus amplitudes 0.01/0.03 N m at periods 5/15 s; 60-s horizon. Compare active control against feedback-disabled response for each of four nonzero loads, with identical initial physical states and no external unloading | Proposed nine runs, pending implementation. Loads are synthetic generalized moments, not calibrated sea states. Report RMS/peak error, torque tracking, momentum excursion, limits and first loss of authority; stop before invalid post-stop dynamics. |
| M1 | What limits sustained operation? | Retain reciprocal/same-direction roll and sustained-bias diagnostics; monitor momentum envelope and time to authority loss | Existing studies available, but not indefinite-operation proof. Hypothetical external-roll unloading belongs in a separate limitation/extension panel, never as an installed fixed-thruster capability. |

R2 and G1 geometry changes must pass the mass-assembly and inertia-consistency
checks before running. If the current diagonal-inertia approximation cannot
represent a case, record it as outside the model rather than dropping tensor
terms. Synthetic disturbance amplitudes are proposed engineering test loads,
not literature-derived wave predictions or hardware requirements.

## Fixed reporting rules

- Report whole-trajectory maxima and stage-specific values separately. Do not
  exclude intervals with low thrust activation to improve apparent tracking.
- Retain existing roll screen: final error <=1.8 degrees and 2%-band settling
  by 4 s in a 5-s window. Slower but completed trajectories remain reportable.
- Retain existing H1 mission screen: final speed error <=0.025 m/s, progress
  >=5 m, surge heading/plane errors <=1 degree, cross/out-of-plane displacement
  <=0.05 m, no premature aft thrust, sampled roll infeasibility or limit flags.
  Verify the exact script definition and report final as well as transient
  errors; supervisor capture tolerances are not whole-trajectory guarantees.
- D1 is a characterization study, not a retrospectively tuned pass/fail test.
  Publish performance versus amplitude/period and any failures. A capacity
  boundary is a useful result; do not enlarge limits after observing failure.
- Record commanded and physical moments separately. Estimation error, finite
  actuator tracking loss, saturation and rank deficiency are distinct effects.
- Report rotor Hx and task-direction roll feasibility alongside full allocation
  conditioning. A full roll/pitch singularity need not prevent roll-only action.
- Mass/inertia screening omits buoyancy/trim and CG shifts. Thruster models are
  cycle-averaged; shaft-reaction torque, vortex dynamics and detailed motor
  losses remain outside scope and must be explicit manuscript limitations.

## Figure and table plan

1. Architecture/frames/actuator placement and staged-control diagram.
2. Single versus dual roll response, cross-axis motion and actuator demands (R1).
3. Rotor-size/normalization tradeoff and clearly labelled mechanical work (R2/G1/E1).
4. Three mission paths plus phase-marked heading/plane/speed histories (H1).
5. Compact uncertainty/ablation results including timing failures (H2/U1).
6. Disturbance tracking and momentum-envelope limitation (D1/M1).

Tables: model/actuator assumptions; comparison definitions; compact outcome
matrix. Move detailed sweeps and verification traces to supplementary material
or repository artifacts. Figure count is a planning target, not a journal rule.

## Publication batch and completion gate

Use one recorded Git revision with a clean working tree, MATLAB version,
scenario ID, baseline snapshot, solver tolerances/steps, controller settings,
actual versus estimated parameters, event log and metric definitions. Existing
saved baselines are inputs, not immutable provenance: regenerate/archive them
with the final batch. Use a unique run directory to avoid current-script
same-name overwrites. Keep generated results outside source control but retain
an archived results bundle and manifest. A batch runner/manifest is still to
be implemented, not supplied by this plan.

Repeat representative nominal, largest-error and failure-boundary cases at
finer integration/output or supervisor steps. Report sensitivity rather than
treating numerical agreement as physical validation. Complete pending matrix
rows, reconcile manuscript numbers to archived outputs, and keep failed or
outside-model cases visible before calling the evidence set final.

## Next bounded tasks

1. Audit mechanical energy accounting and choose physically consistent rotor
   geometries for R2/G1; regenerate fair single/dual comparisons.
2. Complete full-mission estimator uncertainty and the fixed-time ablation.
3. Implement the bounded synthetic disturbance matrix and finalize the frozen
   publication batch/manifest. Revisit optional sensor/station-keeping scope
   with the advisor only after the core results are assembled.
