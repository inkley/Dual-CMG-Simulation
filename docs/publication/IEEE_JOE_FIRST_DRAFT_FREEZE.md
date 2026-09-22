# IEEE JOE first-draft scope freeze — v1, 2026-09-22

**Status: scope, retained case definitions, and criteria frozen; publication
outputs NOT yet regenerated or approved.** This supersedes the task list and
pending-status statements in `IEEE_JOE_SIMULATION_MATRIX.md`. It is a project
decision, not a statement of IEEE requirements or guaranteed publishability.

## 1. Claim and exclusions

Demonstrate modeled CMG-enabled roll reorientation and staged roll–turn–surge
operation with fixed body-plane sway/yaw thrusters. Compare configured single
and dual architectures, mechanical work/storage, rotor sizing, selected
uncertainty, and finite-duration synthetic roll-disturbance response.

Retain the existing damped allocator. Corrected hybrid missions explicitly use
bounded CMG pitch recovery after initial roll capture; do not describe them as
roll-only CMG control. Comparison roll tests retain their own documented control
settings. The +/-5% inertia-knowledge target is supported only by selected
disturbed-hold points, not a certified uncertainty box.

Deferred: sensor-based disturbance estimation/feedforward, general station
keeping, waypoint arrival/stopping, a new uncertainty-aware allocator, total
motor-input accounting/efficiency, detailed hardware validation, and fixed-time
versus capture-gating superiority tests. No new capability is required solely
to fill an old planning-matrix row. In particular, unrun fixed-time ablation H2
is removed from the first draft; do not claim gating superiority from it.

## 2. Retained publication cases

IDs below are stable first-draft identifiers. "Supporting" means retained in
tables/supplement/archive, not optional exclusion after a failure. Existing
scripts are entry points, NOT yet an isolated publication batch runner.

| IDs | Exact retained definition | Existing entry point / placement |
|---|---|---|
| R-A-S, R-A-D, R-B-S, R-B-D | Both matched-initial-energy pairs A/B, single/dual; 4-inch disks at 1200 rpm. A: single 0.5-inch / each dual 0.25-inch thickness; B: single 1-inch / each dual 0.5-inch. +90-degree roll, 5 s, unchanged gains and limits | `MATCHED_ENERGY_COMPARISON`; main architecture/work comparison. Keep A-single speed-stop failure and B-single cross-axis response. |
| G-1…G-5 | All five dual disk D×t cases in inches: 3.2×0.5, 4×0.5, 4.8×0.5, 4×0.25, 4×0.75; +90-degree roll, 5 s | `FLYWHEEL_GEOMETRY_SWEEP`; compact sizing table, supporting trajectories. Keep small-disk acceleration limiting. |
| H-P, H-N, H-0 | Plane/turn pairs (+45,+45), (-45,-45), (0,+30) degrees; pitch recovery ON; 24-s turn reference, 0.5 m/s surge, 30-s cruise observation | `RUN_ROLL_TURN_SURGE` with explicit `planePitchCorrection=true`; main mission plots. End in cruising. |
| U-E01…U-E13 | All 13 indexed `MISSION_ESTIMATION_CASES` on +45/+45 mission, pitch recovery ON | `PLANE_PITCH_CORRECTION_SWEEP`; main uncertainty summary/supporting histories. Case 1 duplicates H-P only if complete configurations agree. Preserve strict-screen failures. |
| U-L12, U-L13 | Same isolated unequal-inertia cases 12/13, pitch recovery OFF | `RUN_ROLL_TURN_SURGE` explicit case/flag; before/after diagnostic, retain timeouts. No blanket legacy-controller success claim. |
| U-M01…U-M09 | All nine installed-mass/diagonal-inertia cases and paired roll tests, using their original pitch-recovery-OFF mission configuration | `INSTALLED_MASS_UNCERTAINTY_SWEEP`; supporting sensitivity stratum, NOT a corrected-controller robustness claim. |
| U-A01…U-A09 | All nine original gimbal/VRT/aft actuator-uncertainty mission cases, pitch recovery OFF | `ACTUATOR_UNCERTAINTY_SWEEP`; supporting sensitivity stratum, same controller caveat. |
| D-Z, D-A01…D-A04, D-P01…D-P04 | Zero reference plus 0.01/0.03 Nm × 5/15 s, feedback ON/OFF pairs; 45-degree initialized hold; total 80 s, load 10–70 s, 2-s ramps, no unloading | `RUN_ROLL_DISTURBANCE_TESTS(true)`; main disturbance characterization. Retain three active tracking failures and all passive references. |
| D-E01…D-E05 | Nominal provisional envelope: 0.006 Nm, 200 s, load 10–190 s; periods/biases (5,-0.0005), (5,+0.0005), (15,-0.0005), (15,+0.0005), (10,0) | `VERIFY_ROLL_DISTURBANCE_ENVELOPE`; main/supporting operating limits. Bias remains throughout run. |
| M-Z, M-B | 0.03 Nm, 15 s, 180-s sinusoid in 200-s run; zero bias versus +0.003 Nm bias from t=0; existing 85-degree diagnostic stop on biased run | `EVALUATE_ROLL_DISTURBANCE_REJECTION`; main momentum limitation, retain stopped run. Refinement run is verification, not a third independent scenario. |
| D-U01…D-U12 | Six named configurations in `DISTURBANCE_MISMATCH_SWEEP`, both bias signs at 0.006 Nm/15 s/200 s | Main/supporting mismatch summary. Keep unequal-inertia negative-bias failure. |
| I-N01…I-N09, I-P01…I-P05 | True unequal-inertia plant fixed; per-rotor errors relative to truth. Negative bias: all nine {-5%,0,+5%} pairs. Positive bias: four +/-5% corners plus (0,0). Same 0.006 Nm/15 s/200 s load | `SWEEP_DISTURBANCE_INERTIA_ESTIMATES`; supporting calibration-target evidence; no continuous-box guarantee. |
| I-C075, I-C100 | Same fixed plant and negative bias with error pairs (-7.5%,+7.5%), (-10%,+10%) | Same sweep entry point; retain passing/failing outside-target probes. |
| I-EXACT | Same physical plant and load as failed D-U unequal-inertia negative-bias case, changing ONLY allocator inertia knowledge to truth | `DIAGNOSE_DISTURBANCE_INERTIA_FAILURE`; diagnostic, not a production fix. |

The matched pairs supply the main single/dual comparison. Historical unmatched
baseline figures and the old 36-case roll-only estimation sweep are background
development records, not additional required main-result batches. Existing
historical failures remain archived; they must not be presented as new-revision
results. Mechanical-work/storage analysis reuses retained roll trajectories:
it does not introduce additional simulations or efficiency claims.

## 3. Frozen criteria and interpretation

### Roll/geometry comparisons

Complete the 5-s horizon, absolute final roll error <=1.8 degrees, settling in
the 2% (1.8-degree) band by 4 s and staying in it through the recorded horizon,
and no recorded actuator-limit flags. Report overshoot, cross-axis motion,
actuator demand, terminal event, and partial-run metrics separately. A roll pass
does not mean comparable full attitude performance. Preserve any legacy
roll-only versus overall-screen distinction in the mass sensitivity results.

### Staged mission screen

Use the existing `RUN_ROLL_TURN_SURGE` predicate without alteration:
COMPLETE; maximum surge heading/plane errors <=1 degree each; maximum
cross-track/out-of-plane displacement <=0.05 m each; final speed error <=0.025
m/s; progress >=5 m; pre-surge aft force <1e-9 N; zero recorded limit flags;
zero sampled legacy pitch-neutral allocation-infeasibility flags.

Report phase-specific and whole-trajectory extrema, completion, and strict
screen outcome separately. A completed mission with a brief feasibility flag
remains a strict failure. Also report roll-only authority and moment residual
magnitude/duration to explain it, not to override the predicate. Capture dwell,
timeouts, controller gains, pitch-recovery bound, and abort logic remain those
of the current named configurations; they must be serialized in the batch.

### Disturbance hold screen

Complete the stated horizon; absolute peak roll error <=2 degrees over the
whole run; time-weighted load-window RMS <=1 degree; maximum final-second
absolute error <=0.5 degrees; zero recorded actuator-limit flags. For envelope,
mismatch, and inertia-knowledge studies retain the existing 80%-of-headroom
rotor-x excursion allowance as an additional planning check. Do not apply this
extra gate retroactively to D-A characterization outcomes. Passive responses
are references, not controller pass/fail evidence. Partial-window RMS from
stopped runs must be labelled partial and not ranked as full-horizon RMS.

Momentum stress cases M-Z/M-B are characterization/limitation experiments, not
required disturbance passes. The 85-degree diagnostic guard is distinct from
the +/-100-degree physical assumption. No sustained-bias unloading capability
is credited to the center-plane thrusters.

### Energy, geometry, and numerical verification

Report supported net/gross vehicle work, rotor spin storage and signed changes,
and explicitly labelled partial actuator-work terms. No total motor-input or
electrical efficiency ratios; no summing overlapping transfers as consumption.
Use `VERIFY_CMG_MECHANICAL_ENERGY`, `VERIFY_MATCHED_ENERGY`, and
`VERIFY_FLYWHEEL_GEOMETRY` for their existing numerical checks. Energy equality
does not eliminate different controllers, mounting, motor count, terminal
storage, or cross-axis performance. Tube diameter is a rotor-only screen, not
assembled hardware fit. Retain frozen structural-inertia and disabled
hydrostatic-restoring assumptions explicitly.

Before publication, run the existing torque-sign, momentum, mass-assembly,
actuator, thruster-allocation, supervisor, and pitch-correction verification
checks. Repeat R-A/R-B and G cases at existing half-step settings; H-P at
supervisor/ODE steps 0.025/0.005 s; U-E06/09/10/12/13 at those finer settings;
the worst original disturbance, diagnosed unequal-inertia failure, and
I-N(+5,-5)/I-C100 at half maximum ODE step as already implemented.
Keep known resolution-sensitive strict failures visible. Existing verifier
assertions are frozen with the source revision; numerical agreement is not
physical validation.

## 4. Completion and change control

This freeze does NOT certify archived numbers for the draft. After targeted
cleanup, regenerate retained results from one identified clean source revision.
Archive each unique case ID, complete configuration, true/estimated parameters,
source/baseline hashes, MATLAB version, solver and sampling settings, events,
metrics, pass/fail reasons, and figure-source links in a unique run directory.
Do not overwrite the only evidence bundle or silently deduplicate differing
controller configurations. Identical cases may share outputs only with a
recorded configuration-hash match. A publication runner/manifest remains the
next implementation task, not a capability supplied by this document.

Publication batch success means all retained cases and verifiers have an
accounted-for outcome, not that every scenario passes. Unexpected changes
require investigation before numbers enter the draft. Failures may be reported
as limitations; never delete them or loosen thresholds to complete the batch.
If a scope/configuration/criterion change becomes necessary, record a new freeze
version, reason, and affected cases, then rerun those cases. Do not silently
update the v1 definitions.

Suggested presentation: architecture/control diagram; matched single/dual roll
and cross-axis response; rotor/work table; three staged mission trajectories;
compact uncertainty outcomes; disturbance tracking and finite momentum storage.
Detailed grids and verification traces belong in the supporting archive.
