# Rotor-estimation uncertainty during roll–turn–surge

## Scope

Nine scenarios run the existing +45-degree plane/+45-degree turn and 0.5-m/s
surge mission: nominal plus eight corners. Two patterns (common errors on
both rotors, opposite errors) each combine speed scale bias +/-5% with true
axial rotor-inertia error +/-10%. Within each pattern the bias and inertia
signs vary independently. This is not the full four-independent-parameter box.

The allocator sees Omega_measured=Omega_true*(1+bias) and nominal rotor
inertias. Actual CMG torque uses true speed and perturbed axial inertia.
Installed rigid-body mass/inertia and gimbal assembly inertia remain nominal:
this isolates spin-momentum model uncertainty, not a physically resized rotor.
Positive speed-scale bias overestimates either signed speed's magnitude.
The constant-spin plant remains at -1200/+1200 rpm, so this does not test a
biased spin-speed servo. Other sensing is ideal. Gains, actuator bounds,
supervisor capture criteria and screening thresholds are unchanged.

This extends the earlier roll-only test, without asserting hardware tolerances,
statistical reliability, all-direction maneuver robustness, or combined
rotor/installed-mass/actuator uncertainty. No additive bias, noise or delay
is included in this trajectory sweep. COMPLETE means cruising, not stopping.

## Reproduce

Run `VERIFY_CMG_ESTIMATION`, then `MISSION_ESTIMATION_SWEEP`.
Summary CSV/MAT files are in `Working Results/mission_estimation`; detailed
mission MAT histories, events and figures use `estimation_<case>` subfolders
of the roll_turn_surge results folder. Nominal baseline inputs are unchanged.
`MISSION_ESTIMATION_CASES` gives explicit parameter vectors. Case 0 in the
mission defaults preserves prior behavior; case 1 is the nominal study control.

The physical feasibility diagnostic uses true rotor properties, not estimated
allocation matrices. Pass/fail retains the existing nominal mission screen;
COMPLETION is distinguished from meeting all tracking/limit criteria. Report
failed cases and timeouts as outcomes, not missing data.

For representative opposite-error refinements run cases 6 and 9 at
`sampleTime=.025` and `maxStep=.005` with `RUN_ROLL_TURN_SURGE`, then run
`VERIFY_MISSION_ESTIMATION_RESULTS`. These verify plant/estimate separation,
actual constant spin, capture-failure outcomes and absence of premature surge.

## Results

Five of nine scenarios passed the unchanged mission screen: nominal plus all
four common-error corners. All four opposite-error corners timed out during
turn capture and aborted without ever commanding/producing aft thrust.
These are nine deterministic scenarios, not a statistical success rate.

Among completed cases, final heading error was at most 0.4412 degrees,
maximum surge-plane error 0.6114 degrees, maximum out-of-plane displacement
0.01729 m, and maximum cross-track displacement 0.000286 m. Surge began
between 37.10 and 39.20 s (38.10 s nominal). Final speed was approximately
0.49959 m/s with approximately 12.50 m of progress.

Opposite-error cases reached turn-capture timeout around 55.2--55.35 s.
No recorded actuator-limit flags occurred in any case. Cases 6 and 9 each
had one sampled physical pitch-neutral roll-infeasibility flag; cases 7 and
8 had none. Thus ordinary saturation or sustained loss of roll authority
does not explain all failures. Unequal estimated-versus-actual moment
mapping introduces cross-axis errors that the current coordinated controller
does not consistently resolve within its capture requirements. The sweep
combines speed bias and inertia error; it does not isolate their individual
causal contributions to the failed turn.

The final heading metric after ABORT is not a completed-mission result;
interpret the saved TURN history and capture thresholds instead. The supervisor
changes its hold reference on abort. Do not compare partial/aborted mission
work or terminal errors as equivalent to successful cruise segments.

At the last TURN sample, cases 6--9 had plane errors 1.9273, 1.3686, 1.8023
and 1.4819 degrees, respectively, all above the 0.5-degree capture threshold.
Heading errors were 1.3800, 0.9679, 1.2909 and 1.0490 degrees. Angular-rate
norms were already below the 0.5-degree/s requirement (maximum 0.0602).
Residual plane misalignment therefore prevents capture even in case 7,
whose heading met its 1-degree threshold. Cases 6 and 9 reproduced their
ABORT outcomes under finer steps, with final heading metric changes below
0.0012 degree. Verification checks passed; this does not turn failed
performance screens into successful maneuvers.

## Trello / next development action

Complete: evaluate flywheel-speed measurement bias and rotor-inertia
uncertainty during the specified roll-turn-surge mission. Evaluation is
complete, but robustness to opposite rotor errors has NOT been established.

Add: diagnose opposite-rotor estimation-error turn-capture failures, separating
speed bias from inertia mismatch; assess cross-axis compensation or estimation
improvements before considering changed capture tolerances. Do not simply
relax the acceptance criteria or extend timeouts to mark the cases passing.
