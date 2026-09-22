# Nominal versus exact rotor-inertia knowledge on the same unequal plant

## Controlled comparison

Cases 12 and 13 use opposite +/-10% true axial rotor-inertia errors, unbiased
speed sensing and the existing +45/+45 roll-turn-surge mission. Previously,
the allocator assumed nominal rotor inertia. This diagnostic gives it the
true unequal inertias. All physical parameters, initial states, gains,
supervisor tolerances, timeouts and allocator regularization remain unchanged.
The comparison asserts that the saved plant baseline is identical and only
the two estimated inertias in the controller configuration change.

This isolates whether exact parameter knowledge is sufficient for recovery;
it is not an online identification algorithm or a revised control law.
Installed rigid-body/gimbal inertia remains nominal as in the original
momentum-model uncertainty experiment. No physically resized rotor or new
mass distribution is introduced.

## Reproduction

```matlab
EXACT_INERTIA_KNOWLEDGE_COMPARISON
for k=[12,13]
    RUN_ROLL_TURN_SURGE(struct('estimationCase',k, ...
        'exactRotorInertiaKnowledge',true,'sampleTime',.025,'maxStep',.005))
end
VERIFY_EXACT_INERTIA_KNOWLEDGE
```

The original nominal-knowledge histories must exist. Call
`EXACT_INERTIA_KNOWLEDGE_COMPARISON(false)` to rebuild the summary without
rerunning trajectories. Exact-knowledge outputs are isolated in `exact_inertia`
subfolders of their original mission-case folders. Summaries are in
`Working Results/exact_inertia_knowledge`. Baseline operation remains unchanged
unless the new boolean `exactRotorInertiaKnowledge` option is explicitly set.

Moment identity checks compare estimated and true CMG moments using the same
actual actuator rates. Exact agreement tests model knowledge, not requested
torque tracking through finite servo dynamics or successful plane control.

## Results

All four nominal/exact-knowledge comparisons abort on turn-capture timeout;
none enter SURGE or produce aft thrust. At the last TURN sample:

| True inertia errors | Allocator knowledge | Heading error (deg) | Plane error (deg) |
|---|---|---:|---:|
| (-10%,+10%) | Nominal | 1.3341 | 1.8629 |
| (-10%,+10%) | Exact | 1.4641 | 2.0451 |
| (+10%,-10%) | Nominal | 1.0056 | 1.4212 |
| (+10%,-10%) | Exact | 1.1343 | 1.6014 |

Plane errors remain well above the 0.5-degree capture requirement. Rate norms
are below the 0.5-degree/s threshold. Exact knowledge does not improve capture
in these cases; modestly worse final errors do not imply that inaccurate
parameters should be preferred or deliberately introduced.

Sampled predicted/physical CMG moment agreement with exact knowledge closes
within 1.73e-18 N m. Thus an incorrect inertia parameter in the torque mapping
is not necessary for this failure. Exact mapping still does not make the
underactuated system capable of arbitrary independent roll/pitch/yaw moments.
Last-TURN CMG pitch moments are approximately -2e-9 N m, while yaw moments
are nonzero (7.29e-5 and -8.88e-6 N m). These endpoint samples alone do not
establish the complete causal mechanism or the integrated coupling effect.

Both exact cases abort around 55.30 s at the original step, and 55.275 s at
the refined step. Finer-step final heading metric changes are 0.000759 and
0.000282 degrees. There are no recorded saturation flags. Original-step exact
cases have zero roll-infeasibility samples; both refined cases have one.
That sampling-sensitive diagnostic remains open; stable capture failure is
not explained solely by it. Final post-abort errors are not substituted for
the pre-abort capture metrics in the table.

Plant/configuration identity checks, exact moment identity, safe propulsion
inhibit, refinement and supervisor unit checks passed. This means diagnostic
verification passed, NOT that the maneuver performance screen passed.

## Interpretation / Trello

Complete: compare nominal versus exact inertia knowledge for the same
unequal-inertia plant. Exact parameter knowledge alone does not recover capture.

Keep open: resolve turn-capture failures. Next inspect the full plane-normal
error dynamics, residual CMG cross-axis moments and thruster activation/yaw
braking through the turn. Distinguish physical momentum asymmetry, moving
roll-reference tracking and gating before implementing compensation. Do not
loosen capture tolerances or add inertia identification as if it were an
established remedy. Separately resolve the step-sensitive roll-feasibility
diagnostic. No production controller or baseline was retuned in this study.
