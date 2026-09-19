# Separate speed bias from rotor-inertia mismatch

## Controlled experiment

The existing opposite-error corners are supplemented by four runs: speed bias
[-5,+5]% or [+5,-5]% with nominal true inertia, and true inertia error
[-10,+10]% or [+10,-10]% with unbiased speed sensing. Together with nominal
and the four earlier combinations this completes a 3x3 factorial grid for
the *opposite-error pattern*, not independent arbitrary errors on each rotor.
All use the same +45/+45 roll-turn-surge maneuver, controller gains and capture
criteria. Cases 1--9 retain their original IDs; new cases are 10--13.

The inertia-only experiment changes actual axial rotor inertia while the
allocator retains nominal inertia. This introduces both a physical momentum
asymmetry and a model error. It does not isolate a purely erroneous inertia
estimate with an otherwise identical plant. The installed rigid-body tensor
remains fixed, consistent with the preceding uncertainty study.

`MISSION_ESTIMATION_ABLATION` runs the four new cases and reads the existing
nominal/corner outputs. Do not regenerate those with different parameters
without rerunning the comparison. The table reports last TURN errors rather
than treating post-abort reference-hold errors as completed-mission accuracy.
For completed cases this is immediately before successful capture; for failed
cases it is immediately before timeout, so elapsed times differ.

Moment-model RMSE compares physical and estimated CMG torque at the SAME
actual gimbal rates, spin states and body rates. This diagnoses momentum-model
error, excluding command-versus-actual servo lag. It is computed over each
case's TURN duration and is not a common-horizon total tracking-error metric.

## Reproduction

```matlab
MISSION_ESTIMATION_ABLATION
RUN_ROLL_TURN_SURGE(struct('estimationCase',10,'sampleTime',.025,'maxStep',.005))
RUN_ROLL_TURN_SURGE(struct('estimationCase',12,'sampleTime',.025,'maxStep',.005))
VERIFY_MISSION_ESTIMATION_ABLATION
```

To rebuild only the summary from existing histories, use
`MISSION_ESTIMATION_ABLATION(false)`. CSV/MAT summaries are saved under
`Working Results/mission_estimation_ablation`. Production controller and
capture thresholds are unchanged; this is diagnosis, not a corrective law.

## Results

| Isolated perturbation (rotor 1, rotor 2) | Outcome | Last TURN plane error | Surge start |
|---|---|---:|---:|
| Speed bias (-5%, +5%), exact inertia | Pass | 0.3870 deg | 37.90 s |
| Speed bias (+5%, -5%), exact inertia | Pass | 0.3988 deg | 38.30 s |
| True inertia (-10%, +10%), unbiased speed | Turn timeout | 1.8629 deg | Never |
| True inertia (+10%, -10%), unbiased speed | Turn timeout | 1.4212 deg | Never |

These table outcomes use the original 0.05-s supervisor grid. The nominal
case passes. All four combined corners still fail. Thus the
tested inertia perturbations alone are sufficient to reproduce the failure;
the tested speed bias alone is not. This does not imply that all speed biases
are benign, nor does it quantify a universal relative sensitivity to unequal
5% and 10% error ranges.

Both inertia-only cases abort at approximately 55.30 s. Their last TURN heading
errors are 1.3341 and 1.0056 degrees; rate norms are 0.05875 and 0.00319 deg/s.
Plane capture requires <=0.5 degree, so angular-rate settling is not enough.
Neither isolated inertia case records actuator saturation or pitch-neutral
roll-infeasibility. Aft thrust remains zero after failed capture.

Speed-only successful missions finish with heading errors 0.4734 and 0.3718
degrees and maximum surge-plane errors 0.6565 and 0.5142 degrees. Their largest
out-of-plane displacement is approximately 0.022 m. These differences are
measurable, but both remain inside existing development-screen thresholds.

Moment-model RMSE does not alone rank mission outcomes: the persistent
cross-axis response and capture logic matter, and the cases have different
trajectories/durations. No controller, timeout or capture tolerance was changed
to make an outcome pass.

### Refinement caveat

The speed-only (-5%,+5%) refined run still completes with almost unchanged
tracking, but records one pitch-neutral roll-feasibility flag versus zero at
the coarse step. Its strict overall screen consequently changes from pass
to fail. Do NOT claim refinement-invariant speed-bias robustness. The inertia-
only (-10%,+10%) refined run reproduces turn timeout. Both terminal behaviors
are preserved, but not every binary diagnostic. The extra flag occurs at
1.325 s during ROLL, not at turn capture. Investigate the sampled
roll-feasibility check before claiming all speed-only cases pass. Final
heading changes were 0.000253 and 0.001073 degrees for refined cases 10 and
12. Isolation/trajectory checks passed; strict-screen invariance did not.

## Conclusion and next diagnostic

The failure tracks unequal true rotor inertia in this tested grid, not a need
for both speed bias and inertia mismatch. However, the existing experiment
confounds physical momentum asymmetry with incorrect allocator inertia.
Next compare the SAME unequal-inertia plant with nominal versus exact inertia
knowledge in the allocator. If exact knowledge recovers capture, improved
parameter identification/allocation is a candidate remedy. If it does not,
investigate physical cross-axis coupling and coordination before tuning.

Trello: mark “Separate speed-bias effects from inertia-mismatch effects”
complete. Keep “Resolve unequal-inertia turn-capture failures” open, with the
exact-inertia diagnostic as its next substep. Also retain the speed-only
roll-feasibility sampling sensitivity for follow-up. This is not yet a compensation
implementation or full robustness validation.
