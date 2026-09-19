# Bounded pitch recovery for unequal-inertia turn capture

## Controller change

The legacy hybrid controller commands zero CMG pitch moment. Roll aligns the
thruster plane relative to the current nose direction; it cannot independently
remove a nose-direction component normal to the desired inertial plane.
The tested correction adds an explicit bounded pitch request to the existing
dual roll/pitch allocator after initial roll capture:

    e_plane = asin(clamp(n_desired dot body_x, -1, 1))
    M_d = clamp(0.5*e_plane - 2*q, -0.02, +0.02) N m

Here n_desired is the fixed maneuver-plane normal and q is body pitch rate.
In the intended aligned neighborhood, positive q reduces a positive
n_desired dot body_x. This is local plane recovery, not a global SO(3) law.
The gains have units N m/rad and N m s/rad. The 0.02-N m bound limits the
request; it does not add physical actuator authority or override CMG limits.

The correction is active only in TURN, SURGE and COMPLETE, not ROLL or ABORT.
Existing roll feedback, thruster control, capture dwell/tolerances, timeouts,
spin speeds, allocation damping, servo dynamics and actuator limits remain
unchanged. It uses vehicle attitude/rate feedback and the commanded plane,
not true rotor inertia unavailable to the estimator. The uncertain allocator
still receives its original biased speeds and nominal inertias.

This is an explicit extension of the dual-CMG role from roll-only to roll
with limited pitch recovery. It does NOT claim arbitrary simultaneous moment
control through singularities. Tests and manuscript descriptions must reflect
this architecture change. No direct pitch thruster is added.

## Reproduce / preserve comparisons

```matlab
VERIFY_PLANE_PITCH_CORRECTION
PLANE_PITCH_CORRECTION_SWEEP
RUN_ROLL_TURN_SURGE(struct('estimationCase',12,'planePitchCorrection',true))
```

`planePitchCorrection` remains false by default so prior studies and saved
baseline comparisons retain their original controller. Enable it explicitly
for the corrected mission. Histories include requested/achieved CMG moments;
outputs use a `plane_pitch_correction` subfolder without overwriting legacy
results. The 13-case sweep includes nominal, common/opposite combined errors,
speed-only and inertia-only cases. Summaries are in
`Working Results/plane_pitch_correction/`.

For finer-step checks, repeat cases 6, 9, 10, 12 and 13 with sampleTime=.025
and maxStep=.005; also run nominal (-45,-45) and (0,30) plane/heading pairs.
`VERIFY_PLANE_PITCH_CORRECTION_RESULTS` checks the saved artifacts.

The pre-existing short initial-roll pitch-neutral infeasibility can persist:
this correction intentionally does not act during ROLL. Completing capture
does not erase strict screen failures or justify relaxed acceptance criteria.

## Results at the original supervisor step

All 13 cases completed turn capture and the cruise segment, including all six
previously timing-out opposite-inertia combined/isolated scenarios. Eleven
pass the unchanged strict mission screen. Cases 6 and 9 still fail solely
because of one recorded initial-ROLL feasibility flag each. They complete
the mission; they are not silently counted as passing.

Across the 13 corrected missions:

- Maximum final heading error: 0.002883 degrees.
- Maximum surge-plane error: 0.33682 degrees.
- Maximum out-of-plane displacement: 0.00854 m.
- Maximum cross-track displacement: 0.000429 m.
- Final speed: approximately 0.499593 m/s.
- Maximum gimbal angle: 66.414 degrees.
- No recorded actuator saturation or premature aft thrust.

Peak pitch request was 0.001956 N m, below the 0.02-N m correction bound.
No sampled allocation-infeasibility flags occurred after ROLL in the 13
original-step histories. Plant identity, unchanged capture criteria, phase
gating, request bounds and controller regression checks passed. Negative-
oblique (-45,-45) and horizontal (0,30) nominal cases also passed their
unchanged mission screen with the correction enabled.

Isolated inertia cases 12 and 13 previously aborted near 55.3 s; now surge
starts at 36.70 and 35.85 s, with final heading errors 0.001027 and 0.000535
degrees. This is recovery of the intended capture condition, not a timeout
extension or looser tolerance. The extra pitch request is the material
control-architecture change and must be disclosed when comparing results.

Finer-step cases preserve completed maneuvers, but the speed-only case 10
again records a brief initial-roll feasibility flag and fails the strict
screen. The default boolean is retained for historical comparability; no
claim of all-case, resolution-independent robustness is made.

## Trello handoff

Complete: address unequal-inertia turn-capture failures and rerun the 13
estimator cases without changing acceptance criteria. Bounded pitch recovery
after roll capture recovered mission completion in every tested case.

Keep open: initial-roll pitch-neutral feasibility/transient acceptance and
its adequate temporal resolution. Full strict-screen success for every
uncertainty case has not been established. Also retain broader scenario and
hardware/model limitations; these results do not establish arbitrary full
attitude control by two CMGs.
