# Allocation scope decision for the first manuscript results

## Decision: retain the existing damped allocator

No new uncertainty-aware control law is required **to report the currently
tested, bounded study**. Retain the constant-speed dual-CMG damped least-squares
allocator and the provisional +/-5% per-rotor inertia-knowledge target. This is
a scope decision based on existing evidence, not a finding that uncertainty-aware
allocation is unnecessary in all operations or that a candidate robust allocator
has been tested and rejected.

The existing regularization limits inversion sensitivity but is not an explicit
uncertainty-aware or worst-case robust design. Do not describe it as one.

## Evidence and retained limitations

- All 14 selected +/-5% disturbed-hold points passed without changing gains,
  damping, limits, or acceptance criteria.
- Worst peak/RMS roll error: 1.514506/0.835458 degrees, leaving 0.485494/0.164542
  degrees of margin to the respective criteria.
- No recorded actuator-limit violations; momentum excursions remained inside
  the finite-duration planning allowance.
- The -10%/+10% estimation-error challenge failed at 2.009986 degrees, including
  a half-step repeat. The earlier nominal-estimate challenge also remains failed.
- Exact-inertia knowledge resolves the diagnosed unequal-inertia case, supporting
  sensitivity to model knowledge near poorly conditioned steering as a limitation.

These observations apply to the fixed unequal-inertia plant, 15-second synthetic
sinusoid, selected signed constant biases, initial preload, and 200-second hold
tests already documented. They do not validate every intermediate error pair,
phase, frequency, combined mismatch, or disturbed hybrid maneuver. The +/-5%
target is a provisional modeling/calibration requirement, not demonstrated
hardware accuracy. Persistent bias still consumes momentum; allocation changes
cannot create an external roll-unloading moment.

## Why stop allocator development here?

An explicit uncertainty penalty, adaptive damping, or roll-priority allocation
could change roll/pitch tradeoffs and the gimbal trajectory. Introducing one now
would require renewed nominal, mission, uncertainty, and momentum regressions.
The passing selected target cases do not presently justify that expansion for
the first bounded simulation-results package. Preserve the outside-target
failures rather than widening the supported claim or loosening criteria.

## Manuscript-ready statement

The existing damped allocator was retained. Selected disturbed-hold simulations
met the declared roll and momentum criteria with per-rotor inertia-estimation
errors up to +/-5%. Larger opposite-sign errors produced a reproducible tracking
failure near sensitive steering configurations. Consequently, the results support
the tested finite-duration cases under a provisional inertia-knowledge assumption,
not general robustness to rotor-model uncertainty.

## Checklist disposition

Close **Evaluate uncertainty-aware allocation if needed near poorly conditioned
steering** as: evaluated; additional allocator development deferred outside the
first results package. No production-controller modification was made.

Proceed to configuration/documentation cleanup and the reproducible publication
comparison batch. Reopen allocator development only if a retained publication
case fails the unchanged criteria, or the intended claim requires larger or
less-constrained uncertainty than the tested cases. Such cases must not be
silently excluded after a failed run.

## Verification

`VERIFY_ALLOCATION_SCOPE_DECISION` checks the fourteen saved target points,
unchanged criteria and allocator settings, and the retained refined challenge
failure. It saves an evidence summary under `Working Results/inertia_estimation_bounds`.
It does not run a new robust-control experiment.
