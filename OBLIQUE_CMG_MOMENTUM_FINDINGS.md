# CMG momentum envelope for large oblique turns

## Conclusion

The current 18-second large-oblique-turn failures are **not explained by
exhaustion of rotor spin-momentum magnitude or loss of instantaneous roll
allocation feasibility**. The roll controller tracks its requested moment,
but the moving roll target develops tracking error and the hybrid gate removes
the yaw correction, including braking. Heading then overshoots. A longer
uncontrolled continuation eventually reaches gimbal travel and steering limits.
These are distinct failure stages; increasing flywheel momentum is not yet
justified as the first remedy.

## Reproduce

Run `OBLIQUE_CMG_MOMENTUM_ENVELOPE` from the simulation folder after generating
the symmetric constant-speed dual VFR baseline. It reads that saved baseline
without modifying it or the production controllers. It writes full histories,
configurations, the baseline snapshot, a CSV summary, and two figures to
`Working Results/oblique_momentum_envelope/`.

`OBLIQUE_CMG_MOMENTUM_ENVELOPE(true)` refreshes figures and derived metrics
from its saved characterization, without reintegrating the scenarios.

Nine diagnostic scenarios use zero commanded lateral translation:
45-degree plane with 30, 35, 40, 45, and 60-degree heading commands;
the -45/-45-degree mirror; a 60-second 45/45-degree continuation;
a relaxed roll-rate gate; and a half-step numerical check.
The normal scenarios retain the previous sweep's 0.5/1.5-degree roll-error
gate, 0.5/3-degree-per-second roll-rate gate, gains, and actuator limits.
The relaxed-rate test is diagnostic only and does not change the angle gate.

## Envelope definition

For the spin-axis convention implemented in `CMG.m`,

`H_B = sum_i I_i Omega_i [sin(alpha_i), -cos(alpha_i), 0]`.

At the saved +/-1200 rpm baseline, each rotor has spin-momentum magnitude
0.129387 N m s, and `sum |I_i Omega_i| = 0.258774 N m s`.
The initial -15/+15-degree preload gives `Hx(0) = 0.066976 N m s`.

The triangle-inequality outer bound is `|H_B| <= 0.258774 N m s`.
The actual angle-restricted reachable set is the image of both gimbal angles
within +/-100 degrees; it is sampled directly in the momentum-map figure.
It is not the entire outer disk, and a low magnitude fraction alone does not
prove local controllability or access to every point in that set.

The instantaneous roll interval is calculated by intersecting the gimbal-rate
box with the pitch-neutral condition:

`[K; M] = B alphadot + B [r; r]`, with `M = 0`.

It includes yaw-rate bias, the 20 rad/s rate bound, and outward-motion
restrictions at the angle bounds. It excludes finite acceleration and servo
lag; actual moment residuals and actuator histories are reported separately.
This replaces a symmetric scalar capacity approximation for this diagnostic,
without changing the existing allocator or older analysis functions.

The reaction identity is `tau_CMG = -(dH_B/dt + omega_B cross H_B)`.
In particular, `dHx/dt = -K + r Hy`. Integrating roll torque alone during a
turn is therefore not a valid measure of body-axis momentum depletion.
This is the rotating-frame angular-momentum balance; see
[MIT Engineering Dynamics](https://ocw.mit.edu/courses/2-003sc-engineering-dynamics-fall-2011/pages/angular-momentum-and-torque/).
The check concerns modeled rotor spin momentum, not a complete conservation
audit of the vehicle, finite gimbal/frame inertia, and surrounding fluid.

## Results from the current baseline

All angles below are degrees. Full heading error is the angle between the
actual and desired nose vectors, not only the controller's signed projected
heading metric. These are diagnostic outcomes, not a new mission pass/fail rule.

| Plane / heading command | Final full heading error at 18 s | Peak rotor magnitude / outer bound | Maximum absolute gimbal angle |
| --- | ---: | ---: | ---: |
| +45 / +30 | 0.378 | 26.10% | 46.70 |
| +45 / +35 | 25.533 | 26.07% | 74.93 |
| +45 / +40 | 28.845 | 26.04% | 82.75 |
| +45 / +45 | 29.861 | 26.01% | 88.10 |
| +45 / +60 | 26.883 | 26.45% | 94.03 |
| -45 / -45 | 29.807 | 55.97% | 91.22 |

Across these six cases there are no sampled requests outside the corrected
instantaneous roll interval and no reported gimbal angle, rate, or acceleration
limit events. In the +45/+45 case, roll-moment RMSE after 3.5 s is
`2.65e-5 N m`. During that phase the gimbal separation remains at least
28.46 degrees from a parallel/antiparallel steering configuration. The early
near-singularity during initial roll is not the cause of the later heading
failure. Neither an Euler pitch singularity nor an exactly undefined maneuver
plane occurs in these 18-second runs.

The fixed preload is not reversed for the negative maneuver. Its larger
transient momentum usage is therefore expected to differ from the positive
case; the two runs are not fully mirrored actuator initial conditions.

## Failure sequence and diagnostic distinctions

For +45/+45, thrust activation drops below 0.5 at approximately 6.25 s.
At 10 s, actual roll is 38.03 degrees and the moving target is 35.13 degrees.
The roll error exceeds the angle gate's 1.5-degree cutoff. The ungated yaw
controller requests about -0.522 N m to brake a positive body yaw rate of
0.0907 rad/s, but the gate makes the commanded yaw moment zero. By 18 s the
ungated request is -1.343 N m and the gate remains zero.

The roll law damps absolute Euler roll rate and does not track the derivative
of the moving roll reference. Small net requested roll moment can coexist
with persistent roll error as the reference moves. The gate also multiplies
the yaw damping term, so it removes braking when alignment is lost. These
code paths and histories identify a controller-coordination failure mechanism;
they do not establish that any particular replacement law will succeed.

Relaxing only the absolute roll-rate gate does not recover the maneuver:
final full heading error is 30.758 degrees and the angle gate still shuts
thrust off. Simply waiting also does not recover it.

The 60-second continuation first exceeds the nominal gimbal-angle bound at
about 24.39 s and reaches 103.52 degrees despite a configured 100-degree limit.
Instantaneous pitch-neutral roll infeasibility also begins around 24.39 s.
The present stopping-distance/servo implementation is **not a strict hard
stop**. Its post-limit trajectory must not be treated as hardware-valid, and
the 93.55-degree final heading error is a numerical failure diagnostic only.
The rotor-momentum magnitude still remains below 36.7% of the outer bound;
angle-limited accessible motion is distinct from exhaustion of that bound.

## Verification

- Four analytic regression cases test rate-envelope intervals, rank-deficient
  but roll-capable geometry, pitch infeasibility, and an outward angle stop.
- The rotor reaction identity closes to at most `2.46e-16 N m` across the runs.
- Halving maximum integration step/output spacing from 0.01 to 0.005 s changes
  the +45/+45 final heading error by less than `1e-7` degree. The independently
  integrated body-momentum residual decreases from `1.97e-4` to `4.98e-5 N m s`,
  consistent with trapezoidal output quadrature error in the initial transient.
- The longer post-stop trajectory is not covered by that half-step check.
- This is a limited directional sweep, not a continuous heading-envelope proof
  or a validation of hardware, disturbance rejection, or arbitrary 6-DOF control.

## Checklist update

Completed: characterize the rotor momentum envelope and instantaneous roll
authority for the tested large oblique heading commands.

Next: revise moving-reference roll tracking and thrust-gate coordination so
yaw braking is not inadvertently removed; validate against the same failed
cases without relaxing the acceptance criteria.

Also retain: implement and verify physically consistent gimbal hard-stop
behavior before relying on long-duration or near-limit results.

Keep momentum unloading/singularity avoidance open for sustained operation;
this characterization does not establish unlimited momentum management.
