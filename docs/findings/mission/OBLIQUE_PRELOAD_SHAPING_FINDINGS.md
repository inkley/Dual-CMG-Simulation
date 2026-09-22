# Preload and heading trajectory shaping for oblique turns

## Main result

Preload changes alone did not recover the tested +/-45-degree oblique heading
turns. A 24-second quintic heading-reference transition, using the existing
-15/+15-degree preload, recovered both directions without changing gains,
thrust gating, flywheel speeds, or actuator limits. This is a slow, finite
maneuver candidate, not evidence that the faster coordination problem is fixed.

## Method and reproducibility

Run `OBLIQUE_PRELOAD_SHAPING_SWEEP` after generating the saved symmetric
constant-speed dual VFR baseline. It writes a summary CSV and full vehicle
histories to `Working Results/oblique_preload_shaping/`, separate from the
baseline. The saved MAT file includes the baseline snapshot, variant matrix,
initial states, and scenario configurations. Production control functions
and `AUV_SIM.m` defaults are unchanged.

The sweep tests ten variants in each direction, with plane and heading commands
of +45/+45 or -45/-45 degrees and zero commanded lateral translation:

- Step commands with preload magnitudes of 5, 15, 30, and 45 degrees.
- Step command with the nominal preload sign reversed.
- A step delayed until 4 s.
- Quintic heading transitions of 4, 8, 16, and 24 s, starting at 4 s.

For preload parameter beta, initial gimbal angles are [-beta,+beta]. Rotor
speeds stay at [-1200,+1200] rpm. This comparison changes initial stored momentum
and travel reserve; it does not include the time, effort, or reaction needed
to establish that preload on a freely floating vehicle.

The shaped command rotates the desired nose vector within the fixed inertial
maneuver plane, rather than interpolating Euler yaw. Its angle is
`headingTarget * (10 u^3 - 15 u^4 + 6 u^5)`, where
`u = clamp((t-4)/duration,0,1)`. Reference angular rate and acceleration vanish
at both endpoints. No heading-rate feedforward is added; the existing PD law
receives the time-varying desired heading vector. The 4 s start is a scheduled
delay, not a new state-triggered capture detector; the existing thrust gate
still determines activation.

All variants receive the same 40 s budget. The 24 s reference reaches its
endpoint at 28 s, followed by 12 s of continued control. Thus this experiment
does not claim an 18 s maneuver or identify the fastest feasible trajectory.
Runs terminate at the configured 100-degree gimbal bound; trajectories after
that event are not simulated because hard-stop dynamics remain unvalidated.
Errors reported for terminated runs are at termination, not at 40 s.

## Screening criteria

No gain, gate, angle/rate limit, or existing 1-degree active-plane / 15 mm
translation-drift tolerance is relaxed. Candidate screening additionally
requires full 3-D nose-vector heading error within 1 degree throughout the
final five seconds, final gate >=0.95, no angle-stop event, and no reported
gimbal rate/acceleration or thruster allocation saturation. Candidate roll
requests are independently checked for a rate-bounded exact pitch-neutral
allocation. This is a conservative candidate screen, not a new global
mission-validation claim.

## Results

| Variant | Positive turn | Negative turn |
| --- | --- | --- |
| All five preload-only variants | Fail; angle-stop event | Fail; angle-stop event |
| Delayed step | Fail; angle-stop event | Fail; angle-stop event |
| 4 s or 8 s shaping | Fail; angle-stop event | Fail; angle-stop event |
| 16 s shaping | Fail; 39.58 deg error at 40 s | Fail; angle-stop event near 39.71 s |
| 24 s shaping | Pass | Pass |

For nominal step commands, the positive and negative cases reach the angle
boundary near 24.38 and 20.89 s, respectively. Increasing preload to 45 degrees
brings those events forward to approximately 12.38 and 12.22 s. More initial
preload is therefore not an improvement for this controller/trajectory pair.

The successful candidates retain the nominal -15/+15-degree preload:

| Metric | Positive turn | Negative turn |
| --- | ---: | ---: |
| Final full heading error at 40 s | 0.129 deg | 0.110 deg |
| Maximum plane error while gate >=0.95 | 0.635 deg | 0.635 deg |
| Maximum absolute gimbal angle | 62.10 deg | 63.61 deg |
| Peak gimbal rate | 0.880 rad/s | 0.889 rad/s |
| Peak gimbal acceleration | 94.25 rad/s^2 | 94.25 rad/s^2 |
| Peak module force magnitude | 0.0921 N | 0.0918 N |
| Final thrust-enable multiplier | 1.0 | 1.0 |

Both remain within 1 degree of the final heading over the last five seconds.
This is endpoint recovery, not tight tracking of the entire shaped reference:
the plotted trajectories lag the reference and overshoot the final heading by
approximately 3 degrees. The thrust gate still reduces activation to roughly
0.17 during part of the turn before recovering. The 0.635-degree plane metric
applies only where activation is at least 0.95, not to the complete trajectory.
Translation and out-of-plane position remain zero in these symmetric,
zero-translation tests; this is not a combined sway maneuver demonstration.
The force values are cycle-averaged provisional model outputs, not demonstrated
vortex-ring hardware resolution or minimum controllable thrust.

## Interpretation and next action

The slow reference gives the existing roll controller time to follow the
changing plane-alignment roll target. It avoids the sustained loss of yaw
correction observed with step commands. It does not remove the underlying
absolute-roll-rate damping and gated-yaw-braking limitations. The result
supports retaining the nominal preload and using the 24 s shape as a comparison
case while developing a better coordinated controller.

This is a discrete parameter study, not preload optimization or a minimum-time
search. The two successful maneuvers do not establish robustness to other
headings, combined translation, disturbances, parameter mismatch, or repeated
operation. Both directions use the same actuator preload rather than fully
mirrored actuator initial conditions.

For numerical verification run
`OBLIQUE_PRELOAD_SHAPING_SWEEP([10,20],.005)` and
`VERIFY_OBLIQUE_SHAPING_RESULTS`. This checks heading/gimbal agreement within
0.001 degree, sampled pitch-neutral roll-allocation feasibility, and exports
`SHAPED_OBLIQUE_TURNS.png`.

## Trello update

Completed: evaluate preload and trajectory-shaping options for the tested
large oblique turns. Preload changes alone failed; a 24 s quintic heading
transition recovered both +/-45-degree cases within the fixed model limits.

Next: improve moving-reference roll tracking and yaw-braking coordination,
then compare against this slow candidate with unchanged acceptance criteria.
Keep physical hard-stop validation and broader maneuver testing open.
