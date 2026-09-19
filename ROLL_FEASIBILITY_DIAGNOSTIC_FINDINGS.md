# Step-sensitive roll-feasibility flags: speed-only case

## What the flag means

The mission currently tests whether the true CMG mapping B can produce the
requested [K; M=0], including the body yaw-rate bias, using instantaneous
gimbal rates bounded at +/-20 rad/s. Despite the `rollFeasible` name this
is a pitch-neutral two-axis requirement, not a test of roll torque alone.
It also does not include finite-servo acceleration or gimbal-stop reachability.

The original diagnostic uses a pseudoinverse solution and compares rates to
the bound. A separate diagnostic now minimizes ||B*u-rhs|| over the full
two-rate box by checking its interior and all four edges. This avoids relying
only on a minimum-norm solution at rank deficiency and quantifies unattainable
moment rather than just reporting a boolean. No production tolerance or
pass criterion has been changed.

## Evidence

Case 10 is the speed-only [-5%,+5%] bias run. Its 0.05-s supervisor samples
miss a flag; its 0.025-s samples flag time 1.325 s during initial ROLL. Both
missions complete with similar tracking, but only the coarse output passes
the strict zero-infeasible-sample screen.

Dense replay of each saved trajectory over 1.2--1.45 s reveals an interval of
infeasibility in BOTH, showing how the coarse samples can miss it. The
0.05-s trajectory interpolation flags approximately 1.3145--1.3220 s; the
0.025-s interpolation flags approximately 1.3197--1.3320 s. These different
intervals show why interpolation alone must not establish timing or peaks.

A third check locally reintegrates the fine trajectory from its saved state
at 1.2 s, using 0.0001-s maximum step and outputs, tighter ODE tolerances and
the unchanged held ROLL references. This is a local diagnostic, not a full
mission rerun or a proof covering every possible crossing.

The locally integrated interval is approximately 1.3209--1.3311 s (10.2 ms
between first and last flagged samples). The largest sampled exact-inverse
rate is 2647 rad/s versus the 20-rad/s bound; this near-singular peak is
sampling dependent, not a required practical motor rating. The largest
minimum achievable two-axis residual is 0.0038364 N m. Component maxima
are approximately 0.0003681 N m roll and 0.0038187 N m pitch; they need not
occur at the same instant. The pitch component dominates this diagnostic.

This is not solely roundoff or a pseudoinverse classification artifact: the
box-constrained minimum residual is well above the existing 1e-7 N m test
tolerance. Nor does it mean actual rates reach thousands of rad/s: the
production allocator is damped and actual actuator rates remain bounded.
The condition reflects exact pitch-neutral tracking becoming incompatible
with the instantaneous rate bounds near column alignment under biased
allocation and altered geometry.

## Conclusion / Trello

Complete: investigate step-sensitive roll-feasibility flags in the speed-only
case. Sparse output sampling missed a real short pitch-neutral allocation
limitation. It does not by itself prove inability to produce the required
roll component, trajectory instability, or motor saturation.

Next: report roll-only authority separately from pitch-neutral two-axis
feasibility and track bounded residual magnitude/duration at adequate time
resolution. Decide explicit acceptable cross-axis transient criteria before
changing the strict mission screen. Do not delete the flag, inflate numerical
tolerances, or claim the coarse-grid pass proves absence of transient loss.
This diagnostic does not fix the separate sustained turn-capture failure in
unequal-inertia cases. No controller changes were made.

## Follow-up: roll-only authority and transient response

The helper now reports two different geometric questions. For bias-subtracted
roll demand rhs_K, the instantaneous roll-only rate-box capacity is
L*(abs(B11)+abs(B12)); the margin is this capacity minus abs(rhs_K).
A second calculation enforces exact roll and minimizes absolute pitch error
over the intersection of the exact-roll line and the rate box. This is a
counterfactual static allocation, NOT a replacement production controller.

Results from the finely integrated 1.2--1.45 s interval:

| Quantity | Value |
|---|---:|
| Roll-only infeasible samples | 0 |
| Minimum static roll-only margin | 5.1106 N m |
| Pitch-neutral residual above 1e-7 N m: sampled time integral | 0.0103 s |
| Peak minimum pitch residual with exact roll prioritized | 0.0038542 N m |
| Absolute pitch residual impulse, static roll-priority solution | 2.003e-5 N m s |
| Peak actual roll tracking error | 0.0070726 N m |
| Peak actual pitch tracking error | 0.0039533 N m |
| Actual absolute roll-error impulse over full local window | 0.0012824 N m s |
| Actual absolute pitch-error impulse over full local window | 0.00047988 N m s |
| Peak actual gimbal rate / acceleration | 0.75458 rad/s / 18.677 rad/s^2 |
| Maximum absolute Euler pitch in local window | 0.00041629 deg |

The 10.3-ms sampled duration is the integral of the binary residual flag at
0.1-ms spacing; the first-to-last sample span was 10.2 ms. Neither is an exact
continuous event duration. Absolute impulses do not imply signed momentum
accumulation and should not be equated with attitude change.

Static margin allows instantaneous access to any rates in the box; it does
not demonstrate finite-acceleration reachability. Actual tracking errors are
computed from the real finite-servo response and include estimation, damping
and lag effects. They persist outside the geometric-infeasibility interval,
so their 0.25-s impulses cannot be compared as if they arose only during the
10-ms event. Counterfactual roll-priority moments are not integrated into a
new vehicle trajectory. The observed pitch trace belongs to the unchanged
controller and includes earlier history and cross-axis dynamics; it is not a
causal isolation of the flagged interval's pitch response.

The conclusion for this local speed-bias case is loss of exact pitch-neutral
allocation, not loss of roll-only geometric authority or a large observed
pitch departure. Static feasibility, actual servo tracking and vehicle motion
are now separately available in the diagnostic outputs. The legacy mission
boolean/screen is unchanged for comparability. A revised publication screen
must explicitly specify permitted cross-axis error and evaluate adequate
time resolution rather than silently relabel old passes.

Tests cover exact-roll allocation with unavoidable pitch error and with fully
feasible pitch cancellation, in addition to the earlier bounded-solve tests.
The roll-only/pitch-neutral distinction is verified on the local trajectory.
Trello: complete this investigation; retain definition/validation of revised
reporting criteria and the separate unequal-inertia capture failure work.

## Reproduce

Run `INVESTIGATE_ROLL_FEASIBILITY`, then
`VERIFY_ROLL_FEASIBILITY_DIAGNOSTIC`. CSV/MAT diagnostic results are saved to
`Working Results/roll_feasibility_diagnostic`. The helper tests include
feasible, rate-limited, rank-deficient feasible and rank-deficient infeasible
linear systems. Existing mission data and controller settings are untouched.
