# Roll tracking and momentum diagnostics

## Scope

This follow-up retains the existing gains, actuator limits, and original tracking
criteria. It analyzes the four feedback disturbance cases, repeats the worst
case at half the integration/output step, extends its zero-mean sinusoid from
60 to 180 seconds, and separately adds a constant 0.003 Nm bias from time zero
as a momentum-storage stress test. The extended runs have a 200-second horizon.
The bias case is not part of the original nine-case pass-rate comparison.
The initial unguarded bias run was manually stopped after excessive runtime;
no outcome is claimed for it. The reported repeat has an explicit 85-degree
gimbal diagnostic stop before the 90-degree symmetric roll steering singularity.
This is not a change to the physical 100-degree limit or tracking thresholds.

## Results (2026-09-18)

The three original tracking failures are explained by closed-loop disturbance
response, not recorded actuator saturation. Late-cycle fitted roll amplitudes
agree closely with the local servo-lag prediction:

| Load (Nm) | Period (s) | Fitted amplitude (deg) | Predicted amplitude (deg) |
|---|---|---|---|
| 0.01 | 5 | 1.2453 | 1.2452 |
| 0.01 | 15 | 1.8491 | 1.8489 |
| 0.03 | 5 | 3.7351 | 3.7356 |
| 0.03 | 15 | 5.5474 | 5.5466 |

Across the original feedback cases, peak actual-versus-requested roll moment
residual was at most 0.000453 Nm. Peak gimbal rate and acceleration were only
0.1523 rad/s and 0.1902 rad/s^2, versus configured limits of 20 and 500.
Peak gimbal angle was 53.99 degrees. No actuator-limit flags were recorded.
Steady error amplitudes are distinct from onset/recovery peak-error metrics.

Halving both maximum integration step and output interval from 0.01 to 0.005 s
for the worst original case changed roll at common sample times by less than
1.2e-9 degrees and rotor Hx by less than 2.3e-11 N m s. Sampled peak error was
5.5524 versus 5.5525 degrees because the finer grid resolves the peak differently;
the tracking-failure classification is unchanged.

With 180 seconds of zero-mean loading (200-second total run), peak error stayed
5.5524 degrees and peak momentum excursion stayed 0.14235 N m s. Same-phase
cycle-to-cycle fitted drift was approximately 1.52e-10 N m s per second, and the
final rotor momentum change after recovery was 5.88e-7 N m s. No saturation was
recorded. This establishes repeatability over twelve load cycles, not unlimited
operation. The original RMS criterion is still exceeded (extended RMS 3.9141 deg).

Adding a constant positive 0.003 Nm bias from t=0 to that same sinusoid reached
the 85-degree diagnostic guard at **17.007 s**. Rotor Hx had increased by
0.19081 N m s, almost consuming the approximately 0.192 N m s of initial positive
storage headroom before the symmetric 90-degree extremum. The peak error up to
the stop was 6.1178 degrees. No rate/acceleration saturation was recorded before
the stop; this is a momentum/geometry boundary, not a completed sustained run.
The bias-case RMS in the CSV covers only its partial load window and must not
be compared to complete-run RMS as though both covered 60 or 180 seconds.

The scalar momentum-balance residual was below 3.15e-7 N m s for the original
feedback runs, and below 1.58e-7 N m s for the extended runs. The finer-step
worst case reduced its residual from 1.57e-7 to 3.93e-8 N m s, consistent with
the sampled numerical integration used for the impulse balance.

## Development decision

The bounded disturbance-rejection and momentum-accumulation evaluation is
complete for these symmetric synthetic loads. It is **not** a claim that all
disturbance requirements pass. Keep the three original tracking failures visible.
The next decision is to define the intended disturbance amplitude/frequency/bias
envelope, then assess controller changes against the same declared tolerances.
Integral action alone would not solve finite momentum capacity under a persistent
load. Sustained biased operation needs an external roll-moment source or an
explicit finite-duration operating restriction; none is added in this work.

## Tracking mechanism

For the symmetric pure-roll case, let e = phi - phi_desired. The approximate
small-signal equation with ideal torque tracking is

    Ix * e_ddot + Kd * e_dot + Kp * e = K_external.

There is no integral term or disturbance feedforward. A slowly varying external
moment therefore requires a nonzero angle error to generate an opposing PD
moment. At low frequency, the error amplitude approaches load amplitude / Kp.
This explains why adequate actuator authority alone does not imply small error.

The diagnostic additionally includes the configured gimbal-rate servo lag tau:

    |e / K_external| = |1 / (Ix*s^2 + (Kp + Kd*s)/(1 + tau*s))|.

This local approximation neglects nonlinear roll drag and changing gimbal
geometry. The script compares it with a sine/cosine fit to late complete load
cycles; it is explanatory, not a replacement for nonlinear simulation.

## Momentum interpretation

The diagnostic checks the symmetric pure-roll balance

    Ix*(p-p0) + (Hx-Hx0) = integral(K_external + K_hyd) dt,

where Hx is the sum of the two rotor body-x momentum components. The scalar
check explicitly requires negligible pitch/yaw motion. It is not a general
six-DOF inertial-frame angular-momentum validation.

Periodic excursion, same-phase cycle-to-cycle drift, and final residual momentum
are reported separately. Zero-mean external impulse does not guarantee small
within-cycle momentum excursions. A persistent bias can require ongoing storage;
the installed center-plane thrusters do not supply external roll unloading.

## Reproduction

Run the original nine-case harness first, then:

```matlab
EVALUATE_ROLL_DISTURBANCE_REJECTION;
REPORT_ROLL_REJECTION_DIAGNOSTICS;
```

Results, the diagnostics CSV, and comparison PNG are saved under
`Working Results/roll_disturbance_tests`. The new MAT files use `analysis` for
their diagnostic summaries. No control-law changes are made by these scripts.
