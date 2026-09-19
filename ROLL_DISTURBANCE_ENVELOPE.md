# Provisional finite-duration disturbance operating envelope

**Mismatch qualification:** the subsequent selected mismatch screen passed
11/12 cases, with an unequal-inertia/negative-bias peak-error failure. This
remains a nominal specification, not a mismatch-robust guarantee. See
`DISTURBANCE_MISMATCH_FINDINGS.md`.

## Intended use

This is a conservative model-development specification for the existing
symmetric dual-CMG, initialized 45-degree roll hold. It is not a maximum physical
capability, an ocean-wave/sea-state rating, or a robustness guarantee. The
original larger-load failures remain in the record; this specification does not
reclassify them or replace their acceptance criteria.

| Quantity | Provisional operating value |
|---|---|
| Sinusoidal body-roll moment amplitude | 0 to 0.006 Nm |
| Frequency | 0.0667 to 0.2 Hz (periods 15 to 5 s) |
| Constant body-roll bias | -0.0005 to +0.0005 Nm |
| Total evaluation horizon | 200 s; no indefinite-operation claim |
| Sinusoidal load window | t=10 to 190 s, with 2 s raised-cosine ramps |
| Phase | Zero at load onset |
| Bias window | Entire run, including initial hold and recovery |
| Momentum allowance | At most 80% of worst-sign initial headroom to the 85-degree diagnostic guard |

All limits apply **together**; they are not independent maxima to mix with a
different frequency, duration, initial momentum, or actuator configuration.
The 0.006 Nm amplitude is the alternating component; the instantaneous total
load can reach 0.0065 Nm with bias. The bias is a constant test load, not an
arbitrary time-varying bias process.

The original criteria remain: peak roll error <=2 degrees, load-window RMS
<=1 degree, final-second error <=0.5 degrees, complete horizon, and no sampled
actuator-limit flags. The momentum reserve adds a planning constraint; it does
not relax a tracking criterion. Recovery here removes the sinusoid, **not** the
bias, so exact zero roll error and zero residual rotor momentum are not expected.

## Basis for selection

The existing PD controller and gimbal servo predict a worst steady oscillation
plus DC offset of 1.2048 degrees peak and 0.7902 degrees RMS over a 1001-point
linear frequency sweep. These are steady-state estimates, not onset/offset
guarantees; the nonlinear cases check the full trajectory.

For fixed spin speeds, define H0 from the initial rotor states and Hguard from
the symmetric branch at 85 degrees. The worst-sign usable increment is
0.8*(Hguard - abs(H0)). For this saved baseline the allowance is 0.152651 N m s.
A conservative planning estimate combines absolute bias impulse over 200 s,
the sinusoidal impulse excursion 2*A/omega, and predicted oscillatory body
momentum. It gives 0.131567 N m s. This estimate neglects nonlinear effects and
is not a general proof for arbitrary load envelopes; measured trajectory
excursions are also compared with the allowance.

The 20% reserve, amplitudes, and test duration are explicit engineering study
choices. They are not hardware ratings. Initial preload, spin speeds, mass
properties, feedback gains, pitch correction, and limits are inherited from the
saved zero-reference test; changing them requires regeneration of this screen.

## Screening and reproduction

`ROLL_DISTURBANCE_ENVELOPE.m` is the machine-readable specification.
Run `VERIFY_ROLL_DISTURBANCE_ENVELOPE` after generating the zero-reference MAT
file with `RUN_ROLL_DISTURBANCE_TESTS`.

The nonlinear matrix uses maximum amplitude at the four frequency/bias corners
(5 and 15 s, each with +/-0.0005 Nm bias), plus a 10 s zero-bias midpoint.
Results are saved under `Working Results/roll_disturbance_envelope`, including
the linear frequency screen, nonlinear summaries, and full case configurations.
The controller is not retuned and this envelope is not silently enabled in the
main driver. No unloading, propulsion, or sway/yaw thrust is enabled.

### Nonlinear results (2026-09-18)

All five cases completed 200 seconds and passed the unchanged tracking criteria
plus the added momentum-reserve check. Across the cases:

- Maximum peak roll error: 1.2055 degrees.
- Maximum load-window RMS error: 0.7889 degrees.
- Maximum final-second error with bias still applied: 0.09555 degrees.
- Maximum rotor momentum excursion: 0.11982 N m s, below the 0.152651 allowance.
- Maximum absolute gimbal angle: 46.21 degrees.
- Recorded actuator-limit flags: zero.

Both nonzero-bias signs left approximately +/-0.100 N m s of accumulated rotor
momentum after 200 seconds. This is expected storage of the persistent external
impulse, not a recovered/unloaded state. The zero-bias midpoint finished with
only 6.14e-8 N m s residual change. The remaining margin must not be interpreted
as permission to repeat biased missions indefinitely.

This is discrete nonlinear screening supported by a local frequency model.
It does not validate every intermediate frequency, arbitrary phase, shorter
window, disturbance mixture, mismatch, or hybrid maneuver. These are nominal
symmetric roll-hold tests, not a general six-DOF disturbance envelope.

## Persistent bias and the next tuning step

Any nonzero allowed bias is **time-budgeted**, not sustainable indefinitely.
The planning rule is bias magnitude times duration plus alternating-load/body
momentum excursions <= reserved headroom. Do not reset the budget after each
maneuver without checking the actual rotor momentum state. Bias remaining
after t=200 would continue accumulating momentum.

Use this specification as the nominal operating target and retain the original
0.01/0.03 Nm cases as outside-envelope challenge cases. If the manuscript needs
larger loads, explicitly revise the performance target, then investigate tracking
changes and storage feasibility separately. Adding integral feedback cannot
remove the need for an external roll reaction under sustained biased loading.
