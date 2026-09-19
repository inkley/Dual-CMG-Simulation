# Bounded roll-disturbance test setup

## Scope and status

The test harness, zero-load reference, and all eight nonzero-load vehicle
experiments have been run. These are synthetic
body-axis roll moments, not validated wave loads or a specified sea state.

The test starts at a 45-degree roll hold with zero vehicle velocity; it does not
simulate the maneuver into that attitude. It uses the saved symmetric dual-CMG
VFR configuration, installed vehicle properties, and finite actuator dynamics.
Roll feedback and the existing bounded plane-pitch correction are active in
feedback cases. Thrusters, aft propulsion, and external momentum unloading are
disabled. Passive cases disable both attitude feedback terms.

## Predeclared experiments and criteria

Nine cases: one zero-load feedback reference and four matched feedback/passive
pairs (peak moments 0.01 and 0.03 Nm; periods 5 and 15 seconds).

Each case runs for 80 seconds: 10 seconds unloaded, 60 seconds of load, and
10 seconds of recovery. Raised-cosine ramps of 2 seconds at both ends bound the
moment by its specified amplitude. The sinusoid starts at zero phase.
The load enters the plant only, without disturbance feedforward to the controller.

Feedback development-screen criteria, fixed before nonzero experiments:

- Complete the 80-second horizon without a terminal safety event.
- Peak absolute roll error no greater than 2 degrees over the entire run.
- Time-weighted RMS roll error no greater than 1 degree over the load window.
- Maximum absolute roll error over the final second no greater than 0.5 degrees.
- No actuator-limit flags at recorded samples.

Passive responses provide a comparison, not controller pass/fail evidence.
Safety events stop integration at configured gimbal-angle or flywheel-speed
bounds, or an 80-degree pitch Euler-coordinate guard. Incomplete runs fail the
screen; partial-window metrics must not be compared as complete-run results.
These thresholds are study design choices, not hardware or general robustness
guarantees. Sampling-based limit checks cannot exclude arbitrarily short events.

The runner also records peak pitch/yaw excursions and change in the body-x
component of rotor momentum (N m s). This momentum metric is not an inertial-frame
conservation balance. Inspect full histories for momentum accumulation, allocation
conditioning, and roll versus pitch-neutral feasibility before making broader
authority claims. Passing the tracking screen alone does not establish those claims.

## Verified reference

The initialized zero-load hold completed 80 seconds with zero recorded roll,
pitch, and yaw error, zero rotor-x momentum change, and no actuator-limit flags.
Waveform bounds, phase/sign, zero amplitude, and absence of unloading passed
automated checks. Existing CMG torque-sign verification also passed.
This equilibrium check does not establish disturbance rejection performance.

## Nonzero-load results (2026-09-18)

All nine cases completed 80 seconds without recorded actuator-limit flags.
The unchanged feedback tracking screen passed only one of four loaded cases:

| Amplitude (Nm) | Period (s) | Feedback peak error (deg) | Feedback load-window RMS (deg) | Screen |
|---|---|---|---|---|
| 0.01 | 5 | 1.2558 | 0.8581 | Pass |
| 0.01 | 15 | 1.8494 | 1.2990 | Fail: RMS |
| 0.03 | 5 | 3.7658 | 2.5739 | Fail: peak and RMS |
| 0.03 | 15 | 5.5524 | 3.8971 | Fail: peak and RMS |

All feedback cases recovered to less than 0.00022 degrees error over the final
second. Their peak rotor body-x momentum changes were respectively 0.01166,
0.04745, 0.03498, and 0.14235 N m s. These are finite-horizon excursions, not
evidence of unlimited sustained authority or momentum unloading.
Peak gimbal travel across feedback cases was 53.99 degrees. Final rotor body-x
momentum change was below 2.60e-6 N m s. These particular windowed sinusoids have
essentially zero net external roll impulse; they do not test a persistent bias
load. Applied-load replay, amplitude bounds, absence of disturbance feedforward,
and disabled external unloading all passed automated checks.

The no-attitude-feedback references reached unwrapped peak roll errors of
43.23, 398.89, 126.66, and 1109.1 degrees, respectively. Values above 360 degrees
represent accumulated rotation, not a wrapped orientation error. Thus the
feedback substantially reduces roll motion, but that is distinct from meeting
the predeclared tracking tolerances. No gain tuning or acceptance changes were
made. Pitch and yaw remained zero under these exactly symmetric roll-only tests;
this does not establish cross-axis robustness with mismatch or hybrid motion.

Next: evaluate tracking bandwidth and momentum excursions over a broader,
explicitly bounded disturbance envelope before claiming roll-disturbance
rejection. The current three screen failures are not actuator saturation failures.

## Reproduction

Run from the Simulation directory in MATLAB. The runner requires the saved
`Working Results/dual/symmetric_spin/VFR/simulation_result.mat` baseline.
Each output MAT file embeds that baseline, the test configuration, test plan,
state trajectory, and replayed diagnostics. Confirm the baseline provenance
before using results in publication comparisons.

```matlab
RUN_ROLL_DISTURBANCE_TESTS;       % zero-load reference only
VERIFY_ROLL_DISTURBANCE_SETUP;
VERIFY_CMG_TORQUE_SIGNS;
RUN_ROLL_DISTURBANCE_TESTS(true); % all nine cases
REPORT_ROLL_DISTURBANCE_TESTS;   % applied-load checks, diagnostics, comparison PNG
```

Outputs are under `Working Results/roll_disturbance_tests`. Re-running overwrites
matching case outputs. Solver tolerances are RelTol=1e-8, AbsTol=1e-10, with
0.01-second maximum step and recording interval. Repeat limiting cases at finer
resolution before drawing conclusions about brief constraint violations.
