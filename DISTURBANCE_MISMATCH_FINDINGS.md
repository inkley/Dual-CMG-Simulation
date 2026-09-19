# Disturbance rejection with rotor and gimbal mismatch

## Test definition

Run `DISTURBANCE_MISMATCH_SWEEP` after the zero-load reference exists. Results
and per-case histories are under `Working Results/roll_disturbance_envelope`.
This reuses the existing controller, provisional envelope, and roll/momentum
screen; no gains or acceptance criteria are tuned to the mismatch results.

Six configurations each receive a 0.006 Nm sinusoid of period 15 s, with
constant bias -0.0005 or +0.0005 Nm. Each test lasts 200 s, with the sinusoid
active from 10 to 190 s and bias active throughout. The period is the most
demanding nominal endpoint previously observed, not a proven worst case for
every uncertain system.

Configurations:

- Nominal repeat.
- Opposite rotor-speed measurement scale biases of +5%/-5% (true speeds unchanged).
- True rotor spin inertias +10%/-10%, with nominal inertias retained by the allocator.
- Plant gimbal-servo time constants scaled 0.8/1.2 and acceleration ceiling scaled 0.8.
- Combined speed bias, inertia mismatch, and servo mismatch.
- Combined case with rotor assignments reversed (same reduced acceleration ceiling).

These are deterministic screening assumptions, not sensor or motor ratings or
statistical uncertainty bounds. Rotor-inertia changes isolate internal momentum
model error at fixed installed mass and vehicle inertia, not resized physical
rotors. Actual spin-speed mismatch is not independently varied in this sweep.
Thrusters and propulsion remain disabled; their uncertainty is therefore not
tested. Angle and vehicle-state sensing remain ideal.

## Interpretation and criteria

Retain peak roll error <=2 degrees, load-window RMS <=1 degree, final-second
error <=0.5 degrees, complete horizon, and zero sampled actuator-limit flags.
The aggregate rotor-x momentum excursion must also remain below the existing
80%-of-headroom planning allowance, recalculated using true rotor inertias.
The 85-degree gimbal diagnostic guard and physical speed guard remain enabled.

Under asymmetry, aggregate Hx headroom is a necessary planning check, not proof
of simultaneous roll/pitch feasibility or an inertial-frame conservation law.
Individual gimbal histories and true spin-speed extrema are retained. Peak pitch
and yaw are reported separately with no retroactively chosen cross-axis pass
threshold. A `pass` in the CSV means this roll/momentum screen only, not a
combined-maneuver, station-keeping, or all-axis attitude pass.

Both bias signs still consume momentum over time. This finite-duration screen
does not authorize indefinite operation or repeated biased missions from an
assumed fresh initial momentum state.

## Results (2026-09-18)

All twelve runs completed 200 seconds with zero recorded actuator-limit flags.
Eleven passed the roll/momentum screen (including two nominal repeats); nine of
ten nonnominal cases passed. No gains or criteria were changed.

| Configuration | Negative bias | Positive bias |
|---|---|---|
| Nominal repeat | Pass | Pass |
| Speed-measurement bias | Pass | Pass |
| Unequal rotor inertia | **Fail: peak roll error** | Pass |
| Unequal gimbal servo | Pass | Pass |
| Combined | Pass | Pass |
| Combined, rotor assignments reversed | Pass | Pass |

The unequal-inertia/negative-bias case reached 2.039360 degrees error at
179.16 s, exceeding the unchanged 2-degree limit. Its RMS error was 0.81143
degrees and final-second error 0.12176 degrees, both within their limits.
Momentum excursion was 0.100 N m s, below the planning allowance, and its
maximum gimbal angle was 39.11 degrees. Thus it is not a sampled actuator-limit
or aggregate momentum-budget failure.

An independent half-step integration (0.005 s maximum step, common 0.01 s
output grid) reproduced the failure. Maximum common-grid roll difference was
1.06e-9 degrees. This rules out step sensitivity at the tested resolutions,
not every possible modeling or allocation issue. Saved refinement results and
`REFINE_DISTURBANCE_INERTIA_FAILURE` reproduce this check.

Across all cases, maximum pitch/yaw excursions were 0.06824/0.05427 degrees;
maximum gimbal angle was 48.52 degrees; peak rotor-x excursion was 0.11982 N m s.
True flywheel speeds remained at 1200 rpm in magnitude. Constant bias still
left approximately +/-0.100 N m s of stored momentum after the run.

The combined perturbations did not monotonically worsen tracking; the passing
combined cases do not supersede the isolated-inertia failure. The nominal
envelope therefore cannot yet be described as robust to the specified mismatch.

## Next action

Inspect the late-cycle unequal-inertia transient (requested/achieved roll and
pitch moments, steering conditioning, and gimbal trajectory) before changing
gains or shrinking the declared envelope. Keep the 2-degree criterion and the
failure visible. Thruster mismatch, arbitrary phase/frequency, and disturbed
hybrid missions remain outside this screening result.

Run `VERIFY_DISTURBANCE_MISMATCH_RESULTS` to check stored configurations,
nominal inertia retained by the allocator, fixed gains, plant uncertainty,
load replay, and agreement of the nominal repeat with the previous envelope.
