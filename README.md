# CMG AUV Simulation

MATLAB research model for single- and dual-control-moment-gyroscope (CMG)
roll actuation of an autonomous underwater vehicle. The hybrid-control concept
uses CMGs to orient the vehicle's maneuver plane, then fixed lateral thrusters
to generate sway and yaw. Aft-propulsion travel is a planned extension.

Author: Tyler J. Inkley. This is ongoing modeling and simulation research
intended to support an IEEE Journal of Oceanic Engineering manuscript; it is
not a hardware-validated controller or an accepted publication.

## Requirements

- MATLAB. Development runs have used MATLAB R2025b; older releases have not
  been validated. No Simulink model is required.
- Open this repository as MATLAB's current folder so its functions are on
  the MATLAB path.
- For figure-window compatibility, set `plotConfig.windowStyle = 'normal'`
  in `AUV_SIM.m` if the container/docking option causes problems.

## Quick start

1. Open `AUV_SIM.m` and review its **USER CONFIGURATION** section.
2. Select `cmgConfig.mode = 'single'` or `'dual'`. The current default is dual
   with the `constant_speed` controller.
3. Keep `simConfig.case = 'VFR'` for the vehicle-from-rest baseline.
4. Run the driver:

   ```matlab
   AUV_SIM
   ```

The driver clears workspace variables and closes figures at startup. Set
configuration values **in the script**, not in the command window beforehand.
It saves figures, diagnostics, and `simulation_result.mat` beneath
`Working Results/`, creating the directory as needed. Repeating a configuration
overwrites its previous outputs; preserve important runs separately.

The default command mode is `roll_to_plane`, with a downward desired lateral
direction. For a direct 90-degree roll command, set:

```matlab
simConfig.commandMode = 'direct_roll';
simConfig.directRollAngle = pi/2;
```

Other initial-condition cases are `FSV` (forward surge velocity), `STM`
(standard turn maneuver), and `SPF` (simple path following). These labels do
not imply that complete mission-level behavior has been validated.

## Optional aft propulsion

The default CMG baseline keeps propulsion disabled. To include a constant
forward thrust, edit these settings in `AUV_SIM.m`:

```matlab
cmgConfig.propulsion.enabled = true;
cmgConfig.propulsion.commandForce = 5; % N; direct thrust, not speed control
```

Enabled runs add state 21 (actual aft thrust) and save beneath the case's
`aft_propulsion/` subfolder, preserving the CMG-only baseline output path.
The model uses forward-only net thrust, finite lag and slew, and a mounting
moment computed as r cross F. The default centerline mounting adds surge only;
shaft-reaction torque and propeller inflow/RPM physics are not represented.
`AFT_PROPULSION_DEFAULTS.m` holds provisional settings, not hardware ratings.
Run `VERIFY_AFT_PROPULSION` and `AFT_PROPULSION_ANALYSIS` for actuator tests and
isolated surge/shutdown validation. See `AFT_PROPULSION_FINDINGS.md`.
This does not yet implement speed regulation or roll-turn-surge sequencing.

## Complete roll-turn-surge demonstration

Run `RUN_ROLL_TURN_SURGE` for a from-rest roll, shaped in-plane turn, and
speed-controlled cruise segment. This is a separate mission entry point;
`AUV_SIM.m` remains the baseline driver. The mission reads the saved symmetric
dual VFR baseline and does not overwrite it. Its supervisor requires sustained
capture before turning and before enabling aft thrust. Lateral control switches
to a cross-track path during surge so it does not oppose oblique forward travel.

```matlab
RUN_ROLL_TURN_SURGE
RUN_ROLL_TURN_SURGE(struct('planeDeg',-45,'headingDeg',-45))
```

`ROLL_TURN_SURGE_DEFAULTS.m` defines the finite mission and controller settings.
See `ROLL_TURN_SURGE_FINDINGS.md` for criteria, results and limitations.
`COMPLETE` means the cruise observation interval ended, not that the vehicle
stopped or reached a waypoint. Existing gimbal-stop and propeller shaft-torque
limitations remain. No automatic return to nominal roll is attempted.

## Single and dual comparisons

Run `AUV_SIM` once in single mode and once in symmetric dual mode, using
consistent case, command, duration, and installed-mass assumptions. Then run:

```matlab
COMPARE_CMG_BASELINES
```

This analysis loads the saved VFR baselines; it does not regenerate them.
Single and dual configurations have different installed rotor masses, so a
fair comparison means consistent mass-accounting rules, not identical masses.
Generated results are intentionally excluded from Git and must be produced
locally after cloning.

## Model organization

| File | Role |
| --- | --- |
| `AUV_SIM.m` | Configuration, initialization, integration, diagnostics, plots, and output |
| `REMUS.m` | Vehicle dynamics and state derivatives |
| `CONTROL.m`, `TORQUE.m` | Control requests and applied force/moment calculations |
| `CMG.m`, `CMG_ALLOCATE.m` | CMG reaction moments and actuator allocation |
| `ASSEMBLE_VEHICLE_MASS_PROPERTIES.m` | Installed rotor mass and inertia accounting |
| `ROLL_TO_PLANE_COMMAND.m`, `ROLL_TO_PLANE_ALIGNMENT.m` | Plane command generation and alignment metrics |
| `VORTEX_RING_THRUSTERS.m`, `THRUSTER_ALLOCATE.m` | Cycle-averaged thruster dynamics and force allocation |
| `HYBRID_MANEUVER_CONTROL.m` | Closed-loop lateral/heading requests and thrust activation |

## Verification and analysis

The `VERIFY_*.m` functions check specific identities or components, including
CMG torque signs, momentum exchange, mass assembly, steering conditioning,
plane commands, and thruster allocation. For example:

```matlab
VERIFY_CMG_TORQUE_SIGNS
VERIFY_CMG_MOMENTUM_EXCHANGE
VERIFY_MASS_PROPERTY_ASSEMBLY
VERIFY_DUAL_CMG_CONDITIONING
VERIFY_ROLL_TO_PLANE_COMMAND
VERIFY_VORTEX_RING_THRUSTERS
VERIFY_THRUSTER_ALLOCATION
```

After generating a symmetric dual VFR baseline, useful scenario scripts include:

```matlab
HYBRID_COORDINATED_MANEUVER_ANALYSIS
HYBRID_MANEUVER_SWEEP
ROLL_TO_PLANE_ALIGNMENT_SWEEP
DUAL_CMG_MISMATCH_SWEEP
REPEATED_ROLL_MOMENTUM_ANALYSIS
MOMENTUM_UNLOADING_ANALYSIS
OBLIQUE_CMG_MOMENTUM_ENVELOPE
OBLIQUE_PRELOAD_SHAPING_SWEEP
```

Read each script's configuration before running. Scenario scripts generally
load the saved baseline and apply their own commands or parameter changes;
results therefore depend on both the saved baseline and the analysis script.
The oblique-envelope characterization saves full histories, momentum maps,
rate-based roll-authority intervals, and a half-step sensitivity check. See
`OBLIQUE_CMG_MOMENTUM_FINDINGS.md` for interpretation and limitations.
The preload/shaping sweep compares fixed initial gimbal preloads with smooth
in-plane heading references, without changing the production controllers.
See `OBLIQUE_PRELOAD_SHAPING_FINDINGS.md`. Its candidate refinement command is
`OBLIQUE_PRELOAD_SHAPING_SWEEP([10,20],.005)` followed by
`VERIFY_OBLIQUE_SHAPING_RESULTS`.
`*_ANALYSIS.m` also includes callable diagnostic helpers, so not every file
with that suffix is a standalone script.

## Scope and limitations

- The model includes full vehicle pose/velocity states, finite gimbal response,
  actuator limits, and cross-axis coupling. Hydrostatic restoring loads are
  disabled in the current vehicle model.
- Installed CMG mass accounting includes modeled rotors, not a fully designed
  motor, frame, bearing, or pressure-housing assembly.
- Current screening bounds are +/-100 degrees gimbal angle, 20 rad/s gimbal
  rate, 500 rad/s^2 gimbal acceleration, 1800 rpm flywheel speed, and
  500 rad/s^2 flywheel acceleration. These are assumptions, not hardware ratings.
- The nominal dual configuration uses -15/+15 degree initial gimbal angles and
  counter-rotating flywheels at -1200/+1200 rpm. Feasible roll control does not
  establish arbitrary simultaneous roll/pitch authority or unlimited momentum
  capacity near steering singularities.
- Thruster modules provide signed, cycle-averaged body-y forces at provisional
  fore/aft locations. Vortex formation and pulse hydrodynamics are not resolved;
  the physical realization of bidirectional thrust remains unspecified.
- The center-plane thrusters provide no direct roll torque. Momentum-unloading
  studies use an abstract external roll-torque source, not these thrusters.
- Energy accounting describes ideal mechanical work and stored rotor energy,
  not measured electrical consumption or demonstrated electrical efficiency.
- Large oblique hybrid turns remain unresolved. Wave rejection, sensor-based
  feedforward, station keeping, and a full roll-turn-travel mission are not
  established by the current baseline.

The accompanying `*_FINDINGS.md`, `*_RESULTS.md`, and assumption notes record
development history. Numerical claims may refer to older configurations;
regenerate and check results before using them in a manuscript.

## Development and reproducibility

Use Git commits to record meaningful model changes rather than suffixing copies
of every script. For results intended for publication, record the commit ID,
MATLAB release, configuration, solver settings, and acceptance criteria with
the saved outputs. Keep generated figures and MAT files outside version history.
Refresh single/dual comparisons and mismatch sweeps after baseline changes.

No open-source license is granted by this repository. Select an appropriate
license after confirming research ownership and any third-party code terms.
