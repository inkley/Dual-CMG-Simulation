# Primary aft-propulsion dynamics

## Implemented scope

The simulation now supports an optional forward-only axial net-thrust actuator.
Actual force is state 21; its derivative is a first-order lag with a bounded
force rate. Commands are clipped to [0,maxForce]. Disabling requests zero;
stored force decays rather than disappearing instantaneously. No surge-speed
controller, waypoint guidance, or automatic roll-turn-surge state machine is
included in this step.

Default screening assumptions from `AFT_PROPULSION_DEFAULTS`:

| Parameter | Value |
| --- | --- |
| Enabled | false |
| Initial actual force | 0 N |
| Direct command | 0 N |
| Maximum forward thrust | 10 N |
| Thrust time constant | 0.5 s |
| Maximum force rate | 10 N/s |
| Mount location | [-0.8,0,0] m in body coordinates |
| Force direction | positive body x |

The location is provisional, not measured IVER geometry. For centerline axial
thrust, r cross F is zero. Off-axis transverse offsets generate the appropriate
pitch/yaw mounting moments. This does **not** model propeller shaft-reaction
roll torque: zero shaft reaction is an explicit omission/idealization, not a
consequence of centerline placement. Propeller rotational inertia, motor torque,
advance ratio, inflow, wake interaction, efficiency, reverse thrust, and active
braking remain outside this low-order actuator. No new motor/housing mass is
invented or added to the vehicle. These limits are not hardware specifications.

## Integration and compatibility

- `AFT_PROPULSION.m` evaluates actual force, derivative, limits, generalized
  loading and vehicle mechanical transfer F*u.
- `CONTROL.m` appends the force derivative only for a 21-state run.
- `REMUS.m` accepts an optional actual-propulsion generalized force. Its old
  nine-argument calls remain valid.
- `TORQUE.m` records propulsion commands, force, force derivative, flags,
  generalized loading and vehicle mechanical power.
- `AUV_SIM.m` enables the optional state and plots its response. Enabled outputs
  go to an `aft_propulsion` child directory, not over the existing CMG baseline.
- Older 20-state runs with no propulsion configuration remain compatible.

Existing CMG energy reports remain CMG-only. F*u and its integral describe
power/work delivered to the vehicle through axial thrust, not total propeller
shaft power or electrical energy. In particular, F*u is zero at rest despite
nonzero thrust; this does not imply zero motor power at bollard conditions.

## Verification results

`VERIFY_AFT_PROPULSION` passed analytic exponential lag and piecewise slew/lag
tests, forward command clipping, zero-command shutdown, force-state bounds,
centerline and off-axis moment signs, mechanical power, and legacy-state checks.

`AFT_PROPULSION_ANALYSIS` applies 5 N from 0 to 40 s, then zero command until
60 s, with the vehicle initially at rest and no requested roll or lateral thrust.
It compares the full simulation with the independent reduced model:

`(m + 0.93) du/dt = F - 1.62 u |u|`.

The coefficients match the currently implemented REMUS surge added mass and
quadratic drag. This verifies implementation consistency, not experimental
validity of those hydrodynamic coefficients.

| Result | Value |
| --- | ---: |
| Predicted 5 N equilibrium speed | 1.7568 m/s |
| Speed at 40 s shutdown | 1.7530 m/s |
| Coasting speed at 60 s | 0.6551 m/s |
| Distance at 60 s | 76.2636 m |
| Vehicle thrust work over 60 s | 280.765 J |

Full and reduced surge/force trajectories agree within the 1e-6 test tolerance
(reported difference zero in this run). A 90-degree initial yaw rotates the
translation direction without changing body surge. Cross-axis states remain
zero in the aligned test. Twenty saved CMG states give identical first-20
derivatives with missing versus disabled propulsion, including an appended
zero force state. Existing CMG torque-sign, VRT and allocation regressions pass.
A complete disabled-propulsion replay of the saved CMG baseline differs by at
most 2.39e-8 across its first 20 state columns. The 21-state history reconstruction
also reproduces the actual force state exactly. MATLAB code checks report no
issues for the modified driver or the new actuator function.

Outputs: `Working Results/aft_propulsion_validation/validation.mat` and
`AFT_PROPULSION_RESPONSE.png`. Existing saved baselines were not overwritten.

## Use and next steps

For an isolated surge experiment, run `AFT_PROPULSION_ANALYSIS`.
To include propulsion in the main driver, set
`cmgConfig.propulsion.enabled=true` and `commandForce=5` in `AUV_SIM.m`.
That applies thrust immediately from startup; it is **not** a sequenced mission.
Use a zero direct-roll command if an isolated surge run is intended.

Completed checklist item: add primary aft-propulsion dynamics.

Next: add a bounded surge-speed controller and explicit turn-complete criteria
before enabling propulsion, then verify the complete roll-turn-surge sequence.
Keep gimbal hard-stop behavior and moving-reference/yaw-braking coordination
open. Shaft-reaction torque assumptions must also be assessed before claiming
fully physical propulsion/CMG roll coupling.
