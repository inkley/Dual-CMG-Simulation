# CMG actuator-limit assumptions

Status: provisional design-screening bounds for the single-CMG simulation.
These values are not vendor ratings or experimentally validated limits.

| Quantity | Model bound | Basis | Tuned-run peak | Utilization |
|---|---:|---|---:|---:|
| Gimbal angle | +/-100 deg | Rounded requirement above the 93.7 deg margin-adjusted demand | 78.1 deg | 78.1% |
| Gimbal rate | 20 rad/s | Existing simulation screening bound | 14.568 rad/s | 72.8% |
| Flywheel speed | 1800 rpm | Rounded value providing about 20% headroom above the observed peak | 1456.6 rpm | 80.9% |
| Flywheel acceleration | 500 rad/s^2 | Existing simulation screening bound | 295.112 rad/s^2 | 59.0% |

Before hardware claims or experimental implementation, replace these bounds
with measured or manufacturer-supported continuous and transient ratings for
the selected gimbal motor, rotor motor, bearings, drive electronics, and
mechanical stops. Also check torque-speed curves, acceleration duration,
thermal duty cycle, rotor stress, balance, and pressure-housing constraints.

The current result establishes a component requirement, not component
availability: a candidate assembly must provide at least the trajectory peaks
listed above, and should retain an explicit engineering margin after its
continuous/transient ratings and uncertainty are known.

## Representative hardware screen

The modeled flywheel has an axial inertia of approximately 0.00103 kg m^2.
The tuned trajectory therefore requires approximately 0.303 N m peak motor
torque; speed and mechanical-power requirements are calculated on every run by
`CMG_HARDWARE_SCREENING.m`. With a 20% screening margin, the trajectory can be
compared with the maxon EC-i 52 200 W, 24 V motor (part 606793) as a
representative—not selected—flywheel drive. Its published nominal values are
2970 rpm and 0.646 N m, with a maximum speed of 5000 rpm.

The model now includes a gimbal-rate state, a 0.01 s preliminary rate-servo
time constant, and a 500 rad/s^2 acceleration bound. Preliminary gimbal-axis
inertia includes the analytically calculated transverse inertia of the rotor,
the published motor-rotor inertia, and a 25% allowance for the gimbal frame.
The maxon EC-i 52 200 W, 24 V motor (part 606793) is used only as a
representative direct-drive comparison. Its published nominal values include
2970 rpm and 0.646 N m, with a maximum speed of 5000 rpm.

This is a component-level feasibility screen rather than final hardware
validation. The frame allowance, servo response, bearing/seal friction,
pressure effects, drive current, and thermal duty cycle require refinement or
measurement for a selected mechanical design.

With these dynamics enabled, the tuned roll controller reaches the 500
rad/s^2 gimbal-acceleration bound during its initial rate transient. The
representative motor retains torque margin at that bound, but the vehicle-side
moment is temporarily below the allocator request and a small pitch moment is
no longer canceled exactly. This behavior must be retained in reported model
results; the earlier statement that the run has no actuator saturation applies
only to the ideal-rate model.

## Momentum-accounting scope

`VERIFY_CMG_MOMENTUM_EXCHANGE.m` verifies the exact vector identity
`tau_vehicle = -dH_rotor/dt` for randomized CMG states and commands.
`CMG_MOMENTUM_COUPLING_ANALYSIS.m` separately checks the simulated roll
trajectory, including the angular-momentum impulse exchanged with the water
through modeled roll drag. This avoids describing the hydrodynamic vehicle as
a closed system.

The single-CMG allocator controls roll and pitch moments. It cancels applied
CMG pitch moment to numerical precision, but the CMG yaw reaction
`N = I*Omega*(cos(alpha)*p + sin(alpha)*q)` remains. Once yaw rate develops,
the diagonal rigid-body dynamics introduce the pitch forcing
`(Iz-Ix)*p*r`. Therefore, the small pitch response is a physical cross-axis
effect of the modeled single-CMG geometry, not pitch-allocation error.
