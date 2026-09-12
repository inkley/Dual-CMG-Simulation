# Dual-CMG baseline

Configuration: vehicle from rest, 90-degree roll command, unchanged vehicle
PD gains, two constant-speed counter-rotating flywheels, and finite gimbal
rate/acceleration dynamics.

Selected initial state:

- Gimbal angles: alpha1 = -15 degrees, alpha2 = +15 degrees.
- Flywheel speeds: Omega1 = -1200 rpm, Omega2 = +1200 rpm.

The smaller symmetric preload provides sufficient rotor-momentum range for
both positive and negative 90-degree rolls. The 1200 rpm speed also retains
20% margin below the provisional 1800 rpm flywheel-speed bound.

The prior 300 rpm trial was rejected. With [+45,-45] degree angles it steered
toward the +/-90-degree roll null and stalled near 26 degrees of vehicle roll.
Mirroring the angles improved the initial steering direction, but 300 rpm
still exhausted the usable momentum envelope and overshot to approximately
169 degrees. These failures demonstrate that initial torque capacity alone is
not sufficient; the constant-speed pair must store enough angular momentum
for the complete maneuver.

The accepted 1200 rpm baseline settles in 2.881 s with 1.505% overshoot and a
final error of -0.791 degrees. Peak gimbal rate is 1.752 rad/s per module and
peak gimbal acceleration is approximately 188.5 rad/s^2. There are no gimbal
rate, gimbal acceleration, flywheel acceleration, angle, or speed limit events.
The trajectory passes through a formal two-axis rank deficiency, but the
requested roll direction remains feasible with at least 10.61 times capacity.

Symmetry cancels the modeled pitch and yaw CMG reactions to numerical
precision. This is a major advantage over the single-CMG baseline. The cost is
two continuously spinning rotors and substantially greater initially stored
rotor energy; the reported ideal 100% maneuver-transfer metric must not be
interpreted as electrical efficiency or as including spin-up losses.

## Controlled comparison with the single CMG

Both simulations use the same roll command, feedback gains, duration, solver
tolerances, and actuator-dynamics model. The current installed-mass dual case
settles in 2.881 s and has similar overshoot to the earlier single baseline.
Its peak gimbal rate is 1.752 rad/s, and per-module gimbal excursion is 35.3
degrees. Symmetry
reduces pitch and yaw motion from 1.062 and 0.317 degrees to numerical zero.

The dual roll-moment RMSE is slightly higher (0.0592 versus 0.0510 N m) because
both finite-bandwidth gimbal servos lag the allocator command. This does not
materially degrade the attitude response.

The ideal maneuver-only actuator proxy is 0.135 J for the constant-speed dual
pair versus 23.038 J for the variable-speed single CMG. Including idealized
initial spin-up changes these totals to 16.394 and 23.546 J, respectively.
These are mechanical bookkeeping proxies, not battery-energy or electrical
efficiency estimates; motor, drive, bearing, seal, and hotel losses are absent.

This is an actuator-level controlled comparison. Both cases presently use the
same assembled-vehicle mass and inertia. A final system-level comparison must
update vehicle mass, center of gravity, and inertia for the installed single-
and dual-CMG hardware configurations. It must also compare package volume,
electrical power, thermal load, and reliability.

## Conditioning and singularity assessment

For the two-column steering matrix B, the determinant is proportional to
`h1*h2*sin(alpha2-alpha1)`. A formal singularity therefore occurs if either
flywheel momentum is zero or if the two steering columns become parallel or
antiparallel. Orthogonal equal-momentum columns have condition number one.

Across the accepted maneuver, the steering columns briefly align and the full
K/M matrix becomes rank deficient. This is not a roll-direction singularity:
the remaining column space is the requested roll direction, pitch demand is
zero, and the minimum active pitch-neutral roll-capacity margin is 10.61 times
the requested moment. No saved sample is directionally rate-infeasible.

The maneuver is therefore comfortably roll-rate feasible but not capable of
arbitrary simultaneous K/M control at the alignment crossing. This distinction
must remain explicit when reporting the roll-only actuator role.

## Installed rotor mass properties

The mass-property assembly now treats the Prestero-based vehicle values as the
unmodified vehicle and adds the mass and centroidal inertia of the two modeled
steel rotors. Each rotor has a calculated mass of approximately 0.798 kg. The
dual modules are placed symmetrically at x = +/-0.30 m on the vehicle
centerline. This keeps the combined CG at the original body-frame origin.

The update changes total mass from 30.479 to 32.075 kg, roll inertia from
0.177000 to 0.178119 kg m^2, pitch inertia from 3.450000 to 3.595625 kg m^2,
and yaw inertia from 3.450000 to 3.594684 kg m^2. These correspond to increases
of 5.24%, 0.63%, 4.22%, and 4.19%, respectively. Longitudinal displacement does
not add a parallel-axis term to roll inertia; it adds to pitch and yaw inertia.

With these properties, the dual baseline settles in 2.873 s with 1.541%
overshoot and -0.803 degrees final error. No actuator or singularity limits are
encountered, and ideal symmetry still cancels pitch and yaw motion.

The rotor inertia tensor is evaluated at the initial gimbal geometry and held
constant in REMUS. For the present rotors, the complete gimbal-dependent range
of local roll inertia is less than 0.6% of vehicle roll inertia, so a
time-varying vehicle inertia tensor is not justified for this reduced-order
study. Motor, frame, electronics, and housing masses remain excluded until a
future hardware concept supplies defensible values.
