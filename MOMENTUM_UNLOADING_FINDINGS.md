# Momentum-unloading controller

CMGs are internal actuators and cannot remove net angular momentum from the
vehicle-CMG system alone. The implemented momentum manager therefore uses an
explicit bounded external roll-moment channel. The CMGs receive an unloading
moment proportional to rotor Hx error, while the external actuator receives
the equal-and-opposite moment. Their commanded pair cancels exactly at the
vehicle level while driving rotor momentum toward its reference.

The current design uses a momentum gain of 0.5 per second and a maximum
external unloading moment of 0.05 N m. This external channel is an abstract
actuator for the modeling study. The fixed center-plane sway/yaw thruster pair
does not generate body roll moment and therefore cannot realize this channel.
Momentum unloading would require a roll-capable control surface, a thruster
force line with a radial moment arm, or another actuator that reacts against
the water.

## Sustained disturbance

A constant 0.02 N m roll disturbance was applied for 15 seconds while holding
zero roll. Without unloading, rotor Hx error grows to 0.1918 N m s, the
gimbals reach approximately 90.5 degrees, roll authority collapses, and final
roll error reaches -97.6 degrees. This is momentum saturation even though the
mechanical +/-100-degree stop has not yet been crossed.

With unloading enabled, rotor Hx error converges to 0.0400 N m s, exactly the
expected disturbance/gain equilibrium. The external unloading moment
converges to -0.0200 N m and prevents momentum saturation; maximum gimbal angle
is 24.4 degrees, with zero roll-infeasible samples. The unmanaged case records
9840 roll-infeasible samples after its momentum envelope is exhausted. The
remaining -3.82-degree attitude offset is the expected
steady error of the PD roll controller under constant disturbance. Integral
or disturbance-feedforward control is required to remove that offset.

## Repeated positive rolls

Five consecutive positive 90-degree increments were tested over 30 seconds.
Both managed and unmanaged cases track the attitude sequence. Unloading
reduces maximum rotor-momentum error from 0.1607 to 0.1258 N m s, a reduction
of approximately 21.7%, without rate or angle limiting. The managed momentum
trajectory is shifted by the bounded external counter-moment and ends with
0.0150 N m s error versus -0.00539 N m s unmanaged; peak envelope usage, not
the sign of the final sample, is the relevant improvement for this sequence.

The result demonstrates successful momentum management but not a finalized
thruster design. It remains a separate roll-momentum-management requirement
unless the installed thruster geometry is revised to provide roll authority.

## Singularity-avoidance decision

No separate steering-avoidance law is required for the current roll-only
baseline. The dual-CMG path crosses a formal singularity of the full two-axis
K/M allocation, but the unused direction is pitch. The commanded roll
direction remains feasible, with no roll-capacity violations in the baseline,
repeated-roll, or managed sustained-disturbance cases. The unmanaged sustained
disturbance does lose roll authority, showing that momentum unloading is the
required remedy for this failure mode. The diagnostics now
distinguish this benign full-matrix rank loss from a task-direction
singularity. If a future simulation requests simultaneous CMG pitch moment or
reports a roll-capacity violation, task-priority steering, command limiting,
or a geometry-changing null-motion law must be added at that point.
