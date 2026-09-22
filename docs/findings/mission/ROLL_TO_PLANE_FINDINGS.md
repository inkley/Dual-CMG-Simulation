# Roll-to-plane command generator

`ROLL_TO_PLANE_COMMAND` converts a desired inertial lateral maneuver direction
into the roll attitude required to align the fixed body-y thruster axis with
that direction. It uses the same 3-2-1 Euler-angle and north-east-down
conventions as the REMUS model and accounts for the vehicle's current yaw and
pitch.

For reversible thrusters, roll attitudes separated by 180 degrees represent
the same maneuver plane. The generator selects the equivalent attitude nearest
the supplied reference roll and returns the required thrust polarity. This
avoids unnecessary full or half rotations while preserving the desired force
direction. A deterministic tie favors the positive-thrust solution.

The generator projects any requested vector onto the vehicle transverse plane.
Its `outOfPlaneComponent` reports the unattainable longitudinal component, and
`requestedDirectionResidualAngle` reports the corresponding directional
residual. A direction parallel to the vehicle longitudinal axis is rejected
because roll cannot define that maneuver plane.

The default validation requests inertial down from a level, north-facing
vehicle. The generator returns a +90-degree roll command, positive thruster
polarity, and zero geometric alignment error. The complete dual-CMG simulation
reproduces the established baseline with a final dynamic roll error of
-0.791 degrees; this dynamic tracking error is distinct from command-generator
geometry error.
