# Combined roll, sway, and yaw maneuver envelope

Eleven finite maneuvers were evaluated over 18 seconds. The matrix includes
maneuver-plane angles from -90 to +90 degrees, heading changes in both
directions, positive and negative lateral translations, pure heading
endpoints, pure translation endpoints, and moderate simultaneous sway/yaw
commands.

Nine of eleven cases pass. Across the passing set, maximum plane-alignment
error while thrust is active is 0.635 degrees, maximum final heading error is
0.253 degrees, maximum final lateral-position error is 0.0121 m, and maximum
out-of-plane displacement is 0.0064 m. Peak module force is 3.150 N, below the
provisional 5 N limit. No passing case encounters thruster allocation
saturation, CMG angle/rate limiting, or a roll-direction infeasibility.

The horizontal-plane case completes a simultaneous 45-degree heading change
and 0.25 m translation. Pure +/-0.30 m translations also pass in oblique
+/-45-degree planes. Thus neither heading magnitude nor translation magnitude
alone explains the observed boundary.

The two failures combine an oblique +/-45-degree maneuver plane with a
+/-45-degree heading change. In these cases, gimbal travel reaches
approximately 88-91 degrees and the roll-alignment gate eventually disables
the thrusters. Final heading error remains approximately 30 degrees. No hard
mechanical limit or instantaneous roll-capacity violation is recorded; the
failure is a trajectory-level CMG momentum-envelope limitation associated
with maintaining the oblique plane during the large turn.

The result establishes a defensible current operating envelope rather than an
unqualified all-attitude capability. Moderate combined maneuvers are supported,
while large oblique heading changes require further momentum-envelope work,
such as preload optimization, staged plane reorientation, trajectory shaping,
or an external roll-momentum-management mechanism.
