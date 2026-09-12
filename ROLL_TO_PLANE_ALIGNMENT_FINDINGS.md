# Roll-to-plane alignment validation

Nine maneuver-plane commands were evaluated. Seven level-vehicle cases span
-90, -60, -30, 0, 30, 60, and 90 degrees. Two additional cases combine plane
commands with nonzero initial pitch and yaw to verify that the command
generator uses the full vehicle orientation correctly.

All nine cases pass the one-degree final-alignment requirement. The maximum
absolute final error is 0.791 degrees at the +90-degree command. Geometry-only
command error is at numerical precision, including the nonzero pitch/yaw
cases. This separates command-generation accuracy from the small residual
error of the finite-dynamics PD-controlled maneuver.

No case reaches the configured gimbal-rate or gimbal-angle limits, and no case
contains a roll-direction infeasibility. Across the sweep, peak gimbal rate is
1.791 rad/s versus the 20 rad/s limit, and peak gimbal acceleration is
188.5 rad/s^2 versus the 500 rad/s^2 limit. Positive and negative maneuvers
have closely matched errors and actuator demand.

These results validate vehicle reorientation only. They do not yet validate
force production or yaw/sway motion because the vortex-ring thruster dynamics
and allocation have not been added.
