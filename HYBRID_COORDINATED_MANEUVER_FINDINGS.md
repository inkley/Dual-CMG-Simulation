# Coordinated CMG roll and sway/yaw thrust

The hybrid controller continuously updates the CMG roll command as vehicle
pitch and yaw change, preserving alignment between the fixed body-y thruster
axis and the selected inertial maneuver plane. Thruster activation is smoothly
gated by both roll-angle error and roll rate. This prevents sway/yaw thrust
from beginning during the initial roll transient.

After plane capture, a closed-loop body sway-force request regulates inertial
displacement along the selected lateral direction. A closed-loop yaw-moment
request aligns the vehicle longitudinal axis with a desired heading vector
inside the maneuver plane. The heading error is geometric rather than an Euler
yaw-angle error, which is necessary when the maneuver plane is rolled away
from horizontal.

The validation maneuver uses a 45-degree east/down plane, a 0.25 m lateral
displacement, and a 30-degree heading change within that plane. Thrusters reach
95% activation at 3.366 seconds. Final plane-alignment error is 0.006 degrees,
final heading error is -0.418 degrees, and final lateral displacement is
0.245 m, leaving 0.0048 m error. Peak module force is 2.708 N. No thruster
allocation saturation, thruster force-rate limiting, CMG rate/angle limiting,
or roll-direction infeasibility occurs.

This is a coordinated low-speed maneuver demonstration using provisional
cycle-averaged thruster dynamics. It does not yet include primary propulsion,
wave disturbances, station keeping, or measured vortex-ring actuator data.
