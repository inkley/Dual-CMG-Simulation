# Fixed-thruster force and moment allocation

The allocator maps requested body sway force and yaw moment to fore and aft
module forces using

    [Y; N] = [1, 1; x_fore, x_aft] [F_fore; F_aft].

For the provisional symmetric positions x = +/-0.65 m, common-mode force
produces sway and differential force produces yaw. The allocation matrix has
condition number 1.538 and is nonsingular. Feasible common, differential, and
combined requests are reproduced to machine precision.

If either unconstrained module force exceeds the provisional 5 N bound, both
commands are multiplied by one common scale factor. This direction-preserving
saturation retains the requested Y:N ratio. For example, a requested 20 N
sway force is scaled to the available 10 N without generating a yaw moment.
With the current assumptions, the static envelope reaches +/-10 N sway at
zero yaw and +/-6.5 N m yaw at zero sway; combined commands occupy the diamond
between those extrema.

The allocator output drives the finite 0.15-second thruster force dynamics.
The saved histories distinguish requested generalized force/moment,
force-limited allocation, and dynamically achieved output. This step does not
yet provide feedback control: desired Y and N commands must next be generated
from sway/yaw tracking errors and sequenced after roll-to-plane alignment.
