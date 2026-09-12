# Fixed vortex-ring thruster model

Two fixed lateral vortex-ring thruster modules are represented at provisional
body-axis positions x = +0.65 m (fore) and x = -0.65 m (aft). Each module
produces force along body y. Equal forces therefore generate sway, while
equal-and-opposite forces generate yaw. With both force lines in the body
x-y center plane, the pair generates no direct roll or pitch moment.

Each achieved thruster force is a dynamic state. The present low-order model
uses a 0.15-second first-order force response, a provisional 5 N force bound,
and a provisional 50 N/s force-rate bound per module. These are model-screening
assumptions rather than measured vortex-ring thruster ratings. The model is
cycle averaged and does not resolve vortex circulation, pulse formation,
fluid-memory effects, or vehicle-vortex interaction.

Force/moment unit checks confirm that +2/+2 N produces Y = +4 N and N = 0,
while +2/-2 N produces Y = 0 and N = +2.6 N m. A four-second open-loop
integration using +1/+1 N produces 0.194 m positive sway and zero yaw. The
+1/-1 N case produces 48.432 degrees positive yaw and zero sway. These cases
verify signs, geometry, and dynamic integration; the large yaw excursion is
not a tracking result because closed-loop thruster allocation has not yet
been implemented.

The fixed sway/yaw pair cannot unload CMG roll momentum in this center-plane
configuration. Doing so would require a roll-capable external actuator or a
different force-line geometry with a vertical/radial moment arm.
