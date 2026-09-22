# Installed CMG mass-property assumptions

Purpose: include the mass properties already defined by the simulation without
turning the modeling study into a premature hardware design.

- Base vehicle: 30.479 kg with inertia diag(0.177, 3.45, 3.45) kg m^2.
- Included CMG mass: modeled steel rotor only, approximately 0.798 kg each.
- Dual placement: x = -0.30 m and +0.30 m; y = z = 0.
- Single placement: vehicle origin.
- Rotor centroidal tensor: axisymmetric solid-cylinder inertia evaluated at
  the initial gimbal angle.
- Assembly: combined CG plus full parallel-axis theorem about the combined CG.
- Hydrodynamic coefficients: unchanged because the external hull is unchanged.

The symmetric dual placement preserves zero CG offset and cancels inertia
products. An asymmetric installation that produces nonzero products of inertia
is rejected by the current code because REMUS presently assumes a diagonal
inertia tensor.

These assumptions support a simulation paper's actuator-dynamics comparison.
They do not claim a completed mechanical design. Future experimental work can
replace the module masses, centroidal tensors, and mount locations through the
installation configuration and assembly function.
