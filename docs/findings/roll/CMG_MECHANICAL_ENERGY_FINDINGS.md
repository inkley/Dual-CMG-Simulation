# Single/dual mechanical-energy accounting audit

## Conclusion

The existing ideal input proxy is not independently calculated motor-port
input. Its ratio to gross roll work must not be used to claim mechanical or
electrical efficiency. Near equality in symmetric constant-spin dual operation
is largely a consequence of the definition, not evidence of lossless hardware.
New calculations report separate boundaries instead of a single efficiency.

The current model supports vehicle work, rotor axial spin-energy changes,
and a gimbal-coordinate inertial-work diagnostic. It does NOT yet support a
complete independent motor mechanical-input budget: moving-base coupling,
gyroscopic gimbal loads, friction and full actuator mechanical reactions are
not assembled into an energy-consistent motor-port model. Finite rate dynamics
alone do not supply that budget. Electrical losses/regeneration are unspecified.

## Common accounting boundary

Both saved nominal roll trajectories are evaluated over 0--5 s, using actual
CMG moments and body angular velocities. Commands/configurations retain their
existing tuning and rotor states. These are configured-architecture comparisons,
not equal-resource comparisons. Installed inertia and spin strategy differ.

- Vehicle power: sum_i(tau_i dot omega_body). Roll power: (sum_i K_i)*p.
- At each boundary report integral(P), integral(max(P,0)),
  integral(max(-P,0)), and integral(abs(P)); negative work is an absorption
  magnitude, not assumed recovered electricity.
- Rotor axial spin power: I_i*Omega_i*Omegadot_i; compare its signed integral
  with the change in 0.5*I_i*Omega_i^2. This is an axial relative-spin storage
  diagnostic, not full moving-rotor kinetic energy or complete shaft work.
- Gimbal inertial power component: J_i*alphaddot_i*alphadot_i with configured
  assembly J. Compare with change in 0.5*J_i*alphadot_i^2. Gyroscopic and
  moving-base load work are excluded. This is not the entire gimbal motor work.
- Sum work at individual module ports only when explicitly reporting module
  throughput; net vehicle-port power can cancel between modules. Neither
  port throughput nor a sum of axis absolute powers is input efficiency.
- Initial spin energy is stored in the initial condition, not consumed again
  in every maneuver. Ideal rest-to-spin storage requirement may be stated
  separately, but actual spin-up energy/time/losses are not simulated.

Do not add all the following rows as independent consumption: some describe
energy transfer and others describe storage or only one torque component.

## Re-accounted results (joules)

| Quantity | Single | Dual |
|---|---:|---:|
| Net vehicle work | 0.001059 | 0.000955 |
| Positive vehicle work | 0.066936 | 0.068140 |
| Negative vehicle-work magnitude | 0.065878 | 0.067185 |
| Gross roll work | 0.13290 | 0.13533 |
| Initial axial spin energy | 0.5081 | 16.259 |
| Final axial spin energy | 0.3221 | 16.259 |
| Positive axial spin work | 11.359 | 0 |
| Negative axial spin-work magnitude | 11.545 | 0 |
| Positive gimbal inertial component | 0.054801 | 0.002173 |
| Negative gimbal inertial component | 0.054801 | 0.002174 |

Small net work must not be confused with small gross exchange. The single
controller makes a large rotor-energy excursion and later removes most of it;
the dual controller holds spin speed fixed but begins with approximately 32
times the stored spin energy. Zero axial spin work in the dual case does not
mean zero actuator energy or zero ongoing rotor losses. Comparable vehicle
roll work is expected for comparable maneuvers, but does not compare motor
efficiencies. No efficiency ranking is justified by this table.

The saved single baseline starts at 300 rpm; the dual baseline starts at
-1200/+1200 rpm. The 32-fold stored-energy difference therefore includes both
rotor count and speed, not merely single versus dual architecture.

## Implementation and reproduction

Run `COMPARE_CMG_MECHANICAL_ENERGY` then `VERIFY_CMG_MECHANICAL_ENERGY`.
The comparison snapshots input MAT files, recomputes controller/moment histories
on common 1-ms and 0.5-ms PCHIP grids, and saves CSV/MAT outputs under
`Working Results/mechanical_energy_audit`. This is quadrature refinement on
saved trajectories, NOT a fresh finer-step ODE integration or improvement of
the original solution. Final publication runs should regenerate both baselines
at a frozen revision with documented configuration and matched resource tests.

Analytic sign/storage/cancellation checks and quadrature checks passed.
Maximum spin-storage integral residual was 3.89e-8 J; maximum gimbal inertial
storage residual was 5.06e-7 J. These verify the separate accounting identities,
not total coupled-system energy closure.

`ENERGY_ANALYSIS` retains legacy proxy fields for compatibility but marks their
efficiency ratios NaN. The driver labels now explicitly say these are not
motor input. Previously saved MAT/figure files are unchanged and may still
contain obsolete efficiency ratios; regenerate before manuscript use. The
legacy baseline comparison warns about its proxy fields.

## Checklist / next step

Completed: consistent single/dual comparison of the mechanical work and
storage quantities actually supported by the current model.

Still required before claiming total mechanical input or efficiency: derive
and verify complete actuator-port power with a consistent kinetic-energy and
external-work balance, or explicitly restrict the manuscript to the work and
storage metrics above. Do not quietly replace this with the legacy proxy.

Next: physically consistent rotor geometry and matched-initial-spin-resource
comparisons from the study matrix. The present audit does not change geometry,
controller gains, plant trajectories, or hardware assumptions.
