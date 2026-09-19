# Matched initial stored rotor energy: single versus dual

## Design of comparison

Two pairs use solid steel disks, 4-inch diameter, density 7750 kg/m^3,
and 1200-rpm initial spin magnitude:

| Pair | Single thickness | Each dual thickness | Shared initial energy |
|---|---|---|---|
| A | 0.5 in | 0.25 in | Approximately 8.130 J |
| B | 1.0 in | 0.5 in | Approximately 16.259 J |

At common radius/density, mass and axial inertia scale linearly with thickness.
Each pair therefore matches total rotor mass, summed axial inertia, initial
stored spin energy and summed spin-momentum magnitudes. Vector momentum and
accessible roll-momentum envelopes are NOT identical. The single rotor starts
at alpha=0 and +1200 rpm; dual uses alpha=[-15,+15] deg, Omega=[-1200,+1200]
rpm. Initial vehicle states, 90-degree roll command, PD gains, five-second
horizon and configured per-actuator bounds are common. There is no retuning.

Rotors are physically redefined and installed mass/tensors reassembled from
the same bare vehicle. Single placement is at the origin; dual placement is
at +/-0.30 m. Total modeled mass matches within pairs but installed inertia
does not, owing to transverse rotor geometry, orientation and mounting.
The single architecture permits spin acceleration; dual retains constant spin.
This compares configured architectures at matched initial resources, not a
pure causal test of rotor count. Two gimbal drives are not one gimbal drive;
unmodeled frame/motor/housing mass and power are not resource-matched.

All rotor-only swept envelopes are screened against the user-supplied 5.8-inch
tube diameter. This does not certify a usable internal bore or complete module
clearance. The existing frozen initial structural inertia and omitted
hydrostatic restoring-load assumptions remain unchanged.

## Metrics and reproduction

Run `MATCHED_ENERGY_COMPARISON`, `MATCHED_ENERGY_COMPARISON(.005)`, and
`VERIFY_MATCHED_ENERGY`. Outputs and full histories are in
`Working Results/matched_energy/`; existing baseline files are unchanged.
Tests check equality of initial energy, mass and summed spin inertia within
pairs, plus finer-step trajectory/work agreement. Actual gimbal-angle and
flywheel-speed stop events end a run rather than using invalid post-stop data.

Report roll settling/error, pitch/yaw, actual actuator demands and limits,
gross roll work, initial/final spin storage, and signed-direction spin work.
Gimbal inertial work is only one motor-work component. No total motor input or
electrical-efficiency ratio is asserted. Initial energy equality does not
mean equal energy throughout a variable-spin maneuver. Failed runs and unequal
terminal stored energy must remain visible in any architecture comparison.

## Results

| Pair | Mode | Duration (s) | 2% roll settling (s) | Peak pitch/yaw (deg) | Peak spin (rpm) | Peak gimbal angle (deg) |
|---|---|---:|---:|---|---:|---:|
| A: 8.130 J | Single | 0.594 | Not reached | 0.014 / 0.057 (partial) | 1800 | 49.17 (partial) |
| A: 8.130 J | Dual | 5 | 2.895 | 0 / 0 | 1200 | 69.84 |
| B: 16.259 J | Single | 5 | 2.883 | 8.336 / 4.019 | 1394.4 | 31.48 |
| B: 16.259 J | Dual | 5 | 2.881 | 0 / 0 | 1200 | 20.09 |

Pair A single reaches the physical spin-speed bound before completing the
roll (remaining roll error 72.45 degrees). Its trajectory is terminated by
the test; no claim is made about a speed-saturated continuation. This is a
failure of the tested controller/preload/spin/geometry combination, not proof
that all single-CMG solutions at this energy are infeasible.

Pair B gives a completed-maneuver comparison: gross roll work is 0.13503 J
single versus 0.13533 J dual. The single's positive/negative axial spin work
is 5.697/6.767 J, finishing with 15.190 J stored spin energy; dual axial spin
work is zero and storage remains 16.259 J. Initial equality does not imply
equal terminal storage or equal total motor work. The dual's ideal symmetric
cross-axis cancellation contrasts with the single's appreciable pitch/yaw.

The reported `passes` field is a roll/actuator screen only: five-second
completion, final roll error <=1.8 degrees, settling <=4 s and no recorded
limit flags. It does NOT qualify cross-axis motion for a full mission. Three
cases pass this limited screen; Pair B single should not be labelled a
fully equivalent maneuver solely because its roll response passes.

Energy/mass/inertia matching and finer-step checks passed for all four cases,
including retaining the failed Pair A single outcome. Baseline configurations
were not changed. Results support conditional architecture tradeoffs, not a
global optimum, electrical-efficiency ranking or a full-mission validation.

## Trello

Complete: Compare single/dual CMGs with matched initial stored rotor energy
for two physically consistent common-speed geometry pairs.

Manuscript note: equal initial rotor energy and rotor mass do not eliminate
controller, preload, mounting, motor-count or terminal-energy differences.
Report the single speed-limit failure and the completed single case's
cross-axis response alongside the dual results.
