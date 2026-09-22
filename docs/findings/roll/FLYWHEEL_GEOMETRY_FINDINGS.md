# Physically consistent flywheel geometry study

## Envelope and model assumptions

User-supplied IVER 3EP tube diameter: 5.8 in = 147.32 mm. It has not been
established whether this is a usable bore or external diameter. No fit claim
for an assembled module follows from this number. A centered solid disk has
a conservative all-orientation bounding diameter sqrt(D^2+t^2). The reported
radial remainder is (147.32 mm - bounding diameter)/2, before tube wall,
gimbal/frame, shaft/bearings, motor, housing, cables, tolerances and clearance.
An off-center pivot or protruding shaft requires a larger envelope. Available
longitudinal installation space is not evaluated.

The study uses solid disks at the existing assumed steel density 7750 kg/m^3.
It does not introduce a central bore, rim, spokes or hub. Mass is rho*pi*r^2*t,
spin inertia is m*r^2/2, and transverse inertia is m*(3*r^2+t^2)/12.
Each candidate replaces, rather than adds to, the two nominal rotor masses.
Vehicle mass/tensor are reassembled from the bare-vehicle baseline using
initial rotor orientation and the existing symmetric +/-0.30 m axial mounts.
Gimbal assembly inertia is also recomputed using the existing frame allowance
and motor-rotor inertia assumption.

Candidate diameters/thicknesses (inches): 3.2/0.5, 4.0/0.5, 4.8/0.5,
4.0/0.25 and 4.0/0.75. This is a five-point design sensitivity study, not a
geometry optimization or final rotor selection.

## Experiment and demand definitions

Each candidate runs the same dual +90-degree roll for 5 s with +/-15-degree
initial gimbals, -1200/+1200 rpm initial spin, existing gains and actuator bounds.
The allocator knows the candidate rotor inertia. No estimation errors, waves,
thrusters or propulsion are added. Larger rotors store more energy and angular
momentum at this fixed speed; this is not the matched-resource comparison.

Reported gimbal torque/power are J*alphaddot and J*alphaddot*alphadot:
inertial components only, not full motor requirements. Spin acceleration is
zero in these constant-speed dual trajectories. I*500 rad/s^2 is reported
separately as the axial torque needed if the configured spin acceleration
limit were demanded; it is not observed maneuver torque or a tested motor
capability. Initial spin energy is a storage requirement, not electrical input.
No rotor stress, balancing, bearing load, friction, fatigue or thermal
qualification is performed.

The installed tensor remains frozen at the initial gimbal angles, as in the
baseline model. Symmetric placement cancels initial products of inertia, but
this is not a complete time-varying multibody inertia model. Changes to vehicle
weight do not generate buoyancy/restoring loads in the current REMUS model.
Frame and motor masses are still excluded from assembled vehicle mass.

## Reproduce

```matlab
FLYWHEEL_GEOMETRY_SWEEP
FLYWHEEL_GEOMETRY_SWEEP(.005)
VERIFY_FLYWHEEL_GEOMETRY
```

CSV summaries and full MAT histories are saved under
`Working Results/flywheel_geometry/`. The nominal simulation configuration
and saved baseline are not changed. Roll screening retains final error <=1.8
degrees and settling within the 2% band by 4 s; the overall screen also
requires no recorded actuator-limit flags. A positive rotor-envelope remainder
is not included as evidence of assembled hardware feasibility.

## Results

| Diameter x thickness (in) | Mass per rotor (kg) | Spin inertia (kg m^2) | Radial remainder (mm) | Peak gimbal angle (deg) | Peak gimbal rate (rad/s) | Overall screen |
|---|---:|---:|---:|---:|---:|---|
| 3.2 x 0.5 | 0.5107 | 0.00042174 | 32.53 | 97.51 | 11.52 | Fails: acceleration limiting |
| 4.0 x 0.5 | 0.7980 | 0.0010296 | 22.47 | 20.09 | 1.752 | Pass |
| 4.8 x 0.5 | 1.1491 | 0.0021350 | 12.37 | 15.13 | 0.849 | Pass |
| 4.0 x 0.25 | 0.3990 | 0.00051481 | 22.76 | 69.84 | 3.476 | Pass |
| 4.0 x 0.75 | 1.1969 | 0.0015444 | 21.98 | 15.17 | 1.172 | Pass |

All five completed and met the roll-error/timing screen (settling times
2.874--3.082 s). Four also avoided recorded actuator limits. The small disk
reached 500 rad/s^2 acceleration and 97.51 degrees gimbal angle, leaving
little angular margin to the assumed 100-degree bound. This is not evidence
that small rotors are generally infeasible: only this preload/speed/controller
combination was tested.

Total installed vehicle mass ranged from 31.277 to 32.873 kg; recomputed roll
inertia ranged from 0.17746 to 0.17931 kg m^2. Total initial rotor spin energy
was 6.660, 16.259, 33.715, 8.130 and 24.389 J in case order. Reduced rate at
larger diameter comes with more stored energy, mass and less packaging space.

Peak gimbal inertial torque was 0.1493, 0.1288, 0.1255, 0.1318 and 0.1303
N m, respectively. Corresponding inertial peak power was 1.678, 0.0597,
0.0281, 0.1220 and 0.0403 W. Lower rate demand therefore does not imply
proportional torque reduction: the accelerated gimbal inertia also changes.
These are not full motor torque/power requirements. Torque to achieve the
configured 500 rad/s^2 axial spin acceleration would range from 0.211 to
1.068 N m per rotor; that spin-up trajectory was not simulated.

Geometry scaling, rotor-envelope rejection, mass assembly and half-step
trajectory checks passed, retaining all screen outcomes.

## Design interpretation and checklist

Keep the 4.0 x 0.5-inch rotor as the baseline for now. It has a less constrained
rotor-only envelope than the 4.8-inch disk and substantially less gimbal travel
than the thinner disk. This is a model-development choice, not a hardware
recommendation or optimum. The 4.0 x 0.25-inch candidate is worth retaining
as a lower-mass comparison, not substituting into the full mission without
testing its reduced momentum reserve.

Complete: investigate physically consistent rotor geometry, installed mass
and actuator-demand tradeoffs for the dual 90-degree roll baseline.
Next: matched-resource single/dual comparison; selected candidate full-mission
testing if geometry selection is to extend beyond this baseline roll study.
