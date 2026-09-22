# Installed mass and rigid-body inertia sensitivity

## Scope

The nine-case sweep uses the current symmetric dual-CMG baseline. Each case
runs a five-second 90-degree roll and the complete +45-degree plane/+45-degree
heading roll-turn-surge mission. Controllers and actuator limits are unchanged.

Case scales, relative to saved assembled baseline:

| Case | Total mass | Roll inertia Ix | Transverse inertia Iy, Iz |
|---|---:|---:|---:|
| 1 | 1.0 | 1.0 | 1.0 |
| 2 | 0.9 | 1.0 | 1.0 |
| 3 | 1.1 | 1.0 | 1.0 |
| 4 | 1.0 | 0.8 | 1.0 |
| 5 | 1.0 | 1.2 | 1.0 |
| 6 | 1.0 | 1.0 | 0.8 |
| 7 | 1.0 | 1.0 | 1.2 |
| 8 | 0.9 | 0.8 | 0.8 |
| 9 | 1.1 | 1.2 | 1.2 |

These are deterministic sensitivity ranges, not hardware tolerances or an
exhaustive uncertainty box. The sweep changes total installed rigid-body
properties, not only the added CMG-module contribution. Rotor spin inertia
and measured spin speeds remain nominal: this is separate from the preceding
rotor-estimation uncertainty study. No additional rotor mass is added twice.

`APPLY_INSTALLED_MASS_UNCERTAINTY` scales true mass and principal moments and
checks positivity, principal-moment triangle inequalities and zero CG. It
keeps auv.m, params.m and weight bookkeeping consistent. Mass/Ix/Iy/Iz scales
are parametric perturbations, not a detailed physically reassembled design.
The surge controller retains nominal effective mass for its feedforward term;
it is not given knowledge of the mass perturbation.

## What this excludes

The model holds CG at the body origin and has diagonal inertia. Iy and Iz
are scaled together, preserving their small baseline difference. No CG/CB
shifts, products of inertia, independently perturbed transverse moments or
gimbal-dependent structural inertia are introduced.
Added mass and drag remain fixed. Hydrostatic forces/moments are currently
disabled in REMUS, so the analysis does not model buoyancy/trim effects from
changed weight. It cannot establish robustness to general installation error.

## Reproduce

Run `VERIFY_INSTALLED_MASS_UNCERTAINTY`, `VERIFY_MASS_PROPERTY_ASSEMBLY`,
`INSTALLED_MASS_UNCERTAINTY_SWEEP`, then the refinement below, followed by
`VERIFY_INSTALLED_MASS_RESULTS`:

```matlab
RUN_ROLL_TURN_SURGE(struct('installedMassScales',[1.1,1.2,1.2], ...
    'sampleTime',.025,'maxStep',.005))
```
The sweep saves a checkpoint after each case in
`Working Results/installed_mass_uncertainty/`, including summary.csv, sweep.mat,
roll trajectories and mission summaries. Detailed uncertain mission histories
are in scale-labelled subfolders of the existing roll_turn_surge output folder;
nominal mission output is regenerated normally. Saved baseline inputs are not
modified. All generated results remain excluded from Git.

Roll screening uses a final error <=1.8 degrees and 2%-band settling by 4 s.
Mission screening retains RUN_ROLL_TURN_SURGE's existing criteria. Completion
means a recorded cruise segment, not stopping or waypoint arrival. A completed
trajectory can still fail its performance screen; report both separately.

## Results

Nominal installed properties were 32.075 kg and approximately
diag(0.17812, 3.5956, 3.5947) kg m^2. All nine complete mission cases passed
the existing screening criteria without retuning, recorded actuator-limit
flags, or sampled roll infeasibility. All nine roll-only runs completed within
1.8 degrees final error, but only seven met the four-second settling screen.
Cases 5 and 9 (Ix +20%) settled in 4.94 s versus 2.89 s nominal. Their final
roll error was -1.689 degrees. This is a timing limitation, not divergence.

The mission supervisor accommodated slower roll capture by delaying thrust:
surge started between 36.15 and 40.45 s, compared with 38.10 s nominal.
The high-mass/high-inertia corner was the largest heading/plane-error case:
final heading error 0.6285 degrees, maximum surge plane error 0.8731 degrees,
and maximum out-of-plane displacement 0.03787 m. These remain below the
existing 1-degree and 0.05-m thresholds but are not large margins.

Across all missions, final speed was 0.49949--0.49959 m/s; progress was
12.5006--12.5249 m; maximum cross-track error was 0.000453 m. Peak aft force
ranged from 3.2888 to 3.7388 N, and maximum gimbal travel was 63.272 degrees.
No mass-only roll response change was observed, consistent with the current
zero-CG, symmetric roll-only dynamics. Transverse inertia affected the turn
and subsequent plane-holding behavior much more than the isolated roll.

The high-mass/high-inertia corner also passed with supervisor sampling and
maximum integration step halved. Surge activation shifted from 40.45 to
40.40 s; final heading and peak surge-plane errors changed by less than
0.001 degree. This supports numerical consistency of that representative
corner, not convergence of every uncertain case or physical model validity.

## Trello

Complete: Evaluate installed-mass and diagonal-inertia uncertainty for the
dual roll baseline and +45/+45 roll-turn-surge demonstrator.

Retain caveat: seven of nine roll-only cases met the four-second settling
target; all nine complete missions passed using capture-based sequencing.
This is not a general robustness proof or validation of buoyancy/CG effects.
