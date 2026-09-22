# Documentation — start here

You do not need to read every development note to run the model.

[Folder migration and verification record](REORGANIZATION.md)

## Current decisions

1. [First-draft scope, retained cases and criteria](publication/IEEE_JOE_FIRST_DRAFT_FREEZE.md)
2. [Allocator scope decision](publication/ALLOCATION_SCOPE_DECISION.md)
3. [Inertia-estimation target and tested bounds](assumptions/INERTIA_ESTIMATION_BOUNDS.md)
4. [Finite-duration disturbance envelope](assumptions/ROLL_DISTURBANCE_ENVELOPE.md)
5. [Actuator assumptions](assumptions/CMG_LIMIT_ASSUMPTIONS.md) and
   [installed mass assumptions](assumptions/INSTALLED_MASS_PROPERTY_ASSUMPTIONS.md)

## Findings by topic

- [Roll, geometry, mechanical work and momentum](findings/roll/)
- [Mission sequencing, thrusters and maneuver planes](findings/mission/)
- [Estimation, actuator and installed-mass uncertainty](findings/uncertainty/)
- [Disturbance screening and failure diagnosis](findings/disturbance/)

Findings are development records, not automatically current manuscript numbers.
Their embedded MATLAB command names remain valid after `CMG_SETUP`. Bare file
names mentioned in older notes refer to files listed in [file-map.json](file-map.json).
Retained known failures and distinctions between controller variants must remain
visible when regenerating publication results.

## Historical reference

- [Previous long-form development guide](archive/DEVELOPMENT_GUIDE.md)
- [Superseded simulation planning matrix](archive/IEEE_JOE_SIMULATION_MATRIX.md)

The archived guide contains older status statements; it is not the current
run checklist. The first-draft freeze above takes precedence. Neither this index
nor the reorganization certifies the final publication batch as complete.
