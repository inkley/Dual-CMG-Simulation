# CMG AUV Simulation

Tyler J. Inkley's MATLAB simulation study of single/dual CMGs and staged
roll–turn–surge maneuvers with fixed sway/yaw thrusters. The model is research
software for an IEEE JOE draft, not hardware validation or general robustness.

## Start here

Use MATLAB R2025b (the tested release). Set this repository as the current
folder. In an already-running MATLAB session, initialize its code paths once:

```matlab
CMG_SETUP
```

Then use the **same command names as before**:

```matlab
AUV_SIM                         % Main single/dual driver; settings near its top
RUN_ROLL_TURN_SURGE(struct('planePitchCorrection',true))
MATCHED_ENERGY_COMPARISON
```

`AUV_SIM.m` remains at the top level and initializes paths itself. It still
clears workspace variables and closes figures; edit its USER CONFIGURATION
section before running. Other defaults/case lists are in [config](config/).
`startup.m` initializes paths when MATLAB starts in this directory; changing
folder in an existing session does not automatically invoke startup.
No installation, `savepath`, toolbox, or MATLAB preference changes are made.

## Folder guide

| Location | What belongs here |
|---|---|
| `AUV_SIM.m` | Main driver, user settings, baseline plotting |
| `config/` | Mission defaults, uncertainty case lists, disturbance plans |
| `src/model/` | Vehicle/CMG/actuator dynamics and mass properties |
| `src/control/` | Allocation, feedback, command generation, supervisor |
| `src/analysis/` | Reusable diagnostics and mechanical accounting |
| `studies/roll/` | Single/dual, geometry, energy and momentum studies |
| `studies/mission/` | Roll-to-plane, hybrid maneuvers and propulsion studies |
| `studies/uncertainty/` | Estimation, installed-mass and actuator studies |
| `studies/disturbance/` | Disturbance experiments and diagnostic reports |
| `tests/` | Verification functions and legacy test scripts |
| `docs/` | One documentation index, scope, assumptions and findings |
| `Working Results/` | Generated outputs, unchanged and ignored by Git |

Functions and study entry points were moved, not renamed. After `CMG_SETUP`,
their bare MATLAB commands work from any folder. Use `which CONTROL` or
`which RUN_ROLL_TURN_SURGE` to locate their implementations. Full file paths
used by external editor bookmarks must be updated; see the complete
[old-to-new map](docs/file-map.json).

## Run the relevant check, not every experiment

```matlab
CMG_TESTS('unit')   % Fast equations/logic/layout checks; some use saved baseline
CMG_TESTS('saved')  % Checks existing saved study outputs; does not regenerate them
CMG_TESTS('all')    % Both suites, reporting all failures before stopping
```

A clean clone must first generate the required baseline/study artifacts.
Saved-result checks do not replace regeneration of the publication batch.
An optional pre/post integration check, `VERIFY_REORGANIZATION_REFERENCE`,
uses the locally archived reference fixture; see [verification details](docs/REORGANIZATION.md).
Study runners still overwrite their original scenario output folders; archive
important results before rerunning. This reorganization is not a new batch
runner, changed controller, or changed acceptance criterion.

## Current research status and next action

Start with the [documentation index](docs/README.md), especially the
[first-draft freeze](docs/publication/IEEE_JOE_FIRST_DRAFT_FREEZE.md).
Retain the existing damped allocator and report the known estimation-sensitive
failures. The provisional +/-5% inertia-knowledge target is supported by selected
disturbed holds, not a continuous robust uncertainty guarantee.

Next: regenerate and archive the frozen publication case list with traceable
configurations. Do not reuse historical numerical claims without checking their
model revision and controller variant. Mechanical work/storage comparisons are
not electrical or total motor-input efficiency. Persistent roll bias consumes
finite momentum; center-plane thrusters cannot unload roll momentum. Missions
end in cruising, not station keeping or waypoint stopping.

Historical detail is preserved under `docs/findings` and `docs/archive`, not
deleted. No open-source license is granted; confirm research ownership before
selecting one.
