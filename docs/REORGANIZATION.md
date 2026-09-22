# Repository reorganization — 2026-09-22

## What changed

Moved 97 MATLAB files and 37 Markdown documents out of the flat root into
`config`, `src`, `studies`, `tests`, and indexed `docs` folders. Nothing from
the original model/study library was discarded. `docs/file-map.json` maps all
134 original names to their new locations, including previously uncommitted
scope and estimation-bound work.

The main driver and its user configuration remain in `AUV_SIM.m`. Its only
code change is a call to `CMG_SETUP` after the existing workspace/figure reset.
The moved MATLAB files are byte-identical to the pre-move snapshots except
55 files whose repository-root lookups now use `CMG_ROOT()` instead of their
own file location. Model equations, gains, damping, limits, solver settings,
case definitions and acceptance criteria were not edited.

`CMG_SETUP` adds only code folders, without saving MATLAB paths/preferences.
The root `startup.m` runs it when MATLAB starts here. In an already-open MATLAB
session, run `CMG_SETUP` once after switching to this repository. Existing
bare study/function commands work as before; editor bookmarks using old
absolute file paths must be updated. Generated output directories are unchanged.

## Tests and evidence

- `CMG_TESTS('all')`: 27 equation/logic/layout and saved-result checks passed.
  This means verifier success, not that every scientific scenario passed;
  known strict-screen failures remain preserved in the underlying results.
- Pre/post numerical runs: single-CMG and dual-CMG five-second cases plus the
  200-second unequal-inertia disturbance failure. Every recorded state and
  full replayed control-history field compared exactly equal (`isequaln`).
- Top-level `AUV_SIM` ran to completion, including its figure exports, using
  its unchanged default configuration. Existing formal-K/M conditioning and
  axes-toolbar export warnings are not new failures.
- Corrected nominal +45/+45 mission (`estimationCase=1`, pitch recovery ON)
  was regenerated: COMPLETE at 66.10 s with a strict-screen pass. Full sampled
  mission history and result structure exactly matched the pre-move outputs.
- File/path inventory validation passed from the root and a nested study
  directory. README/index local navigation targets were checked.

Pre-move inputs and numerical reference are retained locally in
`Working Results/reorganization_validation/numerical_reference.mat` (ignored
by Git). After setup, `VERIFY_REORGANIZATION_REFERENCE` replays those three
cases. A clean clone needs the separately archived fixture; do not recreate a
"before" fixture from changed code and call it independent validation.

Driver and selected mission outputs were backed up before smoke-test reruns.
The source layout/path audit and full numerical comparisons apply to this
reorganization; they do not replace the frozen publication-batch regeneration.
No new controller, calibration guarantee, or physical validation is claimed.

## Daily use

Start with the root README, then `docs/README.md`. Choose individual studies
from the frozen case list instead of treating every development file as a
required workflow step. Use `CMG_TESTS('unit')` for fast checks and `saved` for
existing outputs. Historical studies and notes remain discoverable by their
unchanged names without crowding the top level.
