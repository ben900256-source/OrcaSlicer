# Tool-Envelope Collision Detection v2 — Codex Handoff

## Objective

Upgrade the existing standalone tool-envelope collision kernel on branch
`feature/nozzle-collision` into an opt-in, report-only validator of the final,
ordered G-code replay through `GCodeProcessor`.

The production feature must default to `off`, must never change emitted G-code,
must remain independent of `ConflictChecker` and Z Contouring, and must report
collisions or incomplete coverage without blocking export/send. Automatic path
rerouting and automatic Z-hop insertion are explicitly out of scope.

The authoritative user plan is titled **Production Tool-Envelope Collision
Detection v2** in the conversation that produced this file. Its major required
areas are summarized below.

## Required end state

### Kernel and public API

- Sparse deterministic grid: 5 mm XY cells and 1 mm Z slabs.
- Index deposited capsules through a supercover/DDA corridor, not every cell in
  a diagonal AABB.
- Queries gather local references and sort/unique them; do not allocate a
  scene-sized `seen` vector.
- Statistics expose visited cells, references examined, candidates, exact
  results, and sweep subdivision counts.
- Solve sweep Z-overlap analytically and recurse only through possible XY
  intervals. `check_sweep()` must perform a true earliest-hit search rather
  than call the all-hit query.
- Cache obstacle capsules by query and margin, and poll cancellation during
  long work.
- Report conservative `[t_lower, t_upper]` contact brackets with an explicit
  conservative flag.
- All-hit APIs return raw slice/obstacle contacts. Obstacle summarization is a
  separate operation so slice, clearance, and raise values remain from one
  consistent contact.
- Signed policy clearance is tolerance-adjusted: a reported collision may
  never have positive policy clearance.
- Replace `required_z_raise_for_sweep()` with
  `find_safe_z_raise_for_sweep()`. The new optional plan is bounded by machine
  Z, considers obstacles encountered only after raising, merges forbidden
  raise intervals, clears inclusive tolerance with a strictly greater
  representable number, and revalidates lift/traverse/descent.
- Add `ToolIdentity`, typed source metadata, motion uncertainty, and an
  incremental `ToolpathClearanceSession`; retain the batch validator as a
  wrapper.
- Track per-tool retraction debt. Positive E is deposition only after debt is
  repaid. Stationary excess extrusion remains representable as a circular
  deposited bead.
- Depositing chords are checked against the existing scene first; fresh-bead
  prefix contact is evaluated separately; only then is the chord inserted.
- ToolEnvelope JSON schema v1 and its nozzle-tip coordinate conventions remain
  unchanged.

### Final G-code integration

- Add a null-by-default motion observer to `GCodeProcessor`.
- Events cover linear motion (raw start/end XYZ, signed E, width, height, role,
  provenance, and `ToolIdentity`), tool changes, coordinate discontinuities,
  and unsupported/unknown motion.
- Build world-space poses from raw parser coordinates plus plate offset, active
  extruder offset, and Z offset. Do not derive motion starts from `MoveVertex`;
  its `extruder_id` remains filament-oriented for compatibility.
- Reuse G2/G3 chord generation and attach sagitta as conservative XY
  uncertainty. Tool-offset changes are pose discontinuities: pose-check the
  newly active envelope without sweeping across the offset jump.
- G92 is a coordinate-state change, not motion. G28, opaque macros,
  unsupported arc modes, or untrustworthy extrusion dimensions create
  coverage issues instead of invented straight paths.
- Replay the finalized, post-processed G-code file read-only. If a later GUI
  script/plugin produces the actual final copy, replay that copy and replace
  the earlier report.

### Configuration and reporting

- Register printer options:
  - `tool_envelope_collision_mode = off | report`, default `off`.
  - `tool_envelope_json`, one compact canonical JSON document per machine
    configuration slot, sized with the existing printer-variant rules.
- Store both in presets and 3MF projects. Ban both from the G-code config dump
  so registration/report mode cannot change output bytes.
- Never fall back from a missing used slot to slot zero.
- Add expert printer-setting controls for import, validate, replace, explicit
  copy, and clear of each slot.
- Add a structured result to `GCodeProcessorResult` with verdict, coverage,
  first hit, capped retained hits, exact total colliding-motion count,
  provenance/contact/obstacle data, optional safe raise, coverage issues, and
  work statistics.
- Preview warnings are localized and diagnostic navigation highlights the
  conservative motion interval and deposited obstacle. Complete/clear results
  remain in details without a success toast.
- Add deterministic report-schema-v1 JSON output for CLI. Diagnostics go to
  stderr and report mode retains a successful exit code.
- Update documentation to state that the final parser replay is the integration
  boundary and describe coverage, conservative brackets, profile requirements,
  post-processing replay, and limitations.

## Repository and instructions

- Workspace: `C:\Users\wben7\dev\orca-nozzle-collision`
- Branch: `feature/nozzle-collision`
- Baseline commit: `b8e037a66e Add tool envelope collision kernel`
- Baseline is one commit ahead of `origin/main` and matches
  `origin/feature/nozzle-collision`.
- Root `AGENTS.md` applies. Tests also follow `tests/AGENTS.md`.
- Native Windows CMake/MSVC/MSBuild only; do not use Docker; keep PCH enabled.
- Existing build directory: `build`, Visual Studio 17 2022, Release.
- Preserve backward compatibility, cross-platform behavior, profiles, 3MFs,
  and byte-for-byte G-code when disabled or in report mode.
- Use workspace-relative paths with `apply_patch` for edits.

Git initially rejected the repository as dubious ownership because the files
were owned by the interactive Windows user and commands run as
`CodexSandboxOffline`. The workspace has already been registered as a Git safe
directory for this environment.

## Existing v1 implementation

The baseline commit already added:

- `src/libslic3r/PrintedScene.{hpp,cpp}`
- `src/libslic3r/ToolEnvelope.{hpp,cpp}`
- `src/libslic3r/ToolEnvelopeCollision.{hpp,cpp}`
- `src/libslic3r/ToolpathClearanceValidator.{hpp,cpp}`
- focused `libslic3r` tests
- `doc/tool_envelope_collision.md`

The baseline was a standalone kernel only. It used an XY AABB grid, a
scene-sized seen vector, recursive XY/Z sweep intervals, nominal first/last hit
fields, per-obstacle collapsed contacts, an unsafe scalar raise helper, and a
batch-only validator. It had no `GCodeProcessor`, configuration, GUI, CLI, or
final-file replay integration.

## Work completed in this session

### `PrintedScene` v2 index (implemented and previously verified)

Modified:

- `src/libslic3r/PrintedScene.hpp`
- `src/libslic3r/PrintedScene.cpp`

Implemented:

- 3D cell key with 5 mm XY cells and 1 mm Z slabs.
- DDA centerline traversal and padded sparse corridor insertion.
- Query reference collection followed by sort/unique instead of a scene-sized
  seen vector.
- `references_examined` and `sweep_subdivisions` statistics fields.
- `query_sweep_indices()` for sparse translated-volume corridors.
- Deterministic sorted query results with exact AABB/Z filtering.

Before beginning the collision API edits, the following succeeded:

```powershell
cmake --build build --config Release --target libslic3r_tests -- -m
& build/tests/libslic3r/Release/libslic3r_tests.exe "[ToolEnvelope]"
& build/tests/libslic3r/Release/libslic3r_tests.exe "[ToolEnvelopeCollision]"
& build/tests/libslic3r/Release/libslic3r_tests.exe "[ToolpathClearanceValidator]"
```

Results at that checkpoint:

- `[ToolEnvelope]`: 432 assertions, 7 test cases, all passed.
- `[ToolEnvelopeCollision]`: 40 assertions, 11 test cases, all passed.
- `[ToolpathClearanceValidator]`: 22 assertions, 5 test cases, all passed.

Those results validate the `PrintedScene` changes against the original tests,
but **do not validate the current collision files**, which became partially
edited afterward.

### Collision API/private solver (partial and currently inconsistent)

Modified:

- `src/libslic3r/ToolEnvelopeCollision.hpp`
- `src/libslic3r/ToolEnvelopeCollision.cpp`

The header now declares:

- `t_lower`, `t_upper`, and `conservative` on `ToolClearanceResult`.
- compatibility aliases `first_hit_t` / `last_hit_t` intended to carry the same
  conservative bracket.
- `ToolSafeZRaisePlan`.
- `ToolCollisionCancellation`.
- cancellation-aware pose/sweep APIs.
- `summarize_obstacle_contacts()`.
- `find_safe_z_raise_for_sweep()` replacing the old scalar helper.

The anonymous/private portion of `ToolEnvelopeCollision.cpp` now contains:

- expanded statistics accumulation.
- tolerance-adjusted signed vertical clearance.
- bracket-based diagnostics and ordering.
- raw-contact finalization.
- a cancellation poller.
- per-query/margin capsule cache.
- analytic Z-overlap interval solving.
- left-first, XY-only recursive earliest-bracket search.
- forbidden-interval merge support for safe raises.

However, the public function implementation patch was interrupted and **did
not apply**. The file still has the old public implementations beginning near
`check_pose_all()` / `check_sweep_all()` and still references removed private
symbols such as the old per-obstacle merge machinery and old sweep accumulator.
It also still defines `required_z_raise_for_sweep()` while the header no longer
declares it. Therefore the current worktree is expected not to compile.

Current `git status --short` at handoff:

```text
 M src/libslic3r/PrintedScene.cpp
 M src/libslic3r/PrintedScene.hpp
 M src/libslic3r/ToolEnvelopeCollision.cpp
 M src/libslic3r/ToolEnvelopeCollision.hpp
```

No validator, parser, configuration, GUI, CLI, documentation, localization, or
test files have been edited in this session.

## Immediate next action

Finish the public half of `ToolEnvelopeCollision.cpp` so it matches the new
header and private helpers. Do this before any additional feature work.

The interrupted patch intended to:

1. Make `check_pose_all()` return raw slice/obstacle contacts, use the capsule
   cache, cancellation, tolerance-adjusted clearance, and strictly greater
   representable raise boundaries.
2. Make `check_pose()` select the first sorted raw contact.
3. Make `check_sweep_all()` use `PrintedScene::query_sweep_indices()`, cached
   capsules, analytic-Z/XY-recursive brackets, raw contacts, and subdivision
   stats.
4. Implement `check_sweep()` independently as a true earliest-hit path with a
   current-best upper bound; it must not call `check_sweep_all()`.
5. Implement `summarize_obstacle_contacts()` by selecting the earliest complete
   raw contact per obstacle without mixing fields from other slices.
6. Implement `find_safe_z_raise_for_sweep()`:
   - validate finite machine max Z;
   - query all XY-overlapping obstacles over the entire possible raise range;
   - create and merge forbidden raise intervals for every active slice;
   - choose the smallest non-forbidden raise using `std::nextafter`;
   - enforce machine Z;
   - revalidate vertical lift, raised traverse, and descent;
   - return `std::nullopt` if any phase collides.

Then compile immediately. Expect failures until old public references are
removed.

## Suggested implementation sequence after restoring compilation

1. Update existing collision tests for raw contacts and bracket fields.
2. Add randomized indexed-vs-brute-force tests, negative/grid-boundary cases,
   holes, tangency, zero-length paths, duplicate IDs, all Z directions, and
   deterministic ordering.
3. Add algorithmic work-bound tests for vertical distributions, diagonals,
   persistent overlap, and pure Z motion.
4. Add dense-oracle bracket tests and both safe-raise regressions:
   inclusive-boundary landing and raising into a previously irrelevant higher
   obstacle.
5. Refactor `ToolpathClearanceValidator` into an incremental
   `ToolpathClearanceSession` with typed identity/source/uncertainty and
   per-tool retraction debt. Keep batch validation as a wrapper.
6. Add the null motion-observer interface and parser events to
   `GCodeProcessor`; cover G90/G91/G92, G28 coverage issues, arcs and sagitta,
   tool/offset discontinuities, unknown motion, and cancellation.
7. Register and size the two printer options, ban both from
   `GCode::append_full_config()`, and add compatibility/resizing tests.
8. Wire read-only replay after final internal post-processing, then handle any
   later GUI-created final copy by replacing the earlier report.
9. Add structured `GCodeProcessorResult`, warning/navigation UI, deterministic
   CLI schema-v1 JSON, and documentation.
10. Run targeted builds during iteration, then complete `libslic3r`,
    `fff_print`, and OrcaSlicer application builds/tests.

## Important design details already investigated

- `GCodeProcessorResult` is in
  `src/libslic3r/GCode/GCodeProcessor.hpp` around line 178.
- `GCodeProcessor` public/private state and parser methods are in the same
  header around line 463 onward.
- `process_file()` is in `GCodeProcessor.cpp` around line 3598 and already
  accepts a cancellation callback.
- Linear motion is handled by `process_G1()` around line 4856.
- Arc chord generation is already in `process_G2_G3()` around line 5627; it
  calls internal G1 chords.
- G28 is currently converted into an invented G1-to-zero around line 6000;
  the observer must emit a coverage issue instead.
- G90/G91/G92 are around lines 6026–6036; G92 only changes coordinate state.
- Tool changes are handled around line 6540.
- Preview vertices are built in `store_move_vertex()` around line 6990. They add
  plate and extruder offsets and alter Z for preview; do not use them as raw
  observer starts.
- The G-code config-dump banned set is in `src/libslic3r/GCode.cpp` around line
  6703.
- Printer option registration is in `src/libslic3r/PrintConfig.cpp`; existing
  printer-variant sizing logic appears around lines 11261 and 11388.
- The current documentation incorrectly says the future integration boundary
  is `GCode::_extrude`; v2 must change it to finalized `GCodeProcessor` replay.

## Patch-tool behavior in this session

The nested `apply_patch` tool sometimes appeared to hang for 30–200 seconds.
The workspace is native NTFS, not WSL or Docker, and ACLs grant
`CodexSandboxUsers` Modify access. No active MSBuild/compiler process caused the
delay.

The effective workaround was:

```javascript
const pending = tools.apply_patch(patch);
await yield_control();
text(await pending);
```

The outer cell then yields immediately and can be completed with the wait tool.
Use relative patch paths. A fresh session may not exhibit the issue.

## Safety / scope reminders

- Preserve all current user changes; do not reset or discard the partial diff.
- Do not guess or add vendor envelopes.
- Default policy remains hard-collision + toolhead, with deposition + thermal
  ignored and all added margins zero.
- Detection/reporting only; no blocking and no automatic avoidance.
- The modeled scene is active tool versus already deposited material only. Bed
  fixtures, parked carriages, thermal evolution, warping, and rotation remain
  limitations.

