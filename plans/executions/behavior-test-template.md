# Behavior Test Execution — Phase BL{N} (`{package or group}`)

**Date:** YYYY-MM-DD
**Executor:**
**Container image:**
**Branch:**
**Plan:** [../behavior-test-bl{N}-*.md](../behavior-test-bl{N}-*.md)

## Carry-over context

Bullets of issues inherited from earlier phases that still apply here. Link to the prior execution records and to any `plans/errors/*.md` entries they spawned. Examples to delete or adapt:

- **BL{N-1}:** <summary of finding>, see [behavior-test-bl{N-1}-<group>-execution.md](behavior-test-bl{N-1}-<group>-execution.md).
- **Container:** April-2026 apt upgrades assumed in place (per [smoke-test-b1-ch1-execution.md](smoke-test-b1-ch1-execution.md)).
- **GUI:** rendering blocked by container DRI; topic-level evidence used in place of any window check, per [behavior-test-conventions.md](../behavior-test-conventions.md) "Common caveats".
- **Known deferred:** `tf_transformations` numpy-2 import crash for any module that pulls it in — see [errors/fixed/tf_transformations-numpy2-incompatibility.md](../errors/fixed/tf_transformations-numpy2-incompatibility.md).

## Pre-flight

Canary command + result:

```bash
$ python -c "import <module>; print('canary ok')"
canary ok
```

Environment dump (paste actual values, do not leave placeholders):

```bash
$ ros2 doctor --report 2>&1 | head -40
$ env | grep -E "^(ROS_|RMW_|GZ_|LIBGL_|TURTLEBOT3_)"
$ ros2 pkg prefix <packages from phase>
$ colcon build --packages-select <packages from phase>
```

Note any prerequisite that did not hold (missing external package, stale upgrade, missing bag file). If a precondition fails, STOP — do not record half-runs as PASS.

## Acceptance-criteria summary table

One row per criterion from the phase plan. Verdict is one of `PASS` / `FAIL` / `SKIP` / `N/A (env)`.

| # | Criterion | Verdict | Evidence ref |
|---|---|---|---|
| 1 | <copy from phase plan> | PASS / FAIL / SKIP / N/A | §Evidence per criterion #1 |
| 2 |  |  |  |
| 3 |  |  |  |

## Evidence per criterion

### #1 — <criterion text>

Command(s) run and captured output:

```text
$ <command>
<output>
```

Verdict reasoning (one or two sentences linking output to the threshold from [behavior-test-conventions.md](../behavior-test-conventions.md)).

### #2 — <criterion text>

```text
$ <command>
<output>
```

(Repeat per criterion. Keep blocks short — paste the lines that matter, not full logs. Reference artefact files by path for the bulk content.)

## Issues found

Incidents discovered during this run. Open a `plans/errors/<short-name>.md` for each non-trivial issue and link it here. Use the template style of [errors/fixed/tf_transformations-numpy2-incompatibility.md](../errors/fixed/tf_transformations-numpy2-incompatibility.md).

- **Issue A — <title>** (severity: blocker / major / minor / informational). See [../errors/<short-name>.md](../errors/<short-name>.md).
- **Issue B — <title>** (informational, no file written; one-line description).

## Recommendations

Adjustments to the phase plan or the conventions doc that this run surfaced. Each item should be actionable in a follow-up edit — not a wishlist.

1. <e.g. "Tighten the controller heading-tolerance criterion from 10° to 5° — closed_loop_pid converges to 3° in 80% of trials.">
2. <e.g. "Add a pre-flight grep for `<ambient>` colour ranges before launching any `.world`.">

## Checkbox dispositions

Exact copy of the acceptance checkboxes from the phase plan, ticked with the verdict marker.

- [x] / [ ] Criterion 1 (replace `x` with the box state; annotate FAILed boxes with the issue ID, e.g. "blocked by Issue A").
- [ ] Criterion 2
- [ ] Criterion 3

## Artefacts

All under `/tmp/` (ephemeral; lifetime = container session). List every generated file with its absolute path so a reviewer can re-run the assertions:

- `/tmp/bl{N}_<scenario>_$$.mcap` — recorded bag from the capture-then-assert run (per [behavior-test-conventions.md](../behavior-test-conventions.md) "Rosbag capture-then-assert pattern").
- `/tmp/bl{N}_<node>_launch.log` — launch stdout/stderr capture.
- `/tmp/frames_<timestamp>.pdf` / `.gv` — TF tree dump from `ros2 run tf2_tools view_frames` (where applicable).
- `/tmp/bl{N}_<criterion>_evidence.txt` — any side-channel output the executor saved out-of-band.
