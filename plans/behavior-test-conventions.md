# Behavior Test Conventions

**Type:** Shared conventions doc (no phase of its own).
**Referenced by:** every `behavior-test-bl{N}-*.md` (bl1..bl7) and `executions/behavior-test-template.md`.

## Overview

This document collects the shared acceptance-criteria templates, headless-Gazebo recipe, rosbag capture pattern, and import-canary snippet used by every behavioral verification phase of the Jazzy port of [rclpy-from-zero-to-hero/ros2_src/](../ros2_src/). Verification is **headless-first** — programmatic checks via `ros2 topic`, `ros2 lifecycle`, mcap bag post-processing, and Python assertion helpers — because the Jazzy devcontainer cannot reliably open DRI/Qt windows. GUI confirmation (gz-sim window, rqt_image_view, RViz) is optional only. Phase files reference the templates below by name (e.g. *"acceptance criteria per the Controller template in [behavior-test-conventions.md](behavior-test-conventions.md)"*) rather than re-stating them.

## Headless Gazebo snippet

Every Gazebo-driven phase (bl3, bl5a, bl5b, bl6, bl7) must launch gz-sim in **server-only** mode. Two knobs do the work:

```bash
# 1. Force software OpenGL so any GUI library that does start up does not crash
#    looking for /dev/dri.
export LIBGL_ALWAYS_SOFTWARE=1

# 2. Server-only: -r runs simulation, -s suppresses the GUI. Pass via launch arg.
ros2 launch <pkg> <file>.launch.py gz_args:='-r -s <world>.world'
#                                   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#                                   the -s here is what makes it headless;
#                                   the world arg is whatever the launch normally passes.
```

If the launch file hard-codes `gz_args` without a substitution, prefer driving the bridge directly with `ros2 run ros_gz_sim gz_sim --headless-rendering -s <world>` from a Python helper, or pass the override via a temporary launch wrapper. **Do not** edit the package launch file to "fix" this — the launch is correct for an interactive session.

**Detect that gz-sim is actually running, without a window:**

```bash
ros2 topic hz /clock        # expect non-zero (gz publishing sim time)
# Optional cross-check:
gz topic -l | head           # gz CLI lists the server's topics
gz model --list              # entities present in the world
```

If `/clock` reports `no new messages` for more than ~5 s after launch, gz-sim is not running — abort the phase and inspect the launch log before trusting any downstream assertion. See [executions/smoke-test-b5-project-execution.md](executions/smoke-test-b5-project-execution.md) Issue A for an example of `gz sim` exiting silently and downstream criteria registering "advertised" but never "data".

## Rosbag capture-then-assert pattern

The default for any **filter** (bl4, bl7 EKF) or **playback-driven planner** (bl6, bl7 bug2) check is to **record** an mcap and assert on it offline. Do not live-assert — live assertions are not re-runnable, and a flaky output cannot be re-analysed.

```bash
OUT=/tmp/out_$$           # unique per shell so parallel runs do not clash
ros2 launch <pkg> <node-under-test>.launch.py use_sim_time:=true &
NODE_PID=$!

ros2 bag record -s mcap -o "$OUT" /odom /filter_output /tf &
REC_PID=$!

# Replay the input bag at real time (or faster, if the pipeline tolerates it)
ros2 bag play /workspaces/rclpy-from-zero-to-hero/ros2_src/<pkg>/bag/<scenario>/ \
  --clock --rate 1.0

# Once playback ends, drain and stop
kill "$REC_PID" "$NODE_PID"
wait "$REC_PID" 2>/dev/null

# Post-process the recorded mcap; helper prints PASS/FAIL and exits 0/1
python plans/scripts/assert_filter_rmse.py "$OUT" --max-rmse 1.5
```

The recorded `$OUT/*.mcap` is the **artefact** for that criterion — list it under `## Artefacts` in the execution record. If a reviewer disputes the verdict they can replay `$OUT` and re-run the helper.

## Per-node-kind acceptance-criteria templates

Each phase file copies (or references) the matching subsection below. **Cite source-file line numbers when a threshold is a parameter** so the test stays in sync if the default changes.

### Pub/sub

- Publisher up: `ros2 topic info <topic>` reports `Publisher count >= 1`.
- Subscriber connected: `ros2 topic info <topic>` reports `Subscription count >= 1` (where the launch wires endpoints together).
- Rate close to declared: `ros2 topic hz <topic>` within ±20% of the timer period defined in source.
- Counter monotonicity / no drops: helper `plans/scripts/assert_message_count.py` over a 5 s window confirms expected_count ±5%.
- Namespace remap: when remappings are used, `ros2 topic info` shows the remapped topic carrying both endpoints (not the source-default names).

### QoS / threading

- QoS contract: `ros2 topic info -v <topic>` shows the matching `Reliability` / `Durability` / `History` profile on both ends.
- BEST_EFFORT ↔ RELIABLE incompatibility: publisher `Subscription count` stays at `0` for a deliberately mismatched subscriber (negative test).
- ReentrantCallbackGroup overlap: in the log, two callback start timestamps overlap within the same wall-clock millisecond (or the second starts before the first prints "done").
- MutuallyExclusiveCallbackGroup serial ordering: in the log, callback B's start is strictly after callback A's end for every interleaving pair (no overlap permitted).
- MultiThreadedExecutor liveness: `ros2 node info <node>` lists multiple subscriptions; under load (publisher faster than callback can complete) the node does not deadlock — verified by `assert_message_count.py` returning PASS on at least one of the callbacks.

### Controller

- Final position error: ‖p_final − p_goal‖ < `goal_dist_tolerance`.
  - Reference: [ros2_src/ch3/ch3/closed_loop_pid.py:63](../ros2_src/ch3/ch3/closed_loop_pid.py#L63) — `self.goal_dist_tolerance = 0.1  # meters`.
  - Reference: [ros2_src/projects/bug_planning/bug_planning/bug2_planner.py:53](../ros2_src/projects/bug_planning/bug_planning/bug2_planner.py#L53) — `self.goal_dist_tolerance: float = 0.02` (m).
- Final heading error: |θ_final − θ_goal| < `goal_heading_tolerance`.
  - Reference: [ros2_src/ch3/ch3/closed_loop_pid.py:64](../ros2_src/ch3/ch3/closed_loop_pid.py#L64) — `self.goal_heading_tolerance = 10  # degrees`.
  - Reference: [ros2_src/projects/bug_planning/bug_planning/bug2_planner.py:54](../ros2_src/projects/bug_planning/bug_planning/bug2_planner.py#L54) — `self.goal_heading_tolerance: float = radians(10)`.
- Time-to-goal: bounded per phase (default 60 s; bug2 90 s — see bl7).
- `/cmd_vel` does not saturate the actuator limits for more than 5% of the run window (no rail-to-rail oscillation).
- Square-closure error (for ch3 hw3 square traversal): ‖p_end − p_start‖ < 0.3 m, total path within 30% of commanded length.

Use `plans/scripts/assert_pose_reaches_goal.py <bag> --goal x,y --tol <m>` to evaluate.

### Filter

- RMSE: per-axis position RMSE of `/filter_output` vs `/odom` ground truth, over the full input bag, < 1.5× the corresponding RMSE of the raw measurement source (or an absolute bound when an absolute reference is available).
- Covariance monotone: trace(P) decreases for the first 5 update steps after activation (filter is "learning"). A single non-monotone tick is allowed if covariance is being inflated by an injected outlier — log it.
- Finite values: **no `NaN` or `Inf`** anywhere in the recorded pose, velocity, or covariance over the entire bag. (Single most useful canary for a numerical regression.)
- Innovation gating: when applicable (EKF with chi-squared gate), the gated/rejected fraction lies in 0–20% on a clean bag.

Use `plans/scripts/assert_filter_rmse.py <bag> --max-rmse <m> --max-rmse-ratio 1.5`.

### Localization

- `map → odom` transform present: `ros2 run tf2_ros tf2_echo map odom --timeout 5` returns a transform within 5 s of activation.
- Localization error: ‖p_amcl − p_true‖ < 0.5 m once converged.
- Pose covariance bounded: trace of `geometry_msgs/PoseWithCovarianceStamped.pose.covariance` 3-of-6 position block < initial trace (filter has converged).
- For EKF localization variants (bl7): unknown-correspondences association accuracy ≥ 80% against the known-correspondences oracle; MHT hypothesis count never exceeds 10.

### Planner

- Nav2 `/plan` published within 5 s of action goal acceptance (subscribe to `/plan` *before* sending the goal; assert at least one `nav_msgs/Path` arrives).
- `NavigateToPose` action returns `STATUS_SUCCEEDED` within 90 s for the canonical waypoint.
- Plan length < 2× straight-line distance between start and goal (no obvious detour).
- Bug2 FSM: no single mode (`GO_TO_GOAL`, `FOLLOW_BOUNDARY`, etc.) dwells > 30 s; final pose within `goal_dist_tolerance` of goal.

### Perception

- Topic produced: feeding a synthetic input (via `image_sidecar.py`) yields the expected output topic within 1 s.
- Positive frame: a yellow-line PNG → `/line_position` published with finite x,y; `/no_line_detected = False`.
- Negative frame: a fully black PNG → `/no_line_detected = True` (or `/line_position` not published, depending on node semantics).
- cv_bridge round-trip does not corrupt encoding: assertion helper compares input/output dimensions and encoding string.

### Lifecycle

- Every transition succeeds: `ros2 lifecycle set /<node> configure` → `SUCCESS`; same for `activate`, `deactivate`, `cleanup`, `shutdown`.
- State queryable: `ros2 lifecycle get /<node>` matches the expected state after each transition.
- Activation has functional effect: after `activate`, publishing the trigger topic produces a non-zero side-effect on the output topic within 2 s; after `deactivate`, the same trigger produces no output for at least 5 s.

### Service

- `ros2 service call /<srv> <type> <payload>` returns `success: true` within the per-phase timeout (default 60 s).
- Service appears in `ros2 service list` before the test client sends.
- On invalid payload (negative test, optional), the service rejects with `success: false` — does not crash the server.

### Action

- `ros2 action send_goal --feedback /<action> <type> <payload>` returns `STATUS_SUCCEEDED` within the per-phase timeout.
- At least **4 feedback messages** delivered between goal-accept and goal-complete (proves feedback channel is alive, not just terminal status).
- Cancellation path: send goal, wait 5 s, `ros2 action send_goal` with `--cancel` (or via a Python action-client) → server reports `STATUS_CANCELED` *and* the actuator command topic (`/cmd_vel` for drive actions) reaches zero within 1 s of cancellation.

## Python 3.12 / NumPy 2 import canary

The dominant class of regression in this port is `np.maximum_sctype`-style removal triggered at *import* time deep in `transforms3d` / `tf_transformations` / cv_bridge / OpenCV. Catch it in ~0.5 s, before any sim starts:

```bash
python -c "import <module>; print('canary ok')"
```

Concrete examples a phase file may use (substitute as appropriate):

```bash
python -c "import ch1.first_publisher; print('canary ok')"
python -c "import utils.tf_transformations; print('canary ok')"
python -c "import ekf_localization.ekf_localization_with_known_correspondences; print('canary ok')"
python -c "import project.detection; print('canary ok')"
```

If the canary fails, **stop the phase**; the runtime cannot start any of the nodes anyway and a launch attempt will just bury the traceback in a per-process log file. Log the failing import in `plans/errors/<short-name>.md` (cross-reference [plans/errors/fixed/tf_transformations-numpy2-incompatibility.md](errors/fixed/tf_transformations-numpy2-incompatibility.md) for the canonical template) and continue with the next phase only if the failing module is unreachable from there.

## Helper scripts catalog

The helpers below live (or will live) under `plans/scripts/`. They are **TO BE WRITTEN** as part of phase execution — they are intentionally **not** in scope for this plan-authoring step. Phase files reference them by name; the executor writes/adapts the helper when first exercising the phase that needs it. Each helper takes a recorded mcap (or live ROS args) plus a threshold, prints `PASS` / `FAIL` with the measured value, and exits `0` / `1`.

| Helper | Used by | Purpose |
|---|---|---|
| `assert_filter_rmse.py` | bl4, bl7 | Compute per-axis RMSE between `/filter_output` and `/odom` ground truth from a recorded mcap; check against `--max-rmse` / `--max-rmse-ratio`; flag any NaN/Inf in the pose/covariance stream. **TO BE WRITTEN.** |
| `assert_landmark_association.py` | bl7 (ekf_localization unknown-correspondences variant) | Compare landmark association IDs from an unknown-correspondences run against a known-correspondences oracle bag; check accuracy ≥ `--min-accuracy`. **TO BE WRITTEN.** |
| `assert_pose_reaches_goal.py` | bl3, bl6, bl7 (bug2) | Read final `/odom` (or `/amcl_pose`) from a recorded mcap; verify ‖p_final − p_goal‖ < `--tol` and |θ_final − θ_goal| < `--heading-tol`. **TO BE WRITTEN.** |
| `assert_message_count.py` | bl1, bl5a | Count messages on a topic over a `--duration` window; assert within ±`--tolerance-pct` of `--expected`. **TO BE WRITTEN.** |
| `image_sidecar.py` | bl6 | Publish a static PNG as `sensor_msgs/Image` via cv_bridge on a configurable topic and rate; used to feed `project.detection` with deterministic yellow-line / black frames. **TO BE WRITTEN.** |

Phase files may add additional one-off helpers, but anything reused across two or more phases belongs in this catalog.

## Common caveats

The following apply container-wide and must be reiterated in **each** phase's pre-flight block (copy the bullet that applies; do not assume the executor will read this doc end-to-end first):

- **GUI rendering blocked by container DRI / Qt.** No `gz sim` window, no `rqt_image_view`, no RViz display. Use the headless-Gazebo recipe above; treat any GUI-only acceptance criterion (e.g. "must show camera frames in rqt_image_view") as **N/A (env)** and capture the equivalent topic-level evidence instead. Already documented in [executions/smoke-test-b5-project-execution.md](executions/smoke-test-b5-project-execution.md) Issue C and the `jazzy_container_headless_gazebo` memory.
- **Container ships stale ROS packages.** April-2026 apt upgrades (`rclpy`, `rcl`, `fastcdr`, `rosidl-*`, `rmw-fastrtps-cpp`, `type-description-interfaces`, `example-interfaces`) must already be in place — applied during the B1 smoke test, see [executions/smoke-test-b1-ch1-execution.md](executions/smoke-test-b1-ch1-execution.md). If a fresh container is pulled, re-apply or bump the image before running any bl{N}.
- **`tf_transformations` numpy-2 crash is *known and deferred*** until source fixes land. Surface in the canary, do not re-investigate.
- **`detection.py` topic mismatch** (`/camera/image_raw` vs `/image_raw`) — see [smoke-test-b5-project.md](smoke-test-b5-project.md) "Known caveats"; bl6 must decide whether to fix or test both.
- **`nav2_navigation` is commented out** in `project/launch/nav2.launch.py` (lines 40-46). Full planning is not exercised by the default launch — bl6 either uncomments locally or includes `nav2_bringup/launch/bringup_launch.py` separately, then **does not** commit the change as part of the verification record.
- **Bag artefacts under `/tmp/`** are not persisted across container restarts. The execution record under `## Artefacts` lists their paths for the duration of the run only; nothing in `/workspaces/` is rewritten by these tests.
- **Rosbags must be played with `--clock --rate 1.0`** unless the phase explicitly notes otherwise; mismatched clock + node timestamps produce silent RMSE blow-ups that look like a filter regression.

## On reading the rest of the plan

- The eight per-phase files are `behavior-test-bl1-pub-sub-basics.md` through `behavior-test-bl7-advanced-projects.md`; see the table in [/home/georg/.claude/plans/i-m-working-on-porting-glistening-gray.md](../../../.claude/plans/i-m-working-on-porting-glistening-gray.md) for the mapping of phase → packages.
- The execution-record skeleton lives at [executions/behavior-test-template.md](executions/behavior-test-template.md); copy it to `executions/behavior-test-bl{N}-<group>-execution.md` per run.
- Style guides for any new incident report: [errors/fixed/tf_transformations-numpy2-incompatibility.md](errors/fixed/tf_transformations-numpy2-incompatibility.md), [errors/fixed/project-world-ambient-color-out-of-range.md](errors/fixed/project-world-ambient-color-out-of-range.md).
