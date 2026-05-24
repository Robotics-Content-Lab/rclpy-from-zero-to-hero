# Behavior Test — Phase BL3 (ch3 control + diff-drive odometry; hw3 trajectory)

**Phase:** BL3 (chapter-3 control & odometry; homework-3 trajectory)
**Packages:** `ros2_src/ch3`, `ros2_src/hw3_pkg`
**Tracks:** the BL3 row in [/home/georg/.claude/plans/i-m-working-on-porting-glistening-gray.md](../../../.claude/plans/i-m-working-on-porting-glistening-gray.md) — *"Open-loop drives N seconds at commanded velocity. P/PID converge ... Square closure error < 0.3 m. Diff-drive odometry traverse delta ≈ commanded delta ±30%."*

Verification scope: **business logic of every node in `ch3` + `hw3_pkg`**. Launch-level smoke (`smoke-test-b3-ch2-ch3.md`) is **assumed to already pass**; this phase confirms the controllers actually drive to the goal, the open-loop predictor matches commanded motion, and the kinematic odometry tracks ground truth.

All acceptance thresholds, the headless-Gazebo recipe, the rosbag capture pattern, and the import-canary boilerplate are taken from [behavior-test-conventions.md](behavior-test-conventions.md). Where a numeric threshold is a node parameter, the **source line** is cited inline — do not duplicate the value.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch2 ch3 hw3_pkg utils
source install/setup.bash

# Import canary (per conventions §"Python 3.12 / NumPy 2 import canary")
python -c "import ch3.closed_loop_pid; import ch3.closed_loop_p; import ch3.closed_loop_p_square; \
            import ch3.open_loop; import ch3.open_loop_square; import ch3.diff_drive_odometry; \
            import hw3_pkg.trajectory_controller; import hw3_pkg.odometry; \
            import utils.utils.math; print('canary ok')"

# TB3 dependency check (ch3 wraps ch2 which wraps turtlebot3_gazebo)
ros2 pkg prefix turtlebot3_gazebo
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/empty_world.launch.py
export TURTLEBOT3_MODEL=burger
```

**Headless Gazebo** (per conventions §"Headless Gazebo snippet"). `ch3/launch/simulation.launch.py` ([L16-L24](../ros2_src/ch3/launch/simulation.launch.py#L16-L24)) defers to `ch2`'s launch which defers to `turtlebot3_gazebo`'s — `gz_args` is not directly exposed. Two workarounds:

```bash
# Preferred: software GL + tolerate the GUI window (it never opens DRI under the container)
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ch3 simulation.launch.py world_file:=empty_world &

# Fallback if the TB3 launch insists on the GUI:
ros2 run ros_gz_sim gz_sim --headless-rendering -s empty_world.sdf &
# … then `ros2 launch ch2 simulation.launch.py` is **not** used; spawn the TB3
# manually via the spawn_robot pattern from project/.
```

Confirm gz-sim is alive (per conventions — `/clock` is the canary, **not** the gz window):

```bash
ros2 topic hz /clock         # non-zero → server running
ros2 topic list | grep -E '^/(odom|joint_states|cmd_vel)$'
#   expect: /odom /joint_states /cmd_vel  (all bridged by TB3 Jazzy launch)
ros2 topic info /odom        # publisher count >= 1
ros2 topic info /cmd_vel     # subscriber count >= 1 (gz parameter_bridge)
```

If any of those three topics is missing, **abort the phase** — the rest depends on them.

## Part 1 — Open-loop drive (`ch3 open_loop`, `ch3 open_loop_square`)

Sources: [ros2_src/ch3/ch3/open_loop.py](../ros2_src/ch3/ch3/open_loop.py), [ros2_src/ch3/ch3/open_loop_square.py](../ros2_src/ch3/ch3/open_loop_square.py).

**`open_loop`** computes a turn-then-drive plan from `(0,0)` to `(goal_x, goal_y)` (defaults `(2,3)` — [L31-L32](../ros2_src/ch3/ch3/open_loop.py#L31-L32)) using `max_vel=0.22`, `max_omega=2.84/4` ([L44-L45](../ros2_src/ch3/ch3/open_loop.py#L44-L45)); duration counters run at `hz=30`. **`open_loop_square`** defaults to `square_side=2.0`, `times=2` ([L15-L16](../ros2_src/ch3/ch3/open_loop_square.py#L15-L16)) and traces a square via timed straight + 90° spin segments.

### Verification (record-then-assert)

```bash
# Terminal 1 — simulation (must already be up from Pre-flight)
# Terminal 2:
OUT=/tmp/bl3_open_$$
ros2 bag record -s mcap -o "$OUT" /odom /cmd_vel &
REC=$!
ros2 run ch3 open_loop --ros-args -p goal_x:=1.0 -p goal_y:=0.0 &
NODE=$!
# Total commanded duration for (1,0): turn=0 + drive ≈ 1/0.22 ≈ 4.55 s; pad to 8 s.
sleep 8
kill -INT $NODE $REC 2>/dev/null
wait $REC 2>/dev/null

python plans/scripts/assert_pose_reaches_goal.py "$OUT" \
    --goal 1.0 0.0 --tol 0.30 --heading-tol 30
```

Repeat with `open_loop_square` and `square_side:=1.0 times:=1` — total square time ≈ `4*(side/max_vel) + 4*(pi/2/max_omega) ≈ 4*4.55 + 4*0.55 ≈ 20.4 s`; record for ~25 s.

### Acceptance (Part 1) — Controller template

- [ ] `open_loop` 1 m straight: traveled `|p_final − p_start|` within ±30% of `v_cmd * t_drive` (commanded velocity × drive-phase duration). Slip tolerance is wide because open-loop has no feedback.
- [ ] `open_loop` final position within 0.30 m of `(goal_x, goal_y)` after the planned schedule completes.
- [ ] `open_loop_square` total path length ≈ `4 × square_side` ± 30%; closure error `‖p_end − p_start‖` < 0.6 m (open-loop, no closure correction).
- [ ] No NaN / Inf anywhere in the recorded `/odom` or `/cmd_vel` stream.

## Part 2 — Closed-loop P controller (`ch3 closed_loop_p`)

Source: [ros2_src/ch3/ch3/closed_loop_p.py](../ros2_src/ch3/ch3/closed_loop_p.py). Subscribes `/odom`, publishes `/cmd_vel` ([L34-L38](../ros2_src/ch3/ch3/closed_loop_p.py#L34-L38)). Defaults: `goal_x=1.0, goal_y=1.0, goal_th=90.0°` ([L44-L46](../ros2_src/ch3/ch3/closed_loop_p.py#L44-L46)); gains `k_rho=0.3, k_alpha=0.8, k_beta=0.3` ([L48-L58](../ros2_src/ch3/ch3/closed_loop_p.py#L48-L58)); tolerances `goal_dist_tolerance=0.02 m` and `goal_heading_tolerance=10°` ([L75-L76](../ros2_src/ch3/ch3/closed_loop_p.py#L75-L76)). Node terminates on goal (`raise SystemExit`, [L173](../ros2_src/ch3/ch3/closed_loop_p.py#L173)).

### Verification

```bash
OUT=/tmp/bl3_p_$$
ros2 bag record -s mcap -o "$OUT" /odom /cmd_vel &
REC=$!
ros2 run ch3 closed_loop_p --ros-args -p goal_x:=2.0 -p goal_y:=0.0 -p goal_th:=0.0 &
NODE=$!
sleep 30                     # convergence budget
kill -INT $NODE $REC 2>/dev/null
wait $REC 2>/dev/null

python plans/scripts/assert_pose_reaches_goal.py "$OUT" \
    --goal 2.0 0.0 --tol 0.02 --heading-tol 10
#   tol matches goal_dist_tolerance @ closed_loop_p.py:75 (note: stricter than _pid's 0.1)
```

### Acceptance (Part 2) — Controller template

- [ ] Final position error < `goal_dist_tolerance` (= 0.02 m — [closed_loop_p.py:75](../ros2_src/ch3/ch3/closed_loop_p.py#L75)).
- [ ] Final heading error < `goal_heading_tolerance` (= 10° — [closed_loop_p.py:76](../ros2_src/ch3/ch3/closed_loop_p.py#L76)).
- [ ] Convergence time < 30 s.
- [ ] `/cmd_vel` does not saturate `max_vel=0.22` or `max_omega=2.84` for > 5% of the run (per conventions Controller template, rail-to-rail).

## Part 3 — Closed-loop PID controller (`ch3 closed_loop_pid`)

Source: [ros2_src/ch3/ch3/closed_loop_pid.py](../ros2_src/ch3/ch3/closed_loop_pid.py). Subscribes `/odom`, publishes `/cmd_vel` ([L39-L40](../ros2_src/ch3/ch3/closed_loop_pid.py#L39-L40)). Defaults: `goal_x=2.0, goal_y=3.0, goal_th=90.0°` ([L67-L69](../ros2_src/ch3/ch3/closed_loop_pid.py#L67-L69)); PID gains `rho={0.3,0.01,0.5}, alpha={0.8,0.01,0.5}, beta={0.05,0.01,0.5}` ([L75-L88](../ros2_src/ch3/ch3/closed_loop_pid.py#L75-L88)); tolerances **`goal_dist_tolerance=0.1 m`** ([L63](../ros2_src/ch3/ch3/closed_loop_pid.py#L63)) and **`goal_heading_tolerance=10°`** ([L64](../ros2_src/ch3/ch3/closed_loop_pid.py#L64)). Note these defaults are **looser** than `closed_loop_p`'s 0.02 m — match the per-node value, do not cross-cite.

### Verification

```bash
OUT=/tmp/bl3_pid_$$
ros2 bag record -s mcap -o "$OUT" /odom /cmd_vel /tf &
REC=$!
ros2 run ch3 closed_loop_pid --ros-args -p goal_x:=2.0 -p goal_y:=0.0 -p goal_th:=0.0 &
NODE=$!
sleep 30
kill -INT $NODE $REC 2>/dev/null
wait $REC 2>/dev/null

python plans/scripts/assert_pose_reaches_goal.py "$OUT" \
    --goal 2.0 0.0 --tol 0.1 --heading-tol 10
#   tol = closed_loop_pid.py:63 (0.1 m); heading = :64 (10°)
```

### Acceptance (Part 3) — Controller template + smoothness

- [ ] Final position error < `goal_dist_tolerance` (= 0.1 m — [closed_loop_pid.py:63](../ros2_src/ch3/ch3/closed_loop_pid.py#L63)).
- [ ] Final heading error < `goal_heading_tolerance` (= 10° — [closed_loop_pid.py:64](../ros2_src/ch3/ch3/closed_loop_pid.py#L64)).
- [ ] Convergence time < 30 s.
- [ ] **Smoothness bonus** (PID-specific): zero-crossings of `e_x = goal_x − odom.x` over the run < 5 (PID should not chatter). Compute in helper via `numpy.diff(np.sign(...))`; PID with the gains above should be visibly smoother than the P run from Part 2.
- [ ] `/cmd_vel` does not saturate `max_vel=0.22` or `max_omega=2.84` for > 5% of the run.

## Part 4 — Square trajectory (`ch3 closed_loop_p_square` via `ch3 square.launch.py`)

Source: [ros2_src/ch3/ch3/closed_loop_p_square.py](../ros2_src/ch3/ch3/closed_loop_p_square.py) + launch [ros2_src/ch3/launch/square.launch.py](../ros2_src/ch3/launch/square.launch.py). Launch is parameterized — `control_strategy:={open-loop,closed-loop}` ([L32-L36](../ros2_src/ch3/launch/square.launch.py#L32-L36)), `square_side` (default `1.0`, [L37-L41](../ros2_src/ch3/launch/square.launch.py#L37-L41)), `square_times` (default `1`, [L42-L46](../ros2_src/ch3/launch/square.launch.py#L42-L46)). Goals are the four corners `(s,0)→(s,s)→(0,s)→(0,0)` ([L76-L81](../ros2_src/ch3/ch3/closed_loop_p_square.py#L76-L81)); tolerances `0.02 m` / `10°` ([L69-L70](../ros2_src/ch3/ch3/closed_loop_p_square.py#L69-L70)). Node `raise SystemExit` after `times * 4` corners ([L155-L159](../ros2_src/ch3/ch3/closed_loop_p_square.py#L155-L159)).

### Verification

```bash
OUT=/tmp/bl3_square_$$
# Run the launch (it spawns sim too, but the launch already includes ch2/simulation.launch.py)
ros2 launch ch3 square.launch.py control_strategy:=closed-loop square_side:=1.0 square_times:=1 &
LAUNCH=$!

ros2 bag record -s mcap -o "$OUT" /odom /cmd_vel &
REC=$!

# Expected total time: 4 corners × (~side/k_rho + heading settle) ≈ 60–90 s for side=1, gains=0.3.
sleep 120
kill -INT $LAUNCH $REC 2>/dev/null
wait $REC 2>/dev/null

# Post-process: closure error + ∫|ω| ≈ 4 × π/2.
python plans/scripts/assert_pose_reaches_goal.py "$OUT" \
    --goal 0.0 0.0 --tol 0.30 --heading-tol 20
```

Closure metric needs a small ad-hoc Python check (use `pyrosbag2` / `mcap` to read `/odom`, then compute `‖p_end − p_start‖` and `∫|angular.z| dt`).

### Acceptance (Part 4) — Controller template + square-specific

- [ ] All four corners visited (helper detects the four goal-reached log lines, or sees `/cmd_vel` zero-crossings × 4).
- [ ] Closure error `‖p_end − p_start‖` < 0.3 m (per conventions Controller template, last bullet).
- [ ] Total commanded angular budget `∫|angular.z| dt` ≈ `4 × π/2` ± 20%.
- [ ] Total path length within ±30% of `4 × square_side`.
- [ ] No NaN / Inf in `/odom` or `/cmd_vel` over the full run.

## Part 5 — `diff_drive_odometry` (`ch3 diff_drive_odometry`)

Source: [ros2_src/ch3/ch3/diff_drive_odometry.py](../ros2_src/ch3/ch3/diff_drive_odometry.py). Subscribes `joint_states` ([L32-L37](../ros2_src/ch3/ch3/diff_drive_odometry.py#L32-L37)); computes `[x, y, theta]` from wheel-position deltas using `WHEEL_RADIUS=0.033`, `WHEEL_DISTANCE=0.160` ([L22-L23](../ros2_src/ch3/ch3/diff_drive_odometry.py#L22-L23)). **Does not publish** — pose lives in `self.pose` and is plotted via matplotlib. Frame: implicit world (no `tf` broadcast). Ground truth for comparison is `/odom` (published by gz-sim's diff-drive system, **not** by this node).

Verification therefore needs an in-process or sidecar comparison: log `self.pose` against the same-second `/odom` reading. Simplest path is to extend the node temporarily with a `print` of `(timestamp, x, y, theta)` and grep the launch output, OR write a tiny one-shot listener that subscribes `joint_states` itself, re-runs the same kinematic formula, and compares to `/odom`. **Do not commit either change** — they are run-time scaffolding only.

### Verification (sidecar approach, headless)

```bash
# 1. Drive the robot on a known path
OUT=/tmp/bl3_dodom_$$
ros2 bag record -s mcap -o "$OUT" /odom /joint_states /cmd_vel &
REC=$!

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1}, angular: {z: 0.0}}" &
PUB=$!
sleep 10                     # ~1 m of straight-line drive
kill -INT $PUB
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'   # stop
sleep 1
kill -INT $REC 2>/dev/null
wait $REC 2>/dev/null

# 2. Run the node's kinematic model offline against the recorded joint_states,
#    compare the final pose to /odom's final pose.
python plans/scripts/assert_pose_reaches_goal.py "$OUT" --goal 1.0 0.0 --tol 0.1
#   The helper needs a --diff-drive-odom mode to re-run the formula at L80-L89
#   of diff_drive_odometry.py against /joint_states; add as a flag.
```

Note: `diff_drive_odometry.py` discards any wheel-position delta with `|rho| < 1e-2` ([L84-L85](../ros2_src/ch3/ch3/diff_drive_odometry.py#L84-L85)), so the offline re-computation must do the same to match.

### Acceptance (Part 5)

- [ ] Drift between node's computed `(x, y)` and ground-truth `/odom (x, y)` < 10% of distance traveled over a 10 s straight-line open-loop drive (i.e. < 0.1 m for a ~1 m run).
- [ ] Heading drift < 10° over the same run.
- [ ] Update path active: `joint_states` callback fires (recorded bag contains `joint_states` messages at TB3 rate); pose array changes monotonically while `linear.x > 0`.

## Part 6 — `hw3_pkg` (odometry + trajectory_controller)

Sources: [ros2_src/hw3_pkg/hw3_pkg/odometry.py](../ros2_src/hw3_pkg/hw3_pkg/odometry.py), [ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py), [ros2_src/hw3_pkg/launch/hw3.launch.py](../ros2_src/hw3_pkg/launch/hw3.launch.py).

**`hw3_pkg.odometry`** (`VelocityOdometry`) maintains **three** pose estimates side-by-side: velocity model (from `cmd_vel`, [L172-L205](../ros2_src/hw3_pkg/hw3_pkg/odometry.py#L172-L205)), kinematic model (from `joint_states`, [L134-L170](../ros2_src/hw3_pkg/hw3_pkg/odometry.py#L134-L170)), and ground-truth `/odom` ([L114-L131](../ros2_src/hw3_pkg/hw3_pkg/odometry.py#L114-L131)). All three are plotted in matplotlib; **no ROS publisher** for the computed estimates (same pattern as Part 5). `WHEEL_RADIUS=0.033`, `WHEEL_DISTANCE=0.160` ([L45-L46](../ros2_src/hw3_pkg/hw3_pkg/odometry.py#L45-L46)).

**`hw3_pkg.trajectory_controller`** subscribes `/odom`, publishes `/cmd_vel` ([L56-L57](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L56-L57)). Drives a **5-pointed star** (vertex indices `[0,2,4,1,3,0]`, [L15-L27](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L15-L27)) using a `PIController` for distance (`kp=1.0, ki=0.1` — [L66-L67](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L66-L67)) and a `PIDController` for heading (`kp=1.6, ki=0.05, kd=0.3` — [L68-L70](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L68-L70)). FSM: `rotate` (turn until `|angular_error| < 0.02 rad` — [L124](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L124)), then `move` (drive until `distance_to_target < 0.1` — [L136](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L136)). Caveat: `time.sleep(5)` **before** `rclpy.init` ([L159](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L159)) — wait for the simulation to be up before starting the controller, but it also delays test orchestration by 5 s.

Launch [hw3.launch.py](../ros2_src/hw3_pkg/launch/hw3.launch.py) includes `ch2/simulation.launch.py` and spawns both `trajectory_controller` and `odometry` (no parameters exposed).

### Verification

```bash
OUT=/tmp/bl3_hw3_$$
ros2 launch hw3_pkg hw3.launch.py &
LAUNCH=$!
ros2 bag record -s mcap -o "$OUT" /odom /cmd_vel /joint_states &
REC=$!

# Star trajectory has 6 waypoints (5 corners + return to start), unit radius.
# Per-waypoint settle time ≈ 5–10 s ⇒ total ≈ 45–60 s + the 5 s sleep at L159.
sleep 75
kill -INT $LAUNCH $REC 2>/dev/null
wait $REC 2>/dev/null

# Verify the FSM cleared all 5 vertices: helper inspects /cmd_vel for the
# rotate→move alternation and counts the "Reached target N" log lines, OR
# uses the recorded /odom to count star-vertex visits within --tol 0.1.
python plans/scripts/assert_pose_reaches_goal.py "$OUT" \
    --goal 1.0 0.0 --tol 0.1 --heading-tol 15
#   Final goal of the star is index 5 = back to vertex 0 = (cos 0, sin 0) = (1, 0).
```

### Acceptance (Part 6) — Controller template + hw3-specific

- [ ] `/cmd_vel` published by `trajectory_controller` at the timer rate (0.1 s, [L82](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L82)).
- [ ] Star trajectory: all 5 vertices reached (FSM enters `move` then `rotate` 5 times — count "Reached target" log lines, or detect 5 distinct stop-points within 0.1 m of `(cos(angle_k), sin(angle_k))` in `/odom`).
- [ ] Time-to-finish < 60 s (after the 5 s `time.sleep` at [L159](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L159)).
- [ ] Final pose within `0.1 m` (the in-source `distance_to_target` threshold at [L136](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L136)) of the trajectory's last vertex.
- [ ] `hw3_pkg.odometry` velocity-model drift vs `/odom` ground truth < 30% over the full run (looser than Part 5 because it integrates noisy `/cmd_vel` without `joint_states` correction).
- [ ] `hw3_pkg.odometry` kinematic-model drift vs `/odom` < 10% over the full run (same kinematics as Part 5).
- [ ] No NaN / Inf anywhere in `/odom`, `/cmd_vel`, or the matplotlib pose arrays.

## Known caveats

- **`closed_loop_p` and `closed_loop_pid` have *different* default tolerances.** `closed_loop_p.py:75` uses `0.02 m`, but `closed_loop_pid.py:63` uses `0.1 m`. **Cite the per-node value** in the helper invocation; do not assume the controller-template default applies uniformly. The conventions doc Controller-template citation refers specifically to the PID node's `0.1 m`.
- **Square goal uses `compute_beta_sign(self.goal.z)` not the goal angle in radians** ([closed_loop_p_square.py:119](../ros2_src/ch3/ch3/closed_loop_p_square.py#L119)) — a minor inconsistency vs `closed_loop_p.py:124` which passes `goal_theta`. Not a regression; affects only the sign convention. If the square path traces clockwise instead of counter-clockwise, this is the line to inspect, but **do not fix as part of BL3** — it is pre-existing behavior.
- **No headless `gz_args` override in `ch3/simulation.launch.py`** (or `ch2/simulation.launch.py`, or `turtlebot3_gazebo/launch/empty_world.launch.py` upstream). The headless recipe relies on `LIBGL_ALWAYS_SOFTWARE=1` plus the launch tolerating a failed GUI window startup. See conventions §"Headless Gazebo snippet" — if the launch crashes when DRI is unavailable, drop back to invoking `gz_sim --headless-rendering -s …` directly and skip the TB3 launch.
- **`hw3_pkg.trajectory_controller` has `time.sleep(5)` before `rclpy.init`** ([trajectory_controller.py:159](../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py#L159)). Test orchestration must allow this latency on top of the per-vertex settle time.
- **`hw3_pkg.odometry` discards wheel-position deltas with `|rho| < 1e-2`** ([odometry.py:144-145](../ros2_src/hw3_pkg/hw3_pkg/odometry.py#L144-L145)) — when comparing to `/odom`, the helper's offline kinematic re-computation must replicate this filter or the residuals will look inflated.
- **`diff_drive_odometry` (Part 5) and `hw3_pkg.odometry` (Part 6) both call `matplotlib.pyplot.show()`/`pause()`** at every update. Under the headless container, this either no-ops or raises a Qt error — either is fine for the ROS-side acceptance, but if `pause()` raises, the callback never completes and pose stops updating. Treat a flat pose array as **N/A (env)** rather than a controller bug; capture the matplotlib error in the execution record.
- **`diff_drive_odometry` and `hw3_pkg.odometry` publish *no* ROS topics for the computed estimates.** They write to internal arrays + matplotlib. Verification must either (a) re-run the kinematic formula offline against the recorded `/joint_states`, or (b) temporarily add a `print(self.pose)` to the node and parse the log. Option (a) is preferred — do not commit option (b).
- **`/odom` is the ground-truth comparison source.** It comes from gz-sim's diff-drive system — the same one the controller subscribes to. There is no independent "true" pose; "drift" measurements compare the node's *re-derived* odometry against the gz-bridge `/odom`, which is itself integrated. Phrasing in the execution record should reflect this — the criterion is "node tracks the bridge's integrated pose", not "node matches ground truth".

## On pass

- Tick the BL3 row in [/home/georg/.claude/plans/i-m-working-on-porting-glistening-gray.md](../../../.claude/plans/i-m-working-on-porting-glistening-gray.md).
- Record execution as `plans/executions/behavior-test-bl3-ch3-hw3-execution.md` (copy from `executions/behavior-test-template.md`).
- Proceed to **[behavior-test-bl4-ch4-filters.md](behavior-test-bl4-ch4-filters.md)** — Kalman / EKF rosbag-driven verification. BL4 reuses no BL3 artefacts; only the canary + headless-Gazebo discipline carry over.
