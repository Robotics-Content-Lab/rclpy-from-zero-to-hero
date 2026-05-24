# Smoke Test Execution Report — Phase B3 (ch2, ch3)

- **Date:** 2026-05-16
- **Executor:** Claude Code subagent (Opus 4.7, 1M context)
- **Container image:** `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia` (as in B1 run)
- **Branch:** `jazzy-migration`
- **Plan:** [plans/smoke-test-b3-ch2-ch3.md](../smoke-test-b3-ch2-ch3.md)
- **Container fixups carried over from B1:** apt upgrades for `rclpy/rcl/rmw/rosidl/fastcdr/type-description-*` and install of `ros-jazzy-example-interfaces` were already applied in this running container during the B1 run (see [smoke-test-b1-ch1-execution.md](smoke-test-b1-ch1-execution.md)). No further apt fixes were required for B3.

---

## Pre-flight

```text
$ colcon build --packages-select ch2 ch3
Starting >>> ch2
Starting >>> ch3
Finished <<< ch3 [0.94s]
Finished <<< ch2 [0.95s]
Summary: 2 packages finished [1.04s]

$ ros2 pkg prefix turtlebot3_gazebo
/opt/ros/jazzy

$ ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/
empty_world.launch.py
robot_state_publisher.launch.py
spawn_turtlebot3.launch.py
turtlebot3_dqn_stage{1..4}.launch.py
turtlebot3_house.launch.py
turtlebot3_world.launch.py

$ export TURTLEBOT3_MODEL=burger
```

- `ch2` and `ch3` both build cleanly under Jazzy.
- `turtlebot3_gazebo` was **already installed** at `/opt/ros/jazzy`; no apt install needed.
- Both required upstream launch files (`empty_world.launch.py`, `turtlebot3_world.launch.py`) are present.

Caveat: every `ros2` invocation prints `not found: ".../install/project/share/project/hook/hook_gazebo.sh"` — this comes from the (uninstalled) `project` package's stale `colcon.pkg` hook in the overlay and does not affect ch2/ch3. Unrelated to B3 scope.

---

## Test 1 — `ros2 launch ch2 simulation.launch.py`

Launched in background, logged to `/tmp/ch2_launch.log`, 20 s settle window. Default args (`world_file:=empty_world`). Headless (libEGL warnings — GUI does not render in container; sim itself runs fine).

### Launch processes spawned

| Process | Notes |
|---|---|
| `gazebo-1` (`gz sim -r -s -v2 .../empty_world.world`) | Server side |
| `gazebo-2` (`gz sim -g -v2`) | GUI side; QT runs but Ogre2 has no GPU access |
| `create-3` | One-shot `ros_gz_sim` entity-creation node; exits cleanly after spawn |
| `parameter_bridge-4` | `ros_gz_bridge` — sets up 7 bridges (see below) |
| `robot_state_publisher-5` | Publishes URDF + static TF |

Bridge creation log lines:
```
clock      (gz.msgs.Clock)      -> clock      (rosgraph_msgs/msg/Clock)
joint_states (gz.msgs.Model)    -> joint_states (sensor_msgs/msg/JointState)
odom       (gz.msgs.Odometry)   -> odom       (nav_msgs/msg/Odometry)
tf         (gz.msgs.Pose_V)     -> tf         (tf2_msgs/msg/TFMessage)
cmd_vel    (geometry_msgs/msg/TwistStamped) -> cmd_vel (gz.msgs.Twist)   [ROS->GZ]
imu        (gz.msgs.IMU)        -> imu        (sensor_msgs/msg/Imu)
scan       (gz.msgs.LaserScan)  -> scan       (sensor_msgs/msg/LaserScan)
```
`create-3` reported `Entity creation successful.` — TB3 spawned.

### `ros2 node list`

```
/robot_state_publisher
/ros_gz_bridge
```

(`gz sim` server is alive but exposes no ROS node — graph reflects the two ROS-native nodes only. Matches plan's "skeleton" expectation.)

### `ros2 topic list`

```
/clock              /scan
/cmd_vel            /tf
/imu                /tf_static
/joint_states       /parameter_events
/odom               /rosout
/robot_description
```

All 9 plan-required topics present.

### Per-criterion result table

| Criterion | Expected | Observed | Pass? |
|---|---|---|---|
| Launch starts, no Python tracebacks | clean | clean | ✅ |
| Gazebo Harmonic window opens with TB3 | GUI visible | headless (libEGL DRI permission denied); sim runs, GUI is non-functional | ⚠ headless |
| `/clock` non-zero | non-zero | **average rate: 986.87 Hz**, 4937-msg window | ✅ |
| `/scan` non-zero | ~5 Hz | **average rate: 4.93 Hz** | ✅ |
| `/odom` non-zero | >0 Hz | **average rate: 49.35 Hz** | ✅ |
| `/imu` non-zero | ~200 Hz | **average rate: 197.6 Hz** | ✅ (bonus) |
| `/cmd_vel` has ≥1 subscriber | ≥1 | `Subscription count: 1`, type **`geometry_msgs/msg/TwistStamped`** | ✅ (type differs from plan — see Issues) |
| All 9 topics listed | yes | yes | ✅ |
| TF tree matches expected | `odom → base_footprint → base_link → {wheels, caster, imu, scan}` | matches exactly (see below) | ✅ |

### TF tree captured

`ros2 run tf2_tools view_frames` produced `/tmp/frames_2026-05-16_08.57.31.pdf` (16 126 B). Frame yaml (trimmed):

```
base_footprint:    parent: 'odom'           rate: 50.20 Hz
base_link:         parent: 'base_footprint' rate: 10000 Hz (static)
wheel_left_link:   parent: 'base_link'      rate: 20.20 Hz
wheel_right_link:  parent: 'base_link'      rate: 20.20 Hz
caster_back_link:  parent: 'base_link'      rate: 10000 Hz (static)
imu_link:          parent: 'base_link'      rate: 10000 Hz (static)
base_scan:         parent: 'base_link'      rate: 10000 Hz (static)
```

Tree is exactly the one the plan expects, no orphaned frames.

**ch2 verdict: PASS** (headless caveat noted — sim+topics behave correctly without a GUI).

Cleanup: SIGINT to `ros2 launch`, then SIGTERM/SIGKILL to remaining `gz sim` / bridge processes. All processes confirmed dead before Test 2 started.

---

## Test 2 — `ros2 launch ch3 simulation.launch.py`

Launched in background, logged to `/tmp/ch3_launch.log`, 20 s settle window. Default args. Headless.

### Launch processes spawned

Identical set to ch2 (`gazebo-1`, `gazebo-2`, `create-3`, `parameter_bridge-4`, `robot_state_publisher-5`). This is expected — `ch3/launch/simulation.launch.py` is a thin pass-through over `ch2/launch/simulation.launch.py`. Bridge log lines and `Entity creation successful.` mirror ch2.

### `ros2 node list`

```
/robot_state_publisher
/ros_gz_bridge
```

### `ros2 topic list`

```
/clock              /scan
/cmd_vel            /tf
/imu                /tf_static
/joint_states       /parameter_events
/odom               /rosout
/robot_description
```

All 9 plan-required topics present.

### Per-criterion result table

| Criterion | Expected | Observed | Pass? |
|---|---|---|---|
| Launch starts, no Python tracebacks | clean | clean | ✅ |
| Gazebo Harmonic window opens with TB3 | GUI visible | headless (libEGL); sim runs | ⚠ headless |
| `/clock` non-zero | non-zero | **average rate: 981.00 Hz**, 4910-msg window | ✅ |
| `/scan` non-zero | ~5 Hz | **average rate: 4.91 Hz** | ✅ |
| `/odom` non-zero | >0 Hz | **average rate: 49.11 Hz** | ✅ |
| `/imu` non-zero | ~200 Hz | **average rate: 196.3 Hz** | ✅ (bonus) |
| `/cmd_vel` has ≥1 subscriber | ≥1 | `Subscription count: 1`, type **`geometry_msgs/msg/TwistStamped`** | ✅ (type differs from plan) |
| All 9 topics listed | yes | yes | ✅ |
| TF tree matches expected | as above | matches exactly | ✅ |

### TF tree captured

`/tmp/frames_ch3.pdf` (16 121 B) + `/tmp/frames_ch3.gv` (1387 B). Frame yaml (trimmed):

```
base_footprint:    parent: 'odom'           rate: 50.20 Hz
base_link:         parent: 'base_footprint' rate: 10000 Hz (static)
wheel_left_link:   parent: 'base_link'      rate: 20.21 Hz
wheel_right_link:  parent: 'base_link'      rate: 20.21 Hz
caster_back_link:  parent: 'base_link'      rate: 10000 Hz (static)
imu_link:          parent: 'base_link'      rate: 10000 Hz (static)
base_scan:         parent: 'base_link'      rate: 10000 Hz (static)
```

Identical structure to ch2 — confirms the pass-through inclusion works.

**ch3 verdict: PASS** (same headless caveat).

Cleanup: SIGINT then SIGKILL on stuck `gz sim` / `parameter_bridge` PIDs. All processes confirmed dead.

---

## Overall verdict: **PASS**

Both `ros2 launch ch2 simulation.launch.py` and `ros2 launch ch3 simulation.launch.py` start cleanly under Jazzy + Gazebo Harmonic, spawn the expected node/topic graph, and produce the expected TF tree. Migration of the B3 launch wrappers is complete.

---

## Issues found

1. **`/cmd_vel` type is `geometry_msgs/msg/TwistStamped`, not `geometry_msgs/msg/Twist`.**
   The plan's "Expected topics" table lists `/cmd_vel` as `geometry_msgs/msg/Twist`. Under Jazzy + `ros_gz_bridge`, the bridge advertises the ROS side as `TwistStamped` (it converts to `gz.msgs.Twist` on the Gazebo side). This is the upstream `turtlebot3_gazebo` Jazzy default and **not** a regression in the book's code. The plan's sanity-drive snippet
   ```
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```
   will **fail** with a type mismatch on Jazzy. It needs to become `geometry_msgs/msg/TwistStamped` with a populated `twist.linear.x`. This will also affect any chapter exercise code that publishes raw `Twist` to `/cmd_vel` — flag for Phase B3 of the chapter-code migration if not already covered.

2. **`/joint_state_publisher` is not in the graph.**
   The plan correctly anticipates this in the skeleton note ("Jazzy version typically delegates to gz-sim's joint state"). Confirmed: `/joint_states` is published by the gz-bridge (`gz.msgs.Model → sensor_msgs/msg/JointState`), not by `joint_state_publisher`. No action needed; just keep the skeleton-note phrasing.

3. **Gazebo GUI cannot render in this container.**
   `libEGL warning: failed to open /dev/dri/renderD128: Permission denied` and `/dev/dri/card0: Permission denied`. Likely missing `--device /dev/dri` or video-group permissions in the devcontainer. The sim itself runs (clock ticks at ~1 kHz, TB3 spawns, sensors publish), so this does not block the smoke test — but a teammate running the chapters interactively will see no Gazebo window until the container is fixed. This was the same situation in B1.

4. **Stale overlay `colcon.pkg` hook noise.**
   Every `ros2` invocation prints `not found: ".../install/project/share/project/hook/hook_gazebo.sh"`. The `project` package's old hook file got deleted in this working tree (`hook/hook_gazebo.sh` is in `git status` as D) but the overlay's generated setup still references it. Cosmetic, but worth a `rm -rf install/project` or a rebuild once `project` is migrated.

---

## Recommendations for plan updates

- Update the "Expected topics" row for `/cmd_vel` in `plans/smoke-test-b3-ch2-ch3.md`:
  ```
  | /cmd_vel | geometry_msgs/msg/TwistStamped | (Jazzy ros_gz_bridge default) |
  ```
- Update the sanity-drive snippet at the bottom of that plan to use `TwistStamped`:
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.1}}}"
  ```
- Add to "Known caveats": **Gazebo GUI is non-functional in headless / DRI-locked containers; sim still runs.** Acceptance criterion #1 ("Gazebo Harmonic window opens with TB3 visible") should be reframed as "Gazebo Harmonic server starts and `/clock` ticks" so it passes in headless CI.
- Add a heads-up that the wrappers do **not** spawn a `joint_state_publisher` node under Jazzy — joint states come from the gz bridge. (Plan already hints at this; make it explicit.)
- Carry-forward note: the `Twist → TwistStamped` change should be propagated to any chapter exercise in ch2/ch3 that publishes to `/cmd_vel` (closed-loop-P/PI/PID controllers, square-driver nodes). If those nodes still publish raw `Twist`, motion will silently not work even though the sim is healthy.

---

## Artefacts left on disk

- `/tmp/ch2_launch.log` — ch2 launch stdout/stderr
- `/tmp/ch3_launch.log` — ch3 launch stdout/stderr
- `/tmp/frames_2026-05-16_08.57.31.pdf` — ch2 TF tree
- `/tmp/frames_ch3.pdf`, `/tmp/frames_ch3.gv` — ch3 TF tree
