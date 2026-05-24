# Smoke Test Execution — Phase B6 (`ekf_localization`)

**Date:** 2026-05-16
**Executor:** Claude Code (Opus 4.7, 1M context)
**Container image:** `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:iron-desktop-terminal-nvidia` (running ROS 2 Jazzy after B1 apt upgrades)
**Branch:** `jazzy-migration`
**Plan:** [../smoke-test-b6-ekf_localization.md](../smoke-test-b6-ekf_localization.md)

## Carry-over context from earlier phases

- **B1:** `ros-jazzy-example-interfaces` installed; `rclpy/rcl/rmw/rosidl/fastcdr/type-description-*` upgraded ([smoke-test-b1-ch1-execution.md](smoke-test-b1-ch1-execution.md)).
- **B3-B5/MEMORY:** GUI rendering blocked by container DRI permissions (`libEGL warning: failed to open /dev/dri/renderD128`); verify gz via topic graph + `/clock`, not window appearance. **Observed again in B6** — non-fatal; gz sim server runs headlessly.
- **B5/B4:** `tf_transformations` (`transforms3d 0.4.1` + numpy 2) crash documented in [../errors/tf_transformations-numpy2-incompatibility.md](../errors/tf_transformations-numpy2-incompatibility.md). **Pre-verified out of scope for B6**: `landmark_spawner` does not import `tf_transformations`; the EKF estimator nodes that do are not in this launch.
- **B5:** `project.world` `<ambient>` colour bug documented in [../errors/project-world-ambient-color-out-of-range.md](../errors/project-world-ambient-color-out-of-range.md). **Not relevant to B6** — this launch uses `turtlebot3_gazebo/worlds/empty_world.world`, which loads cleanly.

## Pre-flight

```bash
$ colcon build --packages-select ekf_localization utils
Starting >>> ekf_localization
Starting >>> utils
Finished <<< utils [1.11s]
Finished <<< ekf_localization [1.25s]
Summary: 2 packages finished [1.34s]

$ for p in ros_gz_sim ros_gz_bridge ros_gz_interfaces turtlebot3_gazebo turtlebot3_teleop; do
    echo -n "$p: "; ros2 pkg prefix "$p"; done
ros_gz_sim: /opt/ros/jazzy
ros_gz_bridge: /opt/ros/jazzy
ros_gz_interfaces: /opt/ros/jazzy
turtlebot3_gazebo: /opt/ros/jazzy
turtlebot3_teleop: /opt/ros/jazzy

$ ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/robot_state_publisher.launch.py
/opt/ros/jazzy/share/turtlebot3_gazebo/launch/robot_state_publisher.launch.py
$ ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
/opt/ros/jazzy/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
$ ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/empty_world.world
/opt/ros/jazzy/share/turtlebot3_gazebo/worlds/empty_world.world

$ which xterm
/usr/bin/xterm

$ export TURTLEBOT3_MODEL=burger
$ export DISPLAY=:0
```

`config/landmarks.yaml` length: **130 entries** (some duplicates expected per plan).

## Launch behavior

Background-launched with `nohup ros2 launch ekf_localization simulation.launch.py > /tmp/ekf_sim_launch.log 2>&1 &`. Waited ~40 s.

### Process tree

All seven actions registered processes successfully:

```
[INFO] [gazebo-1]: process started with pid [14645]
[INFO] [parameter_bridge-2]: process started with pid [14646]    # clock bridge
[INFO] [robot_state_publisher-3]: process started with pid [14648]
[INFO] [create-4]: process started with pid [14649]              # TB3 spawner (ros_gz_sim::create)
[INFO] [parameter_bridge-5]: process started with pid [14650]    # TB3 sensor/odom/cmd_vel bridge
[INFO] [landmark_spawner-6]: process started with pid [14651]
[INFO] [xterm-7]: process started with pid [14652]
```

### Robot spawn (TB3)

```
[create-4] [INFO] [...] [ros_gz_sim]: Requesting list of world names.
[create-4] [INFO] [1778940390.775947301] [ros_gz_sim]: Entity creation successful.
[INFO] [create-4]: process has finished cleanly [pid 14649]
```

TB3 burger spawned at the origin. Confirmed by `gz model --list`:

```
Available models:
    - ground_plane
    - burger
```

### Landmark spawner — STUCK FOREVER

```
[landmark_spawner-6] [INFO] [1778940389.457480266] [landmark_spawner]: service not available, waiting again...
[landmark_spawner-6] [INFO] [1778940390.459341571] [landmark_spawner]: service not available, waiting again...
...    (repeats every 1 s for the full 40 s+ observation window)
```

**Zero** `Spawned: cylinder_N` lines were ever emitted. `gz model --list` shows **0 cylinders**.

This is **not** the race condition mentioned in the plan ("re-run if it happens once"). It reproduces every run because the launch is structurally missing a ROS↔gz **service** bridge:

```
$ ros2 service info /world/default/create
Type: ros_gz_interfaces/srv/SpawnEntity
Clients count: 1     ← landmark_spawner
Services count: 0    ← nothing offering it on the ROS side

$ gz service -l | grep create
/world/default/create
/world/default/create/blocking
/world/default/create_multiple
/world/default/create_multiple/blocking
```

The service exists only on `gz-transport`; the launch's `parameter_bridge` instances bridge **topics** only (`/clock`, `/scan`, `/odom`, `/cmd_vel`, `/imu`, `/joint_states`, `/tf`). No `service:` block is wired, and there is no `ros_gz_sim::create` invocation in `landmark_spawner` analogous to the one in TB3's `spawn_turtlebot3.launch.py`. Full diagnosis + fix options in [../errors/ekf_localization-missing-world-create-service-bridge.md](../errors/ekf_localization-missing-world-create-service-bridge.md).

### Teleop (xterm)

```
[INFO] [xterm-7]: process started with pid [14652]
[xterm-7] xterm: cannot load font "-misc-fixed-medium-r-semicondensed--13-120-75-75-c-60-iso10646-1"
```

xterm font warning is benign (Debian-slim base lacks the legacy bitmap fonts). The teleop process **is** running underneath:

```
ros_user 14652 14641  0 14:06 ?     xterm -e /opt/ros/jazzy/lib/turtlebot3_teleop/teleop_keyboard --ros-args -r __node:=teleop
ros_user 14673 14652  1 14:06 pts/1 /usr/bin/python3 /opt/ros/jazzy/lib/turtlebot3_teleop/teleop_keyboard --ros-args -r __node:=teleop
```

`/teleop` shows up in `ros2 node list`. Without an interactive xterm window we cannot send keypresses, but teleop *would* publish — and via `/cmd_vel` (`geometry_msgs/msg/TwistStamped` per MEMORY — confirmed by `ros2 topic info`).

### Other warnings (non-fatal)

```
[gazebo-1] QStandardPaths: error creating runtime directory '/run/user/1000' (No such file or directory)
[gazebo-1] libEGL warning: failed to open /dev/dri/renderD128: Permission denied
[gazebo-1] libEGL warning: failed to open /dev/dri/card1: Permission denied
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="turtlebot3_burger"]/.../gz_frame_id]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF.
```

All are the now-familiar headless-container quirks (B1/B3 memory). gz sim runs to completion server-side.

## Acceptance criteria

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | Launch starts cleanly. gz sim window opens with the empty world. | **PASS (server) / N/A (window)** | All seven launch actions started without crash. `gz sim` runs headless; no GUI verifiable in container (B1 known caveat). `/clock` advances. |
| 2 | TB3 burger appears in gz at the spawn pose. | **PASS** | `gz model --list` shows `burger`; `[create-4]` reports "Entity creation successful"; `/odom` reports `position.x ≈ 0`. |
| 3 | Multiple landmark cylinders appear in gz. | **FAIL** | `gz model --list \| grep -ic cylinder` → **0**. `landmark_spawner` looped on "service not available" indefinitely. New error file created: [../errors/ekf_localization-missing-world-create-service-bridge.md](../errors/ekf_localization-missing-world-create-service-bridge.md). |
| 4 | `ros2 node list` contains `/parameter_bridge`, `/landmark_spawner`, `/teleop`, `/robot_state_publisher`. | **PASS** | All four present (two `/ros_gz_bridge` from the clock bridge and the TB3 sensor bridge; plus `/landmark_spawner`, `/robot_state_publisher`, `/teleop`). |
| 5 | `ros2 topic list` contains `/clock`, `/scan`, `/odom`, `/cmd_vel`, `/joint_states`, `/tf`, `/tf_static`. | **PASS** | All present. Also `/imu`, `/robot_description`, `/parameter_events`, `/rosout`. |
| 6 | `ros2 topic hz /clock` non-zero. | **PASS** | `average rate: 1972.596` (sim-time clock running ~2 kHz, normal for headless `-r` mode). |
| 7 | `ros2 topic hz /scan` non-zero. | **PASS** | `average rate: 4.907` Hz (TB3 LDS-01 nominal ~5 Hz). |
| 8 | `ros2 topic hz /odom` non-zero. | **PASS** | `average rate: 49.467` Hz. |
| 9 | `view_frames` shows the standard TB3 tree. | **PASS** | Generated `/tmp/frames_2026-05-16_14.09.23.pdf`. Tree: `odom → base_footprint → base_link → {wheel_left_link, wheel_right_link, caster_back_link, imu_link, base_scan}` (no map→odom, as expected — no localizer in this launch). |
| 10 | `/world/default/create` service is listed. | **PASS (listed) / FAIL (usable)** | `ros2 service list` shows it; `ros2 service info` reports `Clients count: 1, Services count: 0` — the service is *listed* (because a client is registered) but has **no ROS-side server**, so it is unreachable from rclpy. Marking PASS on a strict reading of "listed", but the criterion's intent (the service being callable) is not met. |

**Score: 9 / 10 PASS** (with criterion 10 a borderline pass on technicality).

## Overall verdict

**PARTIAL FAIL** — TB3 simulation, sensors, TF, teleop, clock all work; **landmark spawning is structurally broken** by a missing service bridge between `ros_gz_bridge` and `landmark_spawner`. Because landmarks are the entire purpose of the EKF localization package, this is a meaningful failure even though most launch infrastructure is healthy.

## Issues found (beyond known bugs)

1. **NEW: Missing ROS↔gz service bridge for `/world/default/create`.** Documented in [../errors/ekf_localization-missing-world-create-service-bridge.md](../errors/ekf_localization-missing-world-create-service-bridge.md). The launch wires three topic bridges (clock + two TB3 bridges) but no service bridge. `landmark_spawner.py` uses `rclpy.create_client(SpawnEntity, '/world/default/create')`, which has no server to talk to — the gz-transport service is not exposed to ROS. Same pattern will surface in B5's `project/spawn_robot.py` once its earlier blockers are fixed.

2. **Plan caveat about race condition is incorrect for this state.** The plan says "if 'service not available' appears, re-run". In this environment it is not transient — it reproduces 100% of the time. Recommend rewording the caveat in `smoke-test-b6-ekf_localization.md` once the missing-bridge fix lands.

3. **In-tree migration test is too weak.** `ros2_src/projects/ekf_localization/test/test_jazzy_migration.py` only asserts that the legacy `/spawn_entity` string is absent from `gazebo.py`. It does not verify the new endpoint actually has a server. Add a runtime assertion (or at least a launch-file grep for a service bridge) in a follow-up.

## Recommendations for plan adjustments

- Do **not** tick the B6 smoke checkbox at line 163 of `migrate-ros2_src-to-jazzy-plan-update.md` — landmark spawn is broken.
- Add an action item to B6's plan: "Wire a service bridge for `/world/default/create` (or convert `landmark_spawner` to call `ros_gz_sim::create` like TB3 does)." Block B6 acceptance on landmark count.
- Rewrite the "Known caveats — Race condition" bullet in `smoke-test-b6-ekf_localization.md` once the bridge is fixed; today's behaviour is not a race.
- When B5 is unblocked (after the world `<ambient>` fix + tf_transformations fix), expect the **same** service-bridge issue to bite `project/spawn_robot.py`. Same error file applies.

## Artifacts

- Launch log: `/tmp/ekf_sim_launch.log` (62 lines after the spawn loop began; landmark_spawner continues looping until kill).
- TF tree PDF: `/tmp/frames_2026-05-16_14.09.23.pdf`
- TF tree DOT:  `/tmp/frames_2026-05-16_14.09.23.gv`
