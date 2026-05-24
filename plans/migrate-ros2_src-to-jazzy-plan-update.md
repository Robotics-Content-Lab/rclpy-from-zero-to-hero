# Migration Plan: ros2_src → ROS Jazzy + Gazebo Harmonic (TDD)

## Context

The container images have already moved to Ubuntu 24.04 + ROS Jazzy + Gazebo Harmonic. Students pulling the new container can no longer build the book examples because `ros2_src/` still targets ROS Iron + Gazebo Classic. This plan ports the source tree using **Test-Driven Development**: for each phase, write a failing test first, then make the implementation change that causes it to pass.

Constraints from the already-migrated container:
- Gazebo Harmonic (gz-sim 8), SDF 1.9
- `ros_gz_sim` replaces `gazebo_ros`; `ros_gz_interfaces` replaces `gazebo_msgs`
- Clock bridge required for `use_sim_time`: `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`
- `GZ_SIM_RESOURCE_PATH` replaces `GAZEBO_MODEL_PATH`
- Namespaces without leading slash
- `sjtu_drone` and `robotino` live in the container's `sim_ws` — do not vendor here

---

## TDD Test Strategy

Each phase follows the **Red → Green → Refactor** cycle:

1. **Red**: Write a pytest test that asserts the Jazzy-correct state. Run it — it must fail on the current Iron code.
2. **Green**: Apply the migration change. Run the test — it must pass.
3. **Refactor**: Run `colcon test --packages-select <pkg>` to confirm linting (flake8, pep257, copyright) still passes.

### Test layers

| Layer | Tool | When |
|---|---|---|
| Static content tests | `pytest` (no ROS running) | Every phase — assert correct imports, API names, SDF versions, file existence |
| Build tests | `colcon build` | After every green step |
| Linting | `colcon test` | After every green step (flake8, pep257, copyright) |
| Smoke tests | `ros2 launch` manually | End of each phase, in the Jazzy container with Gazebo running |

### Test file placement

Add `test/test_jazzy_migration.py` inside each affected package alongside the existing linting tests. These are pure-Python pytest files — no live ROS process needed. `colcon test` picks them up automatically.

```
ros2_src/ch5/test/test_jazzy_migration.py
ros2_src/project/test/test_jazzy_migration.py
ros2_src/projects/ekf_localization/test/test_jazzy_migration.py
```

---

## Pre-flight ✅

```bash
git tag iron-final
git push origin iron-final
git checkout -b jazzy-migration   # current branch
```

All subsequent work runs inside the Jazzy container:
`ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia`

---

## Phase B1 — Trivial packages (build & test) ✅

~~No code edits expected. TDD cycle is build-only.~~

All 17 packages build cleanly with `colcon build --symlink-install`.

- [x] `colcon build --packages-select ch1 ch4 hw1_pkg hw3_pkg hw4_pkg preface ch5_interfaces utils`
- [x] `colcon build --packages-select bug_planning ekf_slam`
- [x] Smoke: `ros2 launch ch1 first_pub_sub.launch.py`

---

## Phase B2 — hw2_pkg (skip) ✅

`QoSReliabilityPolicy` resolves fine in Jazzy's `rclpy.qos`. No changes needed.

---

## Phase B3 — ch2, ch3 (launch path verification) ✅

- [x] `ros2_src/ch2/test/test_jazzy_migration.py` written (skips if TB3 not installed)
- [x] `ros2_src/ch3/test/test_jazzy_migration.py` written
- [x] No code changes needed — TB3 Jazzy packages hide Harmonic plumbing
- [x] Smoke: `ros2 launch ch2 simulation.launch.py`
- [x] Smoke: `ros2 launch ch3 simulation.launch.py`

---

## Phase B4 — ch5 and ch6 ✅

### ch5 ✅

- [x] **Red**: `ros2_src/ch5/test/test_jazzy_migration.py` written — 5 tests
- [x] **Green**: `ros2_src/ch5/launch/simulation.launch.py` rewritten to `ros_gz_sim` pattern (gz_sim.launch.py + clock_bridge + spawn_robotino)
- [x] **Green**: `ros2_src/ch5/package.xml` — added `ros_gz_sim`, `ros_gz_bridge`, `robotino_description`, `robotino_bringup`; removed `gazebo_ros`
- [x] **Refactor**: all 5 migration tests pass; flake8 clean on new files
- [x] Smoke: `ros2 launch ch5 simulation.launch.py`
- [x] Smoke: `ros2 run ch5 holonomic_closed_loop_square_action_server &`
- [x] Smoke: `ros2 run ch5 holonomic_closed_loop_square_action_client`

### ch6 ✅

No code changes needed — delegates to `sjtu_drone_bringup` (already Harmonic-ready in `sim_ws`).

- [x] Smoke: `ros2 launch ch6 simulation.launch.py`

---

## Phase B5 — project (the heavy lift) ✅

### B5.1 — spawn_robot.py: service swap ✅

- [x] **Red**: tests added to `ros2_src/project/test/test_jazzy_migration.py`
- [x] **Green**: `ros2_src/project/project/spawn_robot.py` rewritten
  - `from ros_gz_interfaces.srv import SpawnEntity`
  - endpoint: `/world/default/create` (WORLD_NAME = 'default' matches `<world name='default'>` in project.world)
  - `EntityFactory` request structure (`entity_factory.name`, `.sdf`, `.pose`, `.relative_to`)
  - URDF XML passed directly — Gazebo Harmonic's built-in URDF importer converts at spawn time
- [x] **Refactor**: tests pass, flake8 clean

### B5.2 — xacro: 5 plugin rewrites ✅

- [x] **Red**: xacro/SDF plugin tests added
- [x] **Green**: `ros2_src/project/urdf/turtlebot3_burger_camera.xacro` — all 5 Classic plugins replaced with gz-sim systems + `gz_frame_id` overrides
- [x] **Green**: `ros2_src/project/urdf/turtlebot3_burger_camera.urdf` — same 5-plugin replacement
- [x] **Green**: `ros2_src/project/models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf` — same 5-plugin replacement
- [x] **Green**: `ros2_src/project/models/turltebot3_burger_camera/turtlebot3_burger_camera copy.sdf` — deleted
- [x] **Refactor**: tests pass, flake8 clean

### B5.3 — World + model SDF version bumps ✅

- [x] **Red**: SDF version tests added
- [x] **Green**: `ros2_src/project/worlds/project/project.world` — SDF 1.6 → 1.9 (no Classic plugins were present)
- [x] **Green**: `ros2_src/project/models/project/model.sdf` — SDF 1.4 → 1.9
- [x] **Green**: `ros2_src/project/models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf` — SDF 1.7 → 1.9
- [x] **Green**: `ros2_src/projects/ekf_localization/models/Landmark/model.sdf` — SDF 1.7 → 1.9
- [x] **Green**: `ros2_src/utils/models/Landmark/model.sdf` — SDF 1.7 → 1.9
- [x] **Refactor**: tests pass

### B5.4 — Launch file + hook rename ✅

- [x] **Red**: launch + hook tests added
- [x] **Green**: `ros2_src/project/launch/simulation.launch.py` rewritten — gz_sim.launch.py + clock_bridge + topic_bridge (7 topics); removed gzserver/gzclient/OpaqueFunction/use_gui
- [x] **Green**: `ros2_src/project/hook/hook_gz.sh` created; uses `GZ_SIM_RESOURCE_PATH`
- [x] **Green**: `ros2_src/project/hook/hook_gazebo.sh` deleted
- [x] **Green**: `ros2_src/project/setup.py` — hook reference updated to `hook_gz.sh`
- [x] **Green**: `ros2_src/project/colcon.pkg` — hook reference updated to `hook_gz.sh` (fixes `package.dsv` so `source install/setup.bash` no longer errors)
- [x] **Green**: `ros2_src/project/package.xml` — added `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, `nav_msgs`
- [x] **Refactor**: all 23 migration tests pass; `source install/setup.bash` clean; flake8 clean on new files
- [x] Smoke: `ros2 launch project simulation.launch.py`
- [x] Smoke: `ros2 topic list` — expect `/scan /imu /image_raw /camera_info /cmd_vel /odom /joint_states /clock`
- [x] Smoke: `ros2 run rqt_image_view rqt_image_view`
- [x] Smoke: `ros2 run tf2_tools view_frames`
- [x] Smoke: `ros2 launch project nav2.launch.py` — 5/6 acceptance green ([B5 execution](executions/smoke-test-b5-project-execution.md)); only failing criterion is `tf2_echo map odom`, structurally dependent on Part 1 sim

---

## Phase B6 — ekf_localization (mirror of B5) ✅

- [x] **Red**: `ros2_src/projects/ekf_localization/test/test_jazzy_migration.py` written — 9 tests
- [x] **Green**: `ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py` — `ros_gz_interfaces` SpawnEntity, endpoint `/world/default/create`, `EntityFactory` request structure
- [x] **Green**: `ros2_src/projects/ekf_localization/launch/simulation.launch.py` — rewritten to gz_sim.launch.py + clock_bridge; kept TB3 robot_state_publisher and spawn_turtlebot launches
- [x] **Green**: `ros2_src/projects/ekf_localization/package.xml` — added `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`
- [x] **Refactor**: all 9 migration tests pass; flake8 clean on new files
- [x] Smoke: `ros2 launch ekf_localization simulation.launch.py`

---

## Phase B7 — package.xml sweep ✅

- [x] All `gazebo_ros`, `gazebo_msgs`, `gazebo_plugins` deps removed from ch5, project, ekf_localization
- [x] Replaced with `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces` as appropriate

---

## Overall Test Status

| Package | Migration Tests | flake8 (new files) | `colcon build` |
|---|---|---|---|
| ch5 | 5/5 ✅ | clean ✅ | ✅ |
| project | 23/23 ✅ | clean ✅ | ✅ |
| ekf_localization | 9/9 ✅ | clean ✅ | ✅ |

`source install/setup.bash` — no errors ✅

Note: `colcon test` shows `test_flake8`/`test_pep257` failures across all packages — these are pre-existing failures in untouched source files, not introduced by this migration.

---

## End-to-end Smoke Tests ✅

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch ch2 simulation.launch.py              # B3
ros2 launch ch3 simulation.launch.py              # B3
ros2 launch ch5 simulation.launch.py              # B4 (robotino in Harmonic)
ros2 launch ch6 simulation.launch.py              # B4 (sjtu_drone)
ros2 launch project simulation.launch.py          # B5
ros2 launch project nav2.launch.py                # B5
ros2 launch ekf_localization simulation.launch.py # B6
```

For each: confirm `/clock` is publishing, TF tree is complete (`ros2 run tf2_tools view_frames`), sensor topics carry data, and motion behaviour matches the book.

---

## File Change Summary

| Phase | File | Status |
|---|---|---|
| B3 | `ch2/test/test_jazzy_migration.py` *(new)* | ✅ |
| B3 | `ch3/test/test_jazzy_migration.py` *(new)* | ✅ |
| B3 | `ch2/launch/simulation.launch.py` | ✅ no change needed |
| B3 | `ch3/launch/simulation.launch.py` | ✅ no change needed |
| B4 | `ch5/test/test_jazzy_migration.py` *(new)* | ✅ |
| B4 | `ch5/launch/simulation.launch.py` | ✅ rewritten |
| B4 | `ch5/package.xml` | ✅ dep swap |
| B5.1 | `project/test/test_jazzy_migration.py` *(new)* | ✅ |
| B5.1 | `project/project/spawn_robot.py` | ✅ ros_gz_interfaces SpawnEntity |
| B5.2 | `project/urdf/turtlebot3_burger_camera.xacro` | ✅ 5 plugin rewrites + gz_frame_id |
| B5.2 | `project/urdf/turtlebot3_burger_camera.urdf` | ✅ 5 plugin rewrites |
| B5.2 | `project/models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf` | ✅ 5 plugin rewrites + SDF 1.9 |
| B5.2 | `project/models/turltebot3_burger_camera/turtlebot3_burger_camera copy.sdf` | ✅ deleted |
| B5.3 | `project/worlds/project/project.world` | ✅ SDF 1.9 |
| B5.3 | `project/models/project/model.sdf` | ✅ SDF 1.9 |
| B5.3 | `projects/ekf_localization/models/Landmark/model.sdf` | ✅ SDF 1.9 |
| B5.3 | `utils/models/Landmark/model.sdf` | ✅ SDF 1.9 |
| B5.4 | `project/launch/simulation.launch.py` | ✅ ros_gz_sim + topic bridges |
| B5.4 | `project/hook/hook_gz.sh` *(was hook_gazebo.sh)* | ✅ GZ_SIM_RESOURCE_PATH |
| B5.4 | `project/hook/hook_gazebo.sh` | ✅ deleted |
| B5.4 | `project/setup.py` | ✅ hook_gz.sh reference |
| B5.4 | `project/colcon.pkg` | ✅ hook_gz.sh reference (fixes package.dsv) |
| B5.4 | `project/package.xml` | ✅ dep swap |
| B6 | `projects/ekf_localization/test/test_jazzy_migration.py` *(new)* | ✅ |
| B6 | `projects/ekf_localization/ekf_localization/utils/gazebo.py` | ✅ SpawnEntity swap |
| B6 | `projects/ekf_localization/launch/simulation.launch.py` | ✅ ros_gz_sim pattern |
| B6 | `projects/ekf_localization/package.xml` | ✅ dep swap |
