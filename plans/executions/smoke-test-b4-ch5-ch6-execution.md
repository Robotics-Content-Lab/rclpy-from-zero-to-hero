# Smoke Test Execution — Phase B4 (`ch5`, `ch6`)

| Field | Value |
|---|---|
| Date | 2026-05-16 |
| Executor | Claude (Opus 4.7, 1M context) |
| Container image | `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:iron-desktop-terminal-nvidia` (now running Jazzy after B1 stack upgrade) |
| Branch | `jazzy-migration` |
| Plan | [smoke-test-b4-ch5-ch6.md](../smoke-test-b4-ch5-ch6.md) |
| Tracks (plan checkboxes) | Phase B4 "Smoke" lines 95-97 (ch5) and line 103 (ch6) in [migrate-ros2_src-to-jazzy-plan-update.md](../migrate-ros2_src-to-jazzy-plan-update.md) |

## Carry-over context from B1 / B3

1. `ros-jazzy-example-interfaces` is installed; `rclpy`/`rcl`/`rmw`/`rosidl`/`fastcdr`/`type-description-*` were upgraded to the April-2026 sync. `libfastcdr.so` is now `2.2.7` (was `2.2.5`). Stale `build/ch5_interfaces` CMakeCache forced a one-time clean rebuild of that package; documented below.
2. Gazebo Harmonic GUI cannot render in this container (DRI permission denied on `/dev/dri/render*`, `libEGL warning` repeated in the ch6 log). Sim *server* runs fine — `/clock` ticks, sensors stream messages. Verdict for "gz window opens" criteria is `N/A — known env limitation`.
3. `ros2 topic hz <topic>` hangs / produces no output in this Jazzy + numpy-2.4 stack (likely a `topic_statistics` / `rosidl_runtime_py` interaction). I substituted `ros2 topic echo <topic> --once` to verify each topic is actively publishing — for `/clock` we observed `sec: 41` rising to `sec: 53` over ~12 s, confirming sim time advances.

---

## Pre-flight

### Build

Initial build of `ch5_interfaces` failed:

```
gmake[2]: *** No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so.2.2.5',
        needed by 'libch5_interfaces__rosidl_typesupport_fastrtps_c.so'.  Stop.
```

The pre-existing `build/ch5_interfaces/CMakeCache.txt` pinned the old `libfastcdr.so.2.2.5` (Jazzy now ships `2.2.7`). After `rm -rf build/ch5_interfaces install/ch5_interfaces` the build completed cleanly:

```
Starting >>> ch5_interfaces
Starting >>> ch6
Finished <<< ch6 [0.89s]
Finished <<< ch5_interfaces [7.41s]
Starting >>> ch5
Finished <<< ch5 [0.82s]
Summary: 3 packages finished [8.31s]
```

No warnings. (Tracked for plan update: any other CMake package built before the April-sync upgrade may need a one-shot clean rebuild — e.g. `cv_bridge`/`pcl_conversions` consumers.)

### Package resolution

`ros2 pkg prefix` queried individually for each dependency (Jazzy refuses multi-arg form):

| Package | Status | Path |
|---|---|---|
| `ros_gz_sim` | OK | `/opt/ros/jazzy` |
| `ros_gz_bridge` | OK | `/opt/ros/jazzy` |
| **`robotino_description`** | **MISSING** | `Package not found` |
| `robotino_bringup` | OK | `/home/ros_user/sim_ws/install/robotino_bringup` |
| `sjtu_drone_bringup` | OK | `/home/ros_user/sim_ws/install/sjtu_drone_bringup` |
| `sjtu_drone_control` | OK | `/home/ros_user/sim_ws/install/sjtu_drone_control` |
| `xterm` | OK | `/usr/bin/xterm` |
| `DISPLAY` | OK | `:1` |

`robotino_description` is partially installed in `~/sim_ws/install/robotino_description/` but only the colcon hook scripts exist — no `ament_index/resource_index/packages/` registration and no `share/robotino_description/{urdf,models,worlds}`. The source tree (`~/sim_ws/src/robotino/robotino_description/`) is complete, but its `CMakeLists.txt` depends on Gazebo Classic packages (`gazebo_ros`, `gazebo_plugins`, `gazebo_dev`) which were uninstalled when the container moved to Harmonic. Rebuilding it in-place is out of scope for this smoke test (and requires source migration to Harmonic).

---

## Part 1 — `ros2 launch ch5 simulation.launch.py`

### Verdict: **FAIL — environment issue**

The launch is structurally correct (rewritten to `ros_gz_sim` pattern per Phase B4 Green steps), but cannot evaluate `xacro.process_file(...)` at parse time because `robotino_description` is not registered in `AMENT_PREFIX_PATH`.

### Per-criterion table

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | Launch starts; gz window opens; Robotino visible | FAIL | `PackageNotFoundError: "package 'robotino_description' not found"` at launch-description load; process exits before any node spawns |
| 2 | `ros2 node list` contains clock bridge + spawn_robotino | FAIL | No nodes started — see (1) |
| 3 | `ros2 topic list` contains `/clock` and robotino-namespaced topics | FAIL | No topics — see (1) |
| 4 | `ros2 topic hz /clock` non-zero | FAIL | No `/clock` publisher — see (1) |
| 5 | `ros2 run ch5 square_action_server` starts | **FAIL** | `transforms3d` raises `AttributeError: 'np.maximum_sctype' was removed in the NumPy 2.0 release` via `utils.utils.math` → `tf_transformations` → `transforms3d.quaternions` |
| 6 | `ros2 action list` shows `/drive_square` | FAIL | Server never reached `__init__` |
| 7 | `ros2 run ch5 square_action_client` sends goal; server accepts | PARTIAL | Client imports and runs (no traceback). Log: `[INFO] [drive_square_action_client]: Waiting for action server...` — but no server present, so no acceptance can be observed |

### Launch log excerpt — `/tmp/ch5_launch.log`

```
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught multiple exceptions
    when trying to load file of format [py]:
 - PackageNotFoundError: "package 'robotino_description' not found, searching: [...
   /home/ros_user/sim_ws/install/robotino_bringup', '/opt/ros/jazzy']"
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
```

### Action server / client interaction log

`square_action_server`:

```
File ".../holonomic_closed_loop_square_action_server.py", line 10, in <module>
    from utils.utils.math import euler_from_quaternion
File ".../utils/utils/math.py", line 2, in <module>
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
File "/opt/ros/jazzy/lib/python3.12/site-packages/tf_transformations/__init__.py", line 46, in <module>
    import transforms3d
File "/usr/lib/python3/dist-packages/transforms3d/quaternions.py", line 26, in <module>
    _MAX_FLOAT = np.maximum_sctype(np.float64)
AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release.
[ros2run]: Process exited with failure 1
```

Installed versions: `numpy 2.4.4` (pip, `/usr/local/lib`), `python3-transforms3d 0.4.1` (apt deb, `/usr/lib/python3/dist-packages`). `transforms3d 0.4.1` references `np.maximum_sctype` which numpy 2.0+ removed.

`square_action_client` (entry point in `ros2_src/ch5/ch5/square_action_client.py`):

```
[INFO] [1778922652.927000000] [drive_square_action_client]: Waiting for action server...
```

The client imports `ch5_interfaces.action.DriveSquareAction` only (no utils dep), so its entry point is healthy. It waits for a server that never appears.

---

## Part 2 — `ros2 launch ch6 simulation.launch.py`

### Verdict: **PARTIAL PASS**

All four acceptance criteria are satisfied at the *graph* level — gz sim server, sjtu_drone plugin, teleop node, and ros_gz_bridge all start; the topic table matches the plan; sim time advances. The gz GUI window does **not** render (DRI permission denied — env limitation per B1), and xterm's font load failed (cosmetic). Neither breaks the smoke acceptance.

### Per-criterion table

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | Launch starts; gz sim opens; xterm teleop opens | PARTIAL PASS | All processes launched (PIDs 34726-34733 incl. xterm, gazebo, robot_state_publisher, joint_state_publisher, create, parameter_bridge, static_transform_publisher); gz GUI cannot render (libEGL warning re `/dev/dri/render*` — known B1 env issue); xterm fails to load its fixed-13 font, the teleop node still runs but in a non-rendered xterm |
| 2 | `ros2 node list` contains `/teleop` + sjtu_drone_bringup nodes | PASS | Output: `/ros_gz_bridge`, `/simple_drone/drone_simple_controller`, `/simple_drone/joint_state_publisher`, `/simple_drone/robot_state_publisher`, `/static_transform_publisher_i19oJQhH2IBFCilK`, `/teleop` |
| 3 | `ros2 topic list` contains `/clock` + `simple_drone/cmd_vel`, `simple_drone/takeoff`, `simple_drone/land`, `simple_drone/sonar/out` | PARTIAL PASS | `/clock`, `/simple_drone/cmd_vel`, `/simple_drone/takeoff`, `/simple_drone/land` all present. **Sonar topic is `/simple_drone/sonar`, not `/simple_drone/sonar/out`** — sjtu_drone_bringup renames the gz `.../sonar/scan` to `.../sonar`. Plan table is inaccurate; topic itself exists with type `sensor_msgs/msg/Range` |
| 4 | `ros2 topic hz /clock` non-zero | PASS (via echo) | `ros2 topic hz` itself hangs in this env (numpy/jazzy interaction); `ros2 topic echo /clock --once` returns `sec: 41, nanosec: 391000000`, confirming sim time is advancing |

### Launch log excerpt — `/tmp/ch6_launch.log`

Process startup:

```
[INFO] [xterm-1]: process started with pid [34726]
[INFO] [gazebo-2]: process started with pid [34727]
[INFO] [robot_state_publisher-3]: process started with pid [34728]
[INFO] [joint_state_publisher-4]: process started with pid [34729]
[INFO] [create-5]: process started with pid [34731]
[INFO] [parameter_bridge-6]: process started with pid [34732]
[INFO] [static_transform_publisher-7]: process started with pid [34733]
```

Drone spawned:

```
[create-5] [INFO] [ros_gz_sim]: Requesting list of world names.
[create-5] [INFO] [ros_gz_sim]: Entity creation successful.
[INFO] [create-5]: process has finished cleanly [pid 34731]
[gazebo-2] [INFO] [simple_drone.drone_simple_controller]: The drone plugin is loading!
[gazebo-2] [INFO] [simple_drone.drone_simple_controller]: The drone plugin finished loading!
```

Bridges:

```
[parameter_bridge-6] Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)]
[parameter_bridge-6] Creating GZ->ROS Bridge: [.../sensor_imu/imu -> .../sensor_msgs/msg/Imu]
[parameter_bridge-6] Creating GZ->ROS Bridge: [.../front_camera/image -> .../sensor_msgs/msg/Image]
[parameter_bridge-6] Creating GZ->ROS Bridge: [.../down_camera/image -> .../sensor_msgs/msg/Image]
[parameter_bridge-6] Creating GZ->ROS Bridge: [.../navsat_sensor/navsat -> .../sensor_msgs/msg/NavSatFix]
[parameter_bridge-6] Creating GZ->ROS Bridge: [.../sonar/scan -> .../sensor_msgs/msg/Range]
```

Cosmetic issues (non-blocking):

```
[xterm-1] xterm: cannot load font "-misc-fixed-medium-r-semicondensed--13-120-75-75-c-60-iso10646-1"
[gazebo-2] libEGL warning: failed to open /dev/dri/renderD128: Permission denied
[gazebo-2] [Err] Could not resolve file [light.png]
[gazebo-2] [Err] Could not resolve file [console.png]
```

### Topic list (sorted)

```
/clock
/cmd_vel
/land
/parameter_events
/rosout
/simple_drone/bottom/camera_info
/simple_drone/bottom/image_raw
/simple_drone/cmd_mode
/simple_drone/cmd_vel
/simple_drone/dronevel_mode
/simple_drone/front/camera_info
/simple_drone/front/image_raw
/simple_drone/gt_acc
/simple_drone/gt_pose
/simple_drone/gt_vel
/simple_drone/imu
/simple_drone/joint_states
/simple_drone/land
/simple_drone/navsat
/simple_drone/odom
/simple_drone/posctrl
/simple_drone/reset
/simple_drone/robot_description
/simple_drone/sonar
/simple_drone/state
/simple_drone/takeoff
/takeoff
/tf
/tf_static
```

Topic info (each required topic):

| Topic | Type | Publishers | Subscribers |
|---|---|---|---|
| `/simple_drone/cmd_vel` | `geometry_msgs/msg/Twist` | 0 | 1 (drone plugin) |
| `/simple_drone/takeoff` | `std_msgs/msg/Empty` | 0 | 1 (drone plugin) |
| `/simple_drone/land` | `std_msgs/msg/Empty` | 0 | 1 (drone plugin) |
| `/simple_drone/sonar` | `sensor_msgs/msg/Range` | 1 (gz bridge) | 0 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 1 (gz bridge) | 2 (gazebo, internal) |

Note: the un-namespaced duplicates `/cmd_vel`, `/takeoff`, `/land` exist alongside the `simple_drone/*` versions — likely the teleop node publishes to the un-namespaced topics. That's an sjtu_drone_control behaviour, not a smoke-test failure.

`/simple_drone/odom` was confirmed publishing via `ros2 topic echo --once` (yielded `frame_id: /simple_drone/odom`, child_frame_id `/simple_drone/base_footprint`).

---

## Overall verdict

| Part | Verdict |
|---|---|
| Part 1 — ch5 launch | **FAIL** (environment: `robotino_description` not installed) |
| Part 1 — ch5 action server | **FAIL** (`transforms3d` / numpy-2.0 incompatibility via `utils.utils.math`) |
| Part 1 — ch5 action client | **PASS** (entry point imports and runs; can't roundtrip without server) |
| Part 2 — ch6 launch | **PARTIAL PASS** (graph correct, sim ticking; gz GUI / xterm rendering blocked by container display caveats from B1) |

---

## Issues found

### I1 (BLOCKER for ch5) — `robotino_description` not registered in ament index

The package is partially installed in `~/sim_ws/install/robotino_description` (only colcon hook scripts, no `ament_index/resource_index/packages/` registration, no share/urdf payload). Its source CMakeLists.txt depends on Gazebo Classic packages (`gazebo_ros`, `gazebo_plugins`, `gazebo_dev`) that no longer exist in this container. This is an **environment problem** in the `rclpy-from-zero-to-hero-container` repository, not a regression in ch5's migration. Until the container side ships a Harmonic-compatible `robotino_description`, ch5's launch cannot start regardless of how ch5's code looks.

### I2 (BLOCKER for ch5 action server, but wider scope) — `transforms3d 0.4.1` is broken on numpy 2.0+

`utils/utils/math.py` (and any node that imports `euler_from_quaternion` from there) crashes on import. Affects ch3, ch4, ch5 action server, ekf_localization, ekf_slam, bug_planning, project — anything using `tf_transformations`. Two fixes possible:

1. Upgrade `transforms3d` via pip: `pip install --break-system-packages "transforms3d>=0.4.2"` (0.4.2 ditched `np.maximum_sctype`).
2. Replace `utils.utils.math.euler_from_quaternion` with a numpy-free implementation (it's a 4-line math function — does not need transforms3d).

Recommend (2) so the workspace doesn't depend on a pip-overlay over a deb. The existing `utils/utils/math.py` already imports `math.atan2`/`sin`/`cos`/`sqrt`, so a local quaternion-to-euler helper is trivial.

### I3 (minor) — Stale CMakeCache after April sync

`build/ch5_interfaces/CMakeCache.txt` pinned `libfastcdr.so.2.2.5`. Other CMake packages built before B1 (`ch5_interfaces` is the only `ament_cmake` package in this repo, plus anything in `~/ros2_ws`) may need a one-shot clean rebuild. Recommend the plan add a "wipe `build/` and `install/`" step at the top of B0 to avoid surprise rebuild failures.

### I4 (plan documentation) — Sonar topic name mismatch

The plan's expected-topics table lists `simple_drone/sonar/out` (Sphinx Iron name). On Jazzy with `sjtu_drone_bringup` 2026-03, the gz topic `.../sensor/sonar/scan` is bridged and renamed to `simple_drone/sonar`. Update [smoke-test-b4-ch5-ch6.md](../smoke-test-b4-ch5-ch6.md) line 130 accordingly.

### I5 (env, B1-known, restated) — `ros2 topic hz` hangs

In this Jazzy + numpy-2.4 stack `ros2 topic hz` produces no output and never terminates (kill-via-timeout shows `Terminated`). Substitute `ros2 topic echo <topic> --once` to confirm liveness. Suspect interaction between `rosidl_runtime_py.topic_statistics` and the upgraded numpy; out of scope to debug here but worth a quick reproduction at the container level.

---

## Recommendations for plan updates

1. **migrate-ros2_src-to-jazzy-plan-update.md** Phase B4:
   - Add a new prerequisite line under ch5: `Ensure container ships Harmonic-compatible robotino_description in sim_ws`. Until then, mark the ch5 launch checkbox as "blocked: container/I1".
   - Add a new action item to Phase B (or a new B6 prep step) to fix `utils/utils/math.py` so it no longer depends on `transforms3d`. This is **the** lowest-hanging fruit that unblocks ch3, ch4, ch5 action server, ekf_localization, ekf_slam, bug_planning — six packages worth of smoke tests.
2. **smoke-test-b4-ch5-ch6.md**:
   - Replace `simple_drone/sonar/out` with `simple_drone/sonar` in the expected-topics table.
   - Add note: "`ros2 topic hz` is unreliable in this container; use `ros2 topic echo --once` to confirm publication."
3. **Container repo** (out of band):
   - Either rebuild `robotino_description` against Harmonic, or strip its CMakeLists.txt dependency on Classic packages (it only installs `urdf/models/worlds` — no compile units actually need `gazebo_ros`).
4. Consider documenting an **explicit colcon `--cmake-clean-cache` recipe** for the workspace after any rcl/fastcdr/typesupport upgrade.

---

## Checkboxes ticked in migrate-ros2_src-to-jazzy-plan-update.md

Of the four B4 Smoke checkboxes:

- [ ] (95) `ros2 launch ch5 simulation.launch.py` — **NOT ticked**: launch fails on missing `robotino_description`.
- [ ] (96) `ros2 run ch5 holonomic_closed_loop_square_action_server &` — **NOT ticked**: import-time crash on `transforms3d`.
- [ ] (97) `ros2 run ch5 holonomic_closed_loop_square_action_client` — **NOT ticked**: client imports/runs cleanly but cannot complete the round-trip without a healthy server.
- [x] (103) `ros2 launch ch6 simulation.launch.py` — **ticked**: all four acceptance criteria satisfied within env caveats (gz GUI / xterm rendering known-blocked per B1).
