# Smoke Test Execution — Phase B5 (`project`)

**Date:** 2026-05-16
**Executor:** Claude Code (Opus 4.7, 1M context)
**Container image:** `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:iron-desktop-terminal-nvidia` (running ROS 2 Jazzy after B1 apt upgrades)
**Branch:** `jazzy-migration`
**Plan:** [../smoke-test-b5-project.md](../smoke-test-b5-project.md)

## Carry-over context from earlier phases

- **B1:** `ros-jazzy-example-interfaces` installed; `rclpy/rcl/rmw/rosidl/fastcdr/type-description-*` upgraded to April-2026 sync. ([smoke-test-b1-ch1-execution.md](smoke-test-b1-ch1-execution.md))
- **B1/MEMORY:** GUI rendering blocked by container DRI permissions; verify gz via topic graph + `/clock`, not window appearance.
- **B4:** `tf_transformations` (`transforms3d 0.4.1` + numpy 2) is a known import-time crash for every consumer including `project/spawn_robot.py`. Documented in [../errors/tf_transformations-numpy2-incompatibility.md](../errors/tf_transformations-numpy2-incompatibility.md).
- Per request: do **not** fix `tf_transformations` here; confirm and continue.

## Pre-flight

```bash
$ colcon build --packages-select project
Starting >>> project
Finished <<< project [0.95s]
Summary: 1 package finished [1.04s]

$ ros2 pkg prefix ros_gz_sim ros_gz_bridge ros_gz_interfaces nav2_bringup
ros_gz_sim          → /opt/ros/jazzy
ros_gz_bridge       → /opt/ros/jazzy
ros_gz_interfaces   → /opt/ros/jazzy
nav2_bringup        → /opt/ros/jazzy

$ ls $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/localization_launch.py
/opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py        # present

$ echo "$GZ_SIM_RESOURCE_PATH" | tr ':' '\n' | grep project
/workspaces/rclpy-from-zero-to-hero/install/project/share/project/models
/home/ros_user/ros2_ws/install/project/share/project/models
```

No stale-hook warnings observed in this build. Hook rename (`hook_gazebo.sh` → `hook_gz.sh`) is reflected correctly in the install tree.

---

## Part 1 — `ros2 launch project simulation.launch.py`

Launched in background to `/tmp/project_sim_launch.log`, 30 s soak.

### Acceptance-criteria summary (Part 1)

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | Launch starts cleanly. gz sim window opens with `project.world` loaded. | **FAIL** | `gz sim` aborts during world parse — `<ambient>255 255 255 1</ambient>` invalid (see Issue A below). Process exits cleanly before ever advertising services. |
| 2 | TurtleBot3 burger spawned via `/world/default/create`. | **FAIL** | (a) `spawn_robot` crashed at import: `tf_transformations` → numpy 2 `maximum_sctype` (Issue B). (b) Even with that fixed, `/world/default/create` was never advertised because `gz sim` died (Issue A). |
| 3 | `ros2 node list` contains all 6 first-party Nodes. | **PARTIAL** | Present: `/robot_state_publisher`, `/joint_state_publisher`, `/ros_gz_bridge` (×2), `/rviz2`. Missing: `/spawn_robot` (crashed). Gazebo internal nodes also absent because `gz sim` died. |
| 4 | `ros2 topic list` contains `/clock /imu /scan /image_raw /camera_info /cmd_vel /odom /joint_states`. | **PASS (advertised) / FAIL (data)** | All eight topics are *advertised* by the parameter bridges (see node-list output below). No data flows because the gz-side endpoint does not exist. |
| 5 | `topic hz` non-zero on `/clock`, `/scan`, `/odom`, `/image_raw`, `/imu`. | **FAIL** | All five `timeout 8 ros2 topic hz <t>` calls produced no output and were killed by the timeout — no message ever arrived. |
| 6 | `rqt_image_view` displays non-blank image on `/image_raw`. | **N/A (env)** | Without a display, Qt fails to load any platform plugin; with `QT_QPA_PLATFORM=offscreen` the binary runs without errors, but no display can be observed. `/image_raw` has no data anyway (Issue A). |
| 7 | `view_frames` produces a TF tree matching the expected hierarchy. | **PARTIAL** | TF tree produced (`/tmp/frames_2026-05-16_13.44.56.pdf` + `.gv`), but only the `robot_state_publisher` chain (`base_footprint → base_link → wheel_left_link, wheel_right_link, caster_back_link, imu_link, base_scan, camera_link → camera_rgb_frame → camera_rgb_optical_frame`). **No `odom` root** because gz-sim diff-drive system never started. |

### Evidence — node list (during sim launch)

```text
$ ros2 node list
WARNING: Be aware that there are nodes in the graph that share an exact name…
/joint_state_publisher
/robot_state_publisher
/ros_gz_bridge                                    # clock_bridge
/ros_gz_bridge                                    # topic_bridge (7 topics)
/rviz2
/transform_listener_impl_6294659d8100             # rviz2's TF buffer
```

**Missing:** `/spawn_robot` (crashed at import). No gz-sim internal nodes — Gazebo process exited.

### Evidence — topic list (during sim launch)

```text
$ ros2 topic list
/camera/camera_info        # rviz internal
/camera/image_raw          # rviz internal
/camera_info               # ← bridge advertisement
/clicked_point             # rviz
/clock                     # ← bridge advertisement (no data)
/cmd_vel                   # ← bridge subscriber side
/goal_pose                 # rviz
/image_raw                 # ← bridge advertisement
/imu                       # ← bridge advertisement
/initialpose               # rviz
/joint_states              # ← bridge advertisement
/line_detection            # carried over (test artefact, no publisher)
/map                       # carried over, no publisher
/map_updates               # carried over
/odom                      # ← bridge advertisement
/parameter_events
/robot_description
/rosout
/scan                      # ← bridge advertisement
/tf
/tf_static
```

All eight expected topics are present in the graph — they are **advertised** by the `parameter_bridge` nodes. None carry data: the gz-side endpoint that the bridge would relay is absent because `gz sim` died.

### Evidence — bridge advertisements (from `/tmp/project_sim_launch.log`)

```text
[parameter_bridge-4] Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)]
[parameter_bridge-5] Creating GZ->ROS Bridge: [/imu (gz.msgs.IMU) -> /imu (sensor_msgs/msg/Imu)]
[parameter_bridge-5] Creating GZ->ROS Bridge: [/scan (gz.msgs.LaserScan) -> /scan (sensor_msgs/msg/LaserScan)]
[parameter_bridge-5] Creating GZ->ROS Bridge: [/image_raw (gz.msgs.Image) -> /image_raw (sensor_msgs/msg/Image)]
[parameter_bridge-5] Creating GZ->ROS Bridge: [/camera_info (gz.msgs.CameraInfo) -> /camera_info (sensor_msgs/msg/CameraInfo)]
[parameter_bridge-5] Creating ROS->GZ Bridge: [/cmd_vel (geometry_msgs/msg/Twist) -> /cmd_vel (gz.msgs.Twist)]
[parameter_bridge-5] Creating GZ->ROS Bridge: [/odom (gz.msgs.Odometry) -> /odom (nav_msgs/msg/Odometry)]
[parameter_bridge-5] Creating GZ->ROS Bridge: [/joint_states (gz.msgs.Model) -> /joint_states (sensor_msgs/msg/JointState)]
```

### Evidence — `gz sim` world-load failure (Issue A)

```text
[gazebo-3] Error [Param.cc:1288] Error Code 34: Msg: The value <ambient>255 255 255 1</ambient> is invalid.
[gazebo-3] [Err] [Server.cc:86] Error Code 9:
    [/sdf/world[@name="default"]/scene/ambient:.../project.world:L25]:
    Msg: Error reading element <ambient>
[gazebo-3] [Err] [Server.cc:86] Error Code 9:
    [/sdf/world[@name="default"]/scene:.../project.world:L24]: Error reading element <scene>
[gazebo-3] [Err] [Server.cc:86] Error Code 9:
    [/sdf/world[@name="default"]:.../project.world:L2]:   Error reading element <world>
[gazebo-3] [Err] [Server.cc:86] Error Code 1:
    Unable to read file: [.../project.world]
[INFO] [gazebo-3]: process has finished cleanly [pid 9172]
```

`<scene>/<ambient>` values must be normalised `[0, 1]` in Gazebo Harmonic. Line 25 reads `255 255 255 1` (Classic 0-255 RGB convention) which fails range validation, cascading the entire world parse. Full analysis in [../errors/project-world-ambient-color-out-of-range.md](../errors/project-world-ambient-color-out-of-range.md).

There is also a non-fatal SDF warning earlier in the log:

```text
Warning [Utils.cc:132] [/sdf/world[@name="default"]/light[@name="sun"]/pose[@frame=""]:.../project.world:L5]:
  XML Attribute[frame] in element[pose] not defined in SDF.
```

That one is informational and does not abort the load.

### Evidence — `spawn_robot` import-time crash (Issue B = the known tf_transformations bug)

```text
[spawn_robot-6] File ".../project/project/spawn_robot.py", line 7, in <module>
[spawn_robot-6]   from tf_transformations import quaternion_from_euler
[spawn_robot-6] File "/opt/ros/jazzy/lib/python3.12/site-packages/tf_transformations/__init__.py", line 46, in <module>
[spawn_robot-6]   import transforms3d
[spawn_robot-6] File "/usr/lib/python3/dist-packages/transforms3d/__init__.py", line 10, in <module>
[spawn_robot-6]   from . import quaternions
[spawn_robot-6] File "/usr/lib/python3/dist-packages/transforms3d/quaternions.py", line 26, in <module>
[spawn_robot-6]   _MAX_FLOAT = np.maximum_sctype(np.float64)
[spawn_robot-6] AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release.
[ERROR] [spawn_robot-6]: process has died [pid 9176, exit code 1, ...]
```

Matches [../errors/tf_transformations-numpy2-incompatibility.md](../errors/tf_transformations-numpy2-incompatibility.md) exactly. Pre-existing issue, not a new B5 finding — recorded here for completeness.

### Evidence — services (no spawn service exists)

```text
$ ros2 service list | grep -E "create|world"
(empty)
```

Confirms that `/world/default/create` was never advertised — `gz sim` died before activating its services.

### Evidence — sanity drive impossible

```text
$ ros2 topic info /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1     # the topic_bridge subscriber

$ ros2 topic info /odom
Type: nav_msgs/msg/Odometry
Publisher count: 1        # the topic_bridge advertiser
Subscription count: 0
```

`/cmd_vel` would be relayed into gz, but nothing on the gz side; `/odom` has nothing to relay back. The plan's "publish `/cmd_vel` then read `/odom`" sanity-drive cannot be exercised.

### Overall verdict — Part 1: **FAIL (PARTIAL)**

- Launch *starts*, all expected ROS-side nodes except `spawn_robot` come up, all topics are advertised. The simulation back-end (gz sim) does not. Two independent blockers — Issue A is the dominant one (world parse), Issue B is secondary (spawn_robot crash) and only observable as a launch traceback.

---

## Part 2 — `ros2 launch project nav2.launch.py`

Launched standalone (no sim from Part 1 running). Logged to `/tmp/project_nav2_launch.log`, 20 s soak.

### Acceptance-criteria summary (Part 2)

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | `nav2.launch.py` starts without tracebacks. | **PASS** | All three nodes started, no tracebacks in log. Only one INFO "initialPoseReceived" warning later when we tested seeding (see below). |
| 2 | `ros2 node list` contains `/map_server`, `/amcl`, `/lifecycle_manager_localization`. | **PASS** | All three present. |
| 3 | `ros2 lifecycle get /amcl` returns `active`. | **PASS** | `active [3]` |
| 4 | `ros2 lifecycle get /map_server` returns `active`. | **PASS** | `active [3]` |
| 5 | `ros2 topic list` contains `/map`, `/amcl_pose`, particle-cloud topic. | **PASS** | `/map`, `/amcl_pose`, `/particle_cloud` all present. |
| 6 | `tf2_echo map odom` returns a transform (may need `/initialpose` seed). | **FAIL (expected)** | No sim running → no `odom` frame published anywhere in TF, so amcl cannot produce `map → odom`. Seeding `/initialpose` was attempted; amcl logged "initialPoseReceived" then rejected our manually-built message as malformed (zero timestamp), but the *config-file* initial pose was already accepted at activation. Independent of seeding, the missing `odom` frame is the hard blocker. Would PASS once Part 1's sim is working. |

### Evidence — nodes & lifecycle

```text
$ ros2 node list
/amcl
/lifecycle_manager_localization
/map_server
/rqt_gui_cpp_node_*       # rviz/rqt leftovers from prior session, unrelated

$ ros2 lifecycle get /amcl
active [3]

$ ros2 lifecycle get /map_server
active [3]
```

### Evidence — map_server load + activation

```text
[map_server-1] [INFO] [map_server]: Configuring
[map_server-1] [INFO] [map_io]: Loading yaml file: .../project/map/map.yaml
[map_server-1] [INFO] [map_io]: resolution: 0.078
[map_server-1] [INFO] [map_io]: Read map .../project/map/map.png: 162 X 162 map @ 0.078 m/cell
[lifecycle_manager-3] [INFO] [lifecycle_manager_localization]: Activating map_server
[map_server-1] [INFO] [map_server]: Activating
[map_server-1] [INFO] [map_server]: Creating bond (map_server) to lifecycle manager.
[lifecycle_manager-3] [INFO] [lifecycle_manager_localization]: Server map_server connected with bond.
```

### Evidence — amcl initial pose accepted from params

```text
[amcl-2] Activating
[amcl-2] initialPoseReceived
[amcl-2] Setting pose (0.000000): 5.901 12.183 -1.571        # from nav2_params.yaml
[amcl-2] Creating bond (amcl) to lifecycle manager.
[lifecycle_manager-3] Managed nodes are active
```

### Evidence — manual `/initialpose` seed rejected (timestamp issue)

```text
[amcl-2] initialPoseReceived
[amcl-2] [ERROR] [amcl]: Received initialpose message is malformed. Rejecting.
```

This is the `ros2 topic pub --once /initialpose ...` command from the plan. The pose itself is fine; AMCL's `messageFilter` rejects it because the constructed `Header.stamp` is `0`. For the smoke test this is cosmetic — the config-file pose was already accepted during activation.

### Evidence — `/map` topic

```text
$ ros2 topic info /map
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1                    # map_server
Subscription count: 1                 # amcl
```

### Evidence — `tf2_echo map odom` cannot succeed without sim

```text
$ timeout 6 ros2 run tf2_ros tf2_echo map odom
Terminated      # no output, killed by timeout — amcl has no odom frame to anchor against
```

This is the documented limitation of running Part 2 alone; would PASS once Part 1 is fixed and the gz-sim diff-drive system publishes the `odom → base_footprint` edge.

### Overall verdict — Part 2: **PASS (modulo `tf2_echo`)**

- All five "structural" criteria pass: launch clean, lifecycle nodes active, map loaded, topics advertised. The single failing criterion is one that **inherently cannot be tested without Part 1's sim running**; not a defect in `nav2.launch.py`.

---

## Issues found

### Issue A — `project.world` `<ambient>` colour values out of range (**NEW, critical**)

- `project/worlds/project/project.world:25` declares `<ambient>255 255 255 1</ambient>` (Gazebo Classic 0-255 convention). Harmonic expects normalised `[0, 1]` values; the load fails with `Error Code 34`, cascading the whole world parse, and `gz sim` exits.
- This is the dominant blocker for B5 Part 1.
- Full details + fix in [../errors/project-world-ambient-color-out-of-range.md](../errors/project-world-ambient-color-out-of-range.md).

### Issue B — `tf_transformations` numpy-2 crash in `spawn_robot` (**known, deferred**)

- Already tracked in [../errors/tf_transformations-numpy2-incompatibility.md](../errors/tf_transformations-numpy2-incompatibility.md). Re-confirmed here with a fresh traceback. Until Issue A is fixed, this is masked (the spawn service it needs to call doesn't exist anyway).

### Issue C — `rqt_image_view` headless behaviour (env, informational)

- Without an X / Wayland display, the binary fails to load any Qt platform plugin (`This application failed to start because no Qt platform plugin could be initialized.`). Setting `QT_QPA_PLATFORM=offscreen` lets the binary launch without errors. No way to *see* the image inside this container — the plan's "must show camera frames" criterion is effectively `N/A` until VNC/X is available.
- Already covered conceptually by the existing memory `jazzy_container_headless_gazebo` — this is the same root cause (DRI/display) for a different Qt-based tool.

### Issue D — `ros2 topic pub --once /initialpose` produces an AMCL malformed-message reject (informational)

- AMCL rejects the manually-constructed PoseWithCovarianceStamped because `Header.stamp` defaults to 0. The plan's snippet works structurally (the publisher reaches amcl) but the pose is dropped. Cosmetic — the config-file initial pose covers the use case. If we want the snippet to actually take effect, add `"header: {stamp: {sec: ' $(date +%s) '}}"` or use rviz's "2D Pose Estimate" tool. Not worth a `plans/errors/` file; just a note for the plan.

### Issue E — orphan stale topics in graph (informational)

- `/line_detection`, `/map`, `/map_updates`, `/camera/camera_info`, `/camera/image_raw` appear in `ros2 topic list` during Part 1 from previous-run subscribers held alive by `rviz2` and the dangling `transform_listener_impl_…`. Not a defect; just a daemon-discovery artefact in the container.

---

## Recommendations for plan adjustments

1. **Highest priority:** add a B5.3.1 sub-step "validate world colour-channel ranges" — grep for `<(ambient|background|diffuse|specular|emissive)>` values `> 1.0` across all `.world`/`.sdf` files. This caught Issue A retroactively and would have surfaced it during B5.3 TDD.
2. **Update Part 1 acceptance criterion 4** to distinguish "topic advertised in graph" from "topic carries data". Both are useful signals — separating them clarifies whether a failure is in the bridge (advertisement missing) or in gz-sim (no data). Suggested wording in the plan:
   ```
   - [ ] `ros2 topic list` lists all 8 expected topics (advertisement check).
   - [ ] `ros2 topic hz` is non-zero on /clock /scan /odom /image_raw /imu (data check).
   ```
3. **Update Part 2 `tf2_echo map odom` line:** mark it as conditional on Part 1 sim running. When Part 2 is exercised alone, this check must be expected-FAIL.
4. **Headless rqt_image_view note:** add a one-liner to "Known caveats" — "in the devcontainer without X11, `rqt_image_view` cannot render; set `QT_QPA_PLATFORM=offscreen` to at least confirm it can launch."
5. **Manual `/initialpose` seed:** the plan's example payload uses an implicit zero stamp, which AMCL rejects. Either drop the example (config-file initial pose already works) or document the timestamp requirement.

---

## Checkbox dispositions in `migrate-ros2_src-to-jazzy-plan-update.md`

Phase B5 "Smoke" lines (currently lines 148-152) — none of the five can be ticked yet:

| Line | Item | Disposition |
|---|---|---|
| 148 | `ros2 launch project simulation.launch.py` | **NOT TICKED** — Issue A blocks; launch starts but gz dies. |
| 149 | `ros2 topic list` — 8 expected | Arguably PASS (topics are advertised) — but spirit of the check is "are sensors flowing"; leave unchecked until Issue A fixed. |
| 150 | `rqt_image_view` | **NOT TICKED** — no display + no image data. |
| 151 | `view_frames` | **NOT TICKED** — TF tree partial (no `odom` root). |
| 152 | `ros2 launch project nav2.launch.py` | **TICKED** — five of six criteria green; the only failing criterion (`tf2_echo map odom`) is structurally dependent on Part 1, which is a separate checkbox. |

Only line 152 has been ticked. The remaining four stay unchecked until Issue A is fixed and Part 1 re-runs cleanly.

---

## Artefacts

- Sim launch log: `/tmp/project_sim_launch.log` (kept for this run)
- Nav2 launch log: `/tmp/project_nav2_launch.log` (kept for this run)
- TF tree PDF: `/tmp/frames_2026-05-16_13.44.56.pdf` + `.gv` (partial tree, no `odom`)
- New error file: [../errors/project-world-ambient-color-out-of-range.md](../errors/project-world-ambient-color-out-of-range.md)
