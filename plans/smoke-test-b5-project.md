# Smoke Test — Phase B5 (`project`)

**Phase:** B5 (project capstone)
**Launches under test:**
- `ros2 launch project simulation.launch.py`
- `ros2 launch project nav2.launch.py`

**Tracks:** the five "Smoke" checkboxes under **Phase B5** in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md):
- `ros2 launch project simulation.launch.py`
- `ros2 topic list` — expect `/scan /imu /image_raw /camera_info /cmd_vel /odom /joint_states /clock`
- `ros2 run rqt_image_view rqt_image_view`
- `ros2 run tf2_tools view_frames`
- `ros2 launch project nav2.launch.py`

Verification scope: **launch + topics only**. We do not verify that Nav2 produces a plan or that the camera detects the yellow line — those are algorithmic behaviors, out of scope.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select project
source install/setup.bash
```

External packages required:

- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces` — for Gazebo Harmonic.
- `robot_state_publisher`, `joint_state_publisher`, `rviz2`, `xacro`.
- `nav2_bringup` — for the nav2 launch (Phase B5 second launch).

Verify:

```bash
ros2 pkg prefix ros_gz_sim ros_gz_bridge ros_gz_interfaces nav2_bringup
ls $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/localization_launch.py
#   must exist (nav2.launch.py includes it)
```

The `project` hook should have prepended `GZ_SIM_RESOURCE_PATH` to point at the package's `models/` dir:

```bash
echo "$GZ_SIM_RESOURCE_PATH" | tr ':' '\n' | grep project
#   expect a line ending in /share/project/models
```

---

## Part 1 — `ros2 launch project simulation.launch.py`

Source: [ros2_src/project/launch/simulation.launch.py](../ros2_src/project/launch/simulation.launch.py).

### Expected nodes

| Node name | Package | Executable | Source |
|---|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | `robot_state_publisher` | Direct ([line 78-88](../ros2_src/project/launch/simulation.launch.py#L78-L88)) |
| `joint_state_publisher` | `joint_state_publisher` | `joint_state_publisher` | Direct ([line 90-99](../ros2_src/project/launch/simulation.launch.py#L90-L99)) |
| `parameter_bridge` (clock) | `ros_gz_bridge` | `parameter_bridge` | Direct ([line 50-55](../ros2_src/project/launch/simulation.launch.py#L50-L55)) — arguments: `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` |
| `parameter_bridge` (topics) | `ros_gz_bridge` | `parameter_bridge` | Direct ([line 57-70](../ros2_src/project/launch/simulation.launch.py#L57-L70)) — 7 sensor/actuator topics (see table below) |
| `spawn_robot` | `project` | `spawn_robot` | Direct ([line 105-111](../ros2_src/project/launch/simulation.launch.py#L105-L111)) — calls `/world/default/create` (`ros_gz_interfaces/srv/SpawnEntity`) |
| `rviz2` | `rviz2` | `rviz2` | Direct ([line 113-120](../ros2_src/project/launch/simulation.launch.py#L113-L120)) — loads `config/rviz.rviz`; on-exit shutdown |
| `gz sim` process | `ros_gz_sim` | `gz sim` | Include `gz_sim.launch.py` with `gz_args='-r {project.world}'` ([line 44-48](../ros2_src/project/launch/simulation.launch.py#L44-L48)) |

### Expected topics

All eight must appear (the update plan calls out exactly this set):

| Topic | Message type | Bridge direction | Bridge line |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | gz → ROS | clock_bridge |
| `/imu` | `sensor_msgs/msg/Imu` | gz → ROS | `[` ([L61](../ros2_src/project/launch/simulation.launch.py#L61)) |
| `/scan` | `sensor_msgs/msg/LaserScan` | gz → ROS | `[` ([L62](../ros2_src/project/launch/simulation.launch.py#L62)) |
| `/image_raw` | `sensor_msgs/msg/Image` | gz → ROS | `[` ([L63](../ros2_src/project/launch/simulation.launch.py#L63)) |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | gz → ROS | `[` ([L64](../ros2_src/project/launch/simulation.launch.py#L64)) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → gz | `]` ([L65](../ros2_src/project/launch/simulation.launch.py#L65)) |
| `/odom` | `nav_msgs/msg/Odometry` | gz → ROS | `[` ([L66](../ros2_src/project/launch/simulation.launch.py#L66)) |
| `/joint_states` | `sensor_msgs/msg/JointState` | gz → ROS | `[` ([L67](../ros2_src/project/launch/simulation.launch.py#L67)) — note: bridges from `gz.msgs.Model`, joints filtered by the gz-sim joint-state-publisher-system |

Plus `/tf` / `/tf_static` / `/robot_description` from `robot_state_publisher`.

### Expected services

| Service | Type | Used by |
|---|---|---|
| `/world/default/create` | `ros_gz_interfaces/srv/SpawnEntity` | `project.spawn_robot` (called once at startup, spawns `turtlebot3_burger` at x=-0.55 y=6.2 yaw=-90°) |

Spawn server is owned by `gz sim` — verify it appears in `ros2 service list` before `spawn_robot` makes its call.

### TF tree

```
world / odom
└── base_footprint
    └── base_link
        ├── wheel_left_link
        ├── wheel_right_link
        ├── caster_back_link
        ├── imu_link
        ├── base_scan
        └── camera_link
            └── camera_rgb_frame
                └── camera_rgb_optical_frame
```

`world → odom` is published by gz-sim's diff-drive system. The rest comes from `robot_state_publisher`.

### Verification commands

```bash
# Terminal 1: launch
ros2 launch project simulation.launch.py
#   expect: gz sim opens with the maze world; TurtleBot3 burger spawns at (-0.55, 6.2, yaw=-90°)
#   expect: rviz2 opens with the saved config

# Terminal 2: graph checks
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash

ros2 node list
#   expect: /robot_state_publisher, /joint_state_publisher, /parameter_bridge (×2),
#           /spawn_robot, /rviz2  (plus gz-sim internal nodes)

# THE explicit checklist topic-list from the update plan:
ros2 topic list | sort
#   expect to contain at minimum:
#     /clock /imu /scan /image_raw /camera_info /cmd_vel /odom /joint_states

# Confirm sim time and sensor rates
ros2 topic hz /clock                       # non-zero
ros2 topic hz /scan                        # ~30 Hz (xacro: <update_rate>30)
ros2 topic hz /image_raw                   # ~30 Hz
ros2 topic hz /imu                         # ~30 Hz
ros2 topic hz /odom                        # >0 Hz
ros2 topic info /cmd_vel                   # subscriber: gz parameter_bridge

# Spawn service was used
ros2 service list | grep create
#   expect: /world/default/create   (ros_gz_interfaces/srv/SpawnEntity)

# rqt camera view
ros2 run rqt_image_view rqt_image_view
#   pick /image_raw — must show the maze camera frames

# TF tree dump
ros2 run tf2_tools view_frames
#   produces frames.pdf — must include camera_rgb_optical_frame leaf and odom root

# Sanity drive (still launch-scope — verifies /cmd_vel ↔ /odom plumbing)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic echo /odom --once
#   expect: pose.position.x changing slightly between successive echoes
```

### Acceptance criteria (Part 1)

- [ ] Launch starts cleanly. gz sim window opens with `project.world` loaded.
- [ ] TurtleBot3 burger spawned via `/world/default/create` at the configured pose.
- [ ] `ros2 node list` contains all 6 first-party Nodes from the table above.
- [ ] `ros2 topic list` contains every one of `/clock /imu /scan /image_raw /camera_info /cmd_vel /odom /joint_states`.
- [ ] `ros2 topic hz` non-zero on `/clock`, `/scan`, `/odom`, `/image_raw`, `/imu`.
- [ ] `rqt_image_view` displays a non-blank image on `/image_raw`.
- [ ] `view_frames` produces a TF tree matching the expected hierarchy with no missing frames.

---

## Part 2 — `ros2 launch project nav2.launch.py`

Source: [ros2_src/project/launch/nav2.launch.py](../ros2_src/project/launch/nav2.launch.py).

This launch **only includes `nav2_bringup/launch/localization_launch.py`** (the `navigation_launch.py` include is commented out, [line 40-46](../ros2_src/project/launch/nav2.launch.py#L40-L46)). It must be started **after** `simulation.launch.py` is already running, so that `/scan`, `/odom`, `/tf` exist.

### Expected nodes

Owned by `nav2_bringup/launch/localization_launch.py` — the exact set lives in the nav2 share dir. Expect at least:

| Node name | Package | Executable | Notes |
|---|---|---|---|
| `/map_server` | `nav2_map_server` | `map_server` | Loads `project/map/map.yaml` |
| `/amcl` | `nav2_amcl` | `amcl` | Particle filter, scan_topic=`scan` |
| `/lifecycle_manager_localization` | `nav2_lifecycle_manager` | `lifecycle_manager` | Activates the two above |

Confirm with `ros2 node list` post-launch.

### Expected topics

| Topic | Message type | Owner |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | map_server (latched) |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | amcl (published after first update) |
| `/particle_cloud` (or `/particlecloud` in older nav2) | `nav2_msgs/msg/ParticleCloud` | amcl |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | amcl subscriber (you publish here to seed pose) |
| `/scan` | (already from sim) | amcl subscriber |
| `/odom` | (already from sim) | amcl subscriber |

`amcl` adds `map → odom` to the TF tree.

### Verification commands

```bash
# Pre-req: simulation.launch.py is still running from Part 1.

# Terminal 3: nav2 launch
ros2 launch project nav2.launch.py
#   expect: map_server loads map.yaml; amcl initializes; lifecycle manager activates both

# Terminal 4: graph checks
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash

ros2 node list | grep -E "amcl|map_server|lifecycle_manager_localization"
ros2 topic list | grep -E "map|amcl_pose|particle"
ros2 topic info /map
ros2 topic echo /map_metadata --once          # confirms map was loaded
ros2 lifecycle get /amcl                       # should be 'active' after lifecycle manager finishes
ros2 lifecycle get /map_server                 # should be 'active'

# TF chain extension
ros2 run tf2_ros tf2_echo map odom             # must produce a transform once amcl is initialized
```

To seed AMCL's particle cloud (optional, helps the transform show up faster — still launch-scope):

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 5.901, y: 12.182}, orientation: {z: -0.707, w: 0.707}}}}"
```

(These coordinates match the AMCL initial pose configured in `project/config/nav2_params.yaml`.)

### Acceptance criteria (Part 2)

- [ ] `ros2 launch project nav2.launch.py` starts without tracebacks.
- [ ] `ros2 node list` contains `/map_server`, `/amcl`, and `/lifecycle_manager_localization`.
- [ ] `ros2 lifecycle get /amcl` returns `active`.
- [ ] `ros2 lifecycle get /map_server` returns `active`.
- [ ] `ros2 topic list` contains `/map`, `/amcl_pose`, and the particle-cloud topic.
- [ ] `tf2_echo map odom` returns a transform (with `--timeout 5` if needed; may require seeding `/initialpose`).

---

## Known caveats

- **`project.launch.py` exists** and starts both `simulation.launch.py` and `nav2.launch.py` (with a delay event handler so nav2 starts only after sim is up). The B5 smoke list only tests the two leaf launches individually. If you want a single-command sanity run, use `ros2 launch project project.launch.py` — but tick the leaf-launch boxes only after each was verified standalone.
- **`detection.py` topic mismatch.** `project/detection.py` subscribes to `/camera/image_raw`, but the bridge publishes `/image_raw`. This affects the line-following demo, **not** the smoke test (detection is not in `simulation.launch.py`). Note in your report; do not fix as part of smoke testing.
- **`nav2_navigation` is commented out** in `nav2.launch.py`. Full path planning won't work; only localization. If `Nav2` planning is desired later, uncomment lines 40-46 of `project/launch/nav2.launch.py`.
- **Initial pose required for AMCL transform.** Until `/initialpose` is published, `map → odom` may not appear. The `nav2_params.yaml` file *declares* an initial pose, but some Nav2 builds still require an `/initialpose` publish before the transform shows up. If `tf2_echo map odom` times out, publish the initial pose (see Part 2 verification commands).

## On pass

Tick all five `- [ ]` "Smoke" lines under **Phase B5** in `migrate-ros2_src-to-jazzy-plan-update.md`.
