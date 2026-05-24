# Smoke Test — Phase B6 (`ekf_localization`)

**Phase:** B6 (EKF localization with known correspondences)
**Launch under test:** `ros2 launch ekf_localization simulation.launch.py`

**Tracks:** the "Smoke" checkbox under **Phase B6** in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md).

Verification scope: **launch + topics only**. EKF estimator quality, landmark association accuracy, and matplotlib plot correctness are out of scope — covered by the chapter exercises, not by this smoke test.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ekf_localization utils
source install/setup.bash
```

External packages required:

- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`
- `turtlebot3_gazebo` — provides `robot_state_publisher.launch.py`, `spawn_turtlebot3.launch.py`, and the `worlds/empty_world.world` file
- `turtlebot3_teleop` — provides `teleop_keyboard` (run under `xterm -e`)
- `xterm`

Verify:

```bash
ros2 pkg prefix ros_gz_sim ros_gz_bridge ros_gz_interfaces turtlebot3_gazebo turtlebot3_teleop
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/robot_state_publisher.launch.py
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/empty_world.world
which xterm
export TURTLEBOT3_MODEL=burger
```

## What this launch does

Source: [ros2_src/projects/ekf_localization/launch/simulation.launch.py](../ros2_src/projects/ekf_localization/launch/simulation.launch.py).

1. Starts gz sim with `turtlebot3_gazebo/worlds/empty_world.world` (`-r` flag → start running).
2. Bridges `/clock` between gz and ROS.
3. Includes TB3 Jazzy's `robot_state_publisher.launch.py` (passes `use_sim_time=true`).
4. Includes TB3 Jazzy's `spawn_turtlebot3.launch.py` (passes `x_pose=0.0`, `y_pose=0.0`).
5. Spawns landmarks via the first-party `landmark_spawner` node — it reads `config/landmarks.yaml` and calls `/world/default/create` once per landmark (`ros_gz_interfaces/srv/SpawnEntity`).
6. Starts `turtlebot3_teleop/teleop_keyboard` in an `xterm` so the user can drive.

## Expected nodes

| Node name | Package | Executable | Source |
|---|---|---|---|
| `parameter_bridge` (clock) | `ros_gz_bridge` | `parameter_bridge` | Direct ([line 36-41](../ros2_src/projects/ekf_localization/launch/simulation.launch.py#L36-L41)) |
| `landmark_spawner` | `ekf_localization` | `landmark_spawner` | Direct ([line 58-62](../ros2_src/projects/ekf_localization/launch/simulation.launch.py#L58-L62)) — `name='landmark_spawner'` |
| `teleop` | `turtlebot3_teleop` | `teleop_keyboard` | Direct under `xterm -e` ([line 64-69](../ros2_src/projects/ekf_localization/launch/simulation.launch.py#L64-L69)) |
| `robot_state_publisher` | `robot_state_publisher` | (via include) | `turtlebot3_gazebo/launch/robot_state_publisher.launch.py` |
| `gz sim` + spawn_turtlebot3 helpers | (various) | (various) | `turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py` and `ros_gz_sim/launch/gz_sim.launch.py` |

The exact set inside the TB3 includes lives in `~/sim_ws`; capture with `ros2 node list`.

## Expected topics

| Topic | Message type | Source |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | clock_bridge |
| `/scan` | `sensor_msgs/msg/LaserScan` | TB3 lidar bridge (from `spawn_turtlebot3.launch.py`) |
| `/odom` | `nav_msgs/msg/Odometry` | TB3 diff-drive bridge |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `teleop_keyboard` publishes; TB3 diff-drive subscribes (via bridge) |
| `/joint_states` | `sensor_msgs/msg/JointState` | TB3 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher + TB3 |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher (latched) |

The first-party EKF nodes (which subscribe to `/cmd_vel`, `/scan`, `/odom`) are **NOT** in this launch — they're started separately via `known_correspondences.launch.py` or `unknown_correspondences.launch.py`. This smoke test does not run those.

## Expected services

| Service | Type | Used by |
|---|---|---|
| `/world/default/create` | `ros_gz_interfaces/srv/SpawnEntity` | `landmark_spawner` — calls it once per landmark loaded from `config/landmarks.yaml` (~80+ unique entries → many `cylinder_N` spawns) |

The service must exist (advertised by gz sim) **before** `landmark_spawner` makes its first call. There is no explicit wait/sleep in the launch order — the call may race the service advertisement on slow machines. If the landmark_spawner logs "Service /world/default/create not available", that's an environment timing issue, not a migration regression — re-run.

## TF frames

Standard TB3 Burger tree (from TB3 Jazzy's `robot_state_publisher.launch.py`):

```
odom
└── base_footprint
    └── base_link
        ├── wheel_left_link
        ├── wheel_right_link
        ├── caster_back_link
        ├── imu_link
        └── base_scan
```

(No `map → odom` — there is no localizer in this launch.)

## Verification commands

```bash
# Terminal 1: launch
ros2 launch ekf_localization simulation.launch.py
#   expect: gz sim opens; TB3 burger spawned at (0, 0);
#           ~80+ grey cylinder landmarks spawn (may take a few seconds);
#           xterm opens with teleop_keyboard prompt

# Terminal 2: graph checks
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash

ros2 node list
#   expect: /parameter_bridge, /landmark_spawner, /teleop,
#           /robot_state_publisher (+ TB3 sim_ws helpers)

ros2 topic list | sort
#   expect to contain at minimum:
#     /clock /scan /odom /cmd_vel /joint_states /tf /tf_static /robot_description

ros2 topic hz /clock                          # non-zero
ros2 topic hz /scan                           # >0 Hz (TB3 LDS-01 ~5 Hz)
ros2 topic hz /odom                           # >0 Hz

# Landmark spawn service used + present
ros2 service list | grep "/world/default/create"
#   expect: /world/default/create   ros_gz_interfaces/srv/SpawnEntity

# Confirm cylinders appeared in gz (visual check + entity list)
gz model --list                               # expect: turtlebot3_burger + many "cylinder_N"
# alternative: ros2 service call /world/default/scene_info / use the gz sim GUI scene tree

# TF tree
ros2 run tf2_tools view_frames
#   produces frames.pdf with the tree above
```

Driving sanity check (still launch-scope — confirms teleop → bridge → gz sim):

```bash
# In the xterm with teleop_keyboard: press 'w' to drive forward
# In terminal 2:
ros2 topic echo /odom --once     # twice in a row — pose.position.x should differ
```

## Acceptance criteria

- [ ] `ros2 launch ekf_localization simulation.launch.py` starts cleanly. gz sim window opens with the empty world.
- [ ] TB3 burger appears in gz at the spawn pose.
- [ ] Multiple landmark cylinders (`cylinder_0`, `cylinder_1`, …) appear in gz (`gz model --list` or the sim GUI scene tree).
- [ ] `ros2 node list` contains `/parameter_bridge`, `/landmark_spawner`, `/teleop`, `/robot_state_publisher`.
- [ ] `ros2 topic list` contains `/clock`, `/scan`, `/odom`, `/cmd_vel`, `/joint_states`, `/tf`, `/tf_static`.
- [ ] `ros2 topic hz /clock` non-zero (sim time advancing).
- [ ] `ros2 topic hz /scan` non-zero.
- [ ] `ros2 topic hz /odom` non-zero.
- [ ] `view_frames` shows the standard TB3 tree.
- [ ] `/world/default/create` service is listed.

## Known caveats

- **Race condition: landmark_spawner may start before `/world/default/create` is advertised.** The launch file adds all actions in one go; there's no `RegisterEventHandler` waiting for the spawn service. On slow machines you may see a "service not available" error. If so, re-run — and consider opening an issue to wrap `landmark_spawner` in an event handler. Do not block smoke acceptance on this if a second run succeeds.
- **`spawn_turtlebot3.launch.py` may itself use `gazebo_msgs.SpawnEntity` if the container's TB3 build pre-dates the Jazzy bridge update.** If TB3 spawn fails, that's an upstream issue in `sim_ws` — file against the container repo, not this plan.
- **Landmarks YAML is large.** `config/landmarks.yaml` has ~130 entries (many duplicates). Spawning all of them takes several seconds and the `landmark_spawner` log may flood. Expect the scene to populate gradually.
- **No EKF in this smoke test.** The `ekf_localization_with_known_correspondences` node is **not** started by `simulation.launch.py`. That's run separately via `known_correspondences.launch.py`. Behavioral verification of the EKF is out of scope.
- **`TURTLEBOT3_MODEL` env var.** TB3 Jazzy's spawn launch requires this env to be set (typically `burger`). If unset, spawn fails with "model not found".

## On pass

Tick the `- [ ] Smoke: ros2 launch ekf_localization simulation.launch.py` line under **Phase B6** in `migrate-ros2_src-to-jazzy-plan-update.md`.
