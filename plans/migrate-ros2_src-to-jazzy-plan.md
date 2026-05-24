# Plan: Migrate `ros2_src/` to ROS Jazzy + Gazebo Harmonic

## Context

The two container repos (`rclpy-from-zero-to-hero-container/`, `rclpy_from_zero_to_hero_container/`) have already been migrated to Ubuntu 24.04 + ROS Jazzy + Gazebo Harmonic. The published image now ships `robotino` and `sjtu_drone` pre-built into `/home/ros_user/sim_ws` (the latter on the upstream's `--branch jazzy`). Today, this repo's `ros2_src/` still targets Iron + Gazebo Classic, so a student who pulls the new container can no longer build the book's examples.

This plan ports `ros2_src/` to the target stack. It is one of three coordinated plans:
- `../../rclpy_from_zero_to_hero_container/plans/enable-jazzy-in-templates-plan.md` — publishes `:jazzy-*` image tags.
- **This file** — ports the source tree.
- `../../book/plans/migrate-book-to-jazzy-plan.md` — updates the textbook content.

**Strategy** (user-confirmed): clean cut to Jazzy on `main`. Tag the current state as `iron-final` for archival; future work targets Jazzy only.

## Decisions inherited from the backend migration

These are baked into the published images and must be followed for consistency (sources: `rclpy-from-zero-to-hero-container/plans/migrate-sjtu-drone-to-gz-phase-{3,4}-complete.md` and `fix-sjtu-drone-tf-bridge-plan.md`):

- Gazebo Harmonic (gz-sim 8); SDF format **1.9**.
- All sensors bridged via `ros_gz_bridge` with explicit `<gz_frame_id>` overrides on every `<sensor>`.
- Clock bridge required for `use_sim_time`: `parameter_bridge ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']`.
- `joint_state_publisher` must receive `parameters=[{'use_sim_time': use_sim_time}]`.
- Namespaces written **without leading slash** (`simple_drone`, not `/simple_drone`).
- Launch files use `ros_gz_sim`'s `gz_sim.launch.py` — not `gazebo_ros` `gzserver.launch.py`.
- `GZ_SIM_RESOURCE_PATH` replaces `GAZEBO_MODEL_PATH`; the container's bashrc already prepends `/home/ros_user/.gazebo/models`.
- `sjtu_drone` and `robotino` are consumed from the container's `sim_ws` — do **not** vendor drone/robot code here.

## Pre-flight

```bash
git tag iron-final
git push origin iron-final
git checkout -b jazzy-migration
```

All subsequent work happens inside the Jazzy container (default `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia`).

## Phase B1 — Trivial packages (build & test only)

These have no Gazebo coupling and use API surface that is stable Iron → Jazzy. Inside the container:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ch1 ch4 hw1_pkg hw3_pkg hw4_pkg preface ch5_interfaces utils
colcon build --packages-select bug_planning ekf_slam   # under projects/
source install/setup.bash
```

For each, run its launch file (where present) and verify nodes start cleanly. If setuptools 70+ emits warnings about `data_files`, ignore — warnings are non-fatal.

Expected result: all 10 packages build and load. No code edits.

## Phase B2 — `hw2_pkg` QoS rename (optional)

[hw2_pkg/hw2_pkg/sensor_proc.py:8](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L8) imports `QoSReliabilityPolicy`. Both `QoSReliabilityPolicy` and `ReliabilityPolicy` still resolve in Jazzy's `rclpy.qos`, so this is cosmetic. Skip unless Part C's chapter text gets rewritten to match the newer naming.

## Phase B3 — TurtleBot3-based chapters (`ch2`, `ch3`)

The Jazzy build of `turtlebot3_gazebo` internally launches Gazebo Harmonic. Launch-file argument names changed upstream. For each of:

- [ros2_src/ch2/launch/simulation.launch.py](../ros2_src/ch2/launch/simulation.launch.py)
- [ros2_src/ch3/launch/simulation.launch.py](../ros2_src/ch3/launch/simulation.launch.py)

Verify the included `turtlebot3_gazebo` launch paths still exist on Jazzy:

```bash
ros2 pkg prefix turtlebot3_gazebo
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/
```

If `empty_world.launch.py` / `turtlebot3_world.launch.py` were renamed, update the `os.path.join(...)` strings. No deeper rewrite — TB3 hides the Harmonic plumbing from us.

Run to verify:
```bash
ros2 launch ch2 simulation.launch.py
ros2 launch ch3 simulation.launch.py
```

## Phase B4 — `ch5`, `ch6` → `ros_gz_sim` + `sim_ws`

Both currently call `gazebo_ros` `gzserver.launch.py`:
- [ros2_src/ch5/launch/simulation.launch.py:10,32,41](../ros2_src/ch5/launch/simulation.launch.py#L10) (also references `robotino_description` / `robotino_bringup` at lines 14, 22-23)
- [ros2_src/ch6/launch/simulation.launch.py](../ros2_src/ch6/launch/simulation.launch.py)

**Replacement pattern** (lifted from `migrate-sjtu-drone-to-gz-phase-4-complete.md`):

```python
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
world_path = os.path.join(pkg_share, 'worlds', 'something.world')

gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': f'-r {world_path}'}.items(),
)

clock_bridge = Node(
    package='ros_gz_bridge', executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
)
```

Drop the `gzclient.launch.py` include — Harmonic combines server+GUI; omit `-r` for GUI mode, include `-r` for headless.

`robotino_description` and `robotino_bringup` are available in the container from `sim_ws`. Add them as `<depend>` in `package.xml`; no vendoring.

Verify with the chapter's action client/server roundtrip:
```bash
ros2 launch ch5 simulation.launch.py
ros2 run ch5 holonomic_closed_loop_square_action_server &
ros2 run ch5 holonomic_closed_loop_square_action_client
```

## Phase B5 — `project/` (the heavy lift)

Four sub-steps; each must build/run before the next.

### B5.1 — `gazebo_msgs.srv.SpawnEntity` → `ros_gz_interfaces.srv.SpawnEntity`

[project/project/spawn_robot.py:5](../ros2_src/project/project/spawn_robot.py#L5):

- Import: `from ros_gz_interfaces.srv import SpawnEntity` (different message: top-level `entity_factory` field of type `EntityFactory`, containing `name`/`sdf`/`pose`/`relative_to`).
- Service endpoint moves from `/spawn_entity` to `/world/<world_name>/create`.
- Update the client request construction. Wait-for-service timeout stays the same.

### B5.2 — xacro plugin rewrite

[project/urdf/turtlebot3_burger_camera.xacro:229-337](../ros2_src/project/urdf/turtlebot3_burger_camera.xacro#L229) defines five Classic plugins. Replace each:

| Classic plugin | Harmonic equivalent | Bridge entries to add |
|---|---|---|
| `libgazebo_ros_imu_sensor.so` | `<sensor type="imu">` + world-level `gz-sim-imu-system` | `/imu@sensor_msgs/msg/Imu[gz.msgs.IMU` |
| `libgazebo_ros_ray_sensor.so` | `<sensor type="gpu_lidar">` + `gz-sim-sensors-system` | `/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan` |
| `libgazebo_ros_camera.so` | `<sensor type="camera">` + `gz-sim-sensors-system` | `/image_raw@sensor_msgs/msg/Image[gz.msgs.Image`, `/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo` |
| `libgazebo_ros_diff_drive.so` | `<plugin filename="gz-sim-diff-drive-system">` | `/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist`, `/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry` |
| `libgazebo_ros_joint_state_publisher.so` | `<plugin filename="gz-sim-joint-state-publisher-system">` | `/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model` |

Add `<gz_frame_id>` overrides on every `<sensor>` (this is the bug-fix pattern from `fix-sjtu-drone-tf-bridge-plan.md` — without it, sensor frames are missing namespace prefixes).

### B5.3 — World + model SDF format

- [project/worlds/project/project.world](../ros2_src/project/worlds/project/project.world) (2664 lines): bump `<sdf version='...'>` to `1.9`. Strip Classic-specific top-level `<plugin>` elements (physics, world stats, factory) — these aren't needed in Harmonic. Run `gz sim --headless-rendering -r -s project.world` inside the container and iterate on validation errors.
- [project/models/project/model.sdf](../ros2_src/project/models/project/model.sdf) and [project/models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf](../ros2_src/project/models/turltebot3_burger_camera/turtlebot3_burger_camera.sdf): bump to SDF 1.9.

### B5.4 — Launch file + hook

- [project/launch/simulation.launch.py](../ros2_src/project/launch/simulation.launch.py): apply the B4 swap pattern; add the `parameter_bridge` Node listing all topics from B5.2; add the clock bridge; pass `use_sim_time=True` to `joint_state_publisher` and `robot_state_publisher`.
- [project/hook/hook_gazebo.sh](../ros2_src/project/hook/hook_gazebo.sh): change `GAZEBO_MODEL_PATH` → `GZ_SIM_RESOURCE_PATH`. Rename file to `hook_gz.sh` and update [project/setup.py:18](../ros2_src/project/setup.py#L18) to match.

Verify with the project's standard launch:
```bash
ros2 launch project project.launch.py
# Then in another terminal:
ros2 topic list   # /scan, /imu, /image_raw, /camera_info, /cmd_vel, /odom, /joint_states, /clock
ros2 run rqt_image_view rqt_image_view
ros2 run tf2_tools view_frames
```

## Phase B6 — `projects/ekf_localization/`

Same migration pattern as B5, smaller surface:
- [projects/ekf_localization/ekf_localization/utils/gazebo.py:6](../ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py#L6): mirror B5.1's service swap.
- [projects/ekf_localization/launch/simulation.launch.py:27,34](../ros2_src/projects/ekf_localization/launch/simulation.launch.py#L27): mirror B5.4's launch swap (also includes `turtlebot3_gazebo` — verify per B3).
- `projects/ekf_localization/models/Landmark/model.sdf` and `utils/models/Landmark/model.sdf`: SDF 1.7 → 1.9.

## Phase B7 — `package.xml` sweep

After all code edits, sweep every touched package's manifest. Replace:

- `<depend>gazebo_ros</depend>` → `<depend>ros_gz_sim</depend>` + `<depend>ros_gz_bridge</depend>`
- `<depend>gazebo_msgs</depend>` → `<depend>ros_gz_interfaces</depend>`
- `<depend>gazebo_plugins</depend>` → drop (Harmonic doesn't need it)

Verify nothing references Gazebo Classic:
```bash
rg -l 'gazebo_ros|gazebo_msgs|gazebo_plugins|libgazebo_ros_' ros2_src/
# expected: empty
```

## Verification (end-to-end)

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Phase B1
ros2 launch ch1 first_pub_sub.launch.py

# Phase B3
ros2 launch ch2 simulation.launch.py
ros2 launch ch3 simulation.launch.py

# Phase B4
ros2 launch ch5 simulation.launch.py    # robotino in Harmonic
ros2 launch ch6 simulation.launch.py

# Phase B5
ros2 launch project project.launch.py
ros2 launch project nav2.launch.py

# Phase B6
ros2 launch ekf_localization simulation.launch.py
```

In each: confirm `/clock` is publishing, TF tree is complete (`ros2 run tf2_tools view_frames`), sensor topics carry data, and the demonstrated motion behaviour matches what the book describes.

## Critical files

| Phase | Path | Action |
|---|---|---|
| B2 | [ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py) | Optional QoS enum rename |
| B3 | [ros2_src/ch2/launch/simulation.launch.py](../ros2_src/ch2/launch/simulation.launch.py), [ros2_src/ch3/launch/simulation.launch.py](../ros2_src/ch3/launch/simulation.launch.py) | TB3 Jazzy launch-path verify |
| B4 | [ros2_src/ch5/launch/simulation.launch.py](../ros2_src/ch5/launch/simulation.launch.py), [ros2_src/ch6/launch/simulation.launch.py](../ros2_src/ch6/launch/simulation.launch.py) | `gazebo_ros` → `ros_gz_sim` |
| B5.1 | [ros2_src/project/project/spawn_robot.py](../ros2_src/project/project/spawn_robot.py) | `ros_gz_interfaces.SpawnEntity` |
| B5.2 | [ros2_src/project/urdf/turtlebot3_burger_camera.xacro](../ros2_src/project/urdf/turtlebot3_burger_camera.xacro) | 5 plugin rewrites + `<gz_frame_id>` |
| B5.3 | [ros2_src/project/worlds/project/project.world](../ros2_src/project/worlds/project/project.world), [ros2_src/project/models/](../ros2_src/project/models/) | SDF 1.4 → 1.9 |
| B5.4 | [ros2_src/project/launch/simulation.launch.py](../ros2_src/project/launch/simulation.launch.py), [ros2_src/project/hook/hook_gazebo.sh](../ros2_src/project/hook/hook_gazebo.sh) | `ros_gz_sim` launch + `GZ_SIM_RESOURCE_PATH` hook |
| B6 | [ros2_src/projects/ekf_localization/](../ros2_src/projects/ekf_localization/) | Mirror of B5 (smaller) |
| B7 | `ros2_src/*/package.xml` (all touched) | gazebo_* deps → ros_gz_* |

## Out of scope

- Iron support — explicitly retired; users wanting Iron pull the `iron-final` tag.
- Robotino / sjtu_drone source migration — already done in the container repo's `sim_ws`.
- Re-capture of book screenshots — tracked in `../../book/plans/migrate-book-to-jazzy-plan.md`.
