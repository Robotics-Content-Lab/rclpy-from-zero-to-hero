# Smoke Test — Phase B3 (`ch2`, `ch3`)

**Phase:** B3 (TurtleBot3-based chapters)
**Launches under test:**
- `ros2 launch ch2 simulation.launch.py`
- `ros2 launch ch3 simulation.launch.py`

**Tracks:** the two "Smoke" checkboxes under **Phase B3** in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md).

Verification scope: **launch starts cleanly and the expected nodes/topics appear**. No verification of TB3 motion or sensor content.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch2 ch3
source install/setup.bash

# Required external packages (ship in the container's sim_ws):
ros2 pkg prefix turtlebot3_gazebo                       # must succeed
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/launch/
#   must contain empty_world.launch.py AND turtlebot3_world.launch.py

# TB3 model selection (Burger is the book default)
export TURTLEBOT3_MODEL=burger
```

If `turtlebot3_gazebo` is missing, install via `sudo apt install ros-jazzy-turtlebot3-gazebo` (or rebuild `~/sim_ws` per the container's instructions).

## What each launch does

Both launches are **thin wrappers** — they include `turtlebot3_gazebo` launches and add no first-party nodes:

- [ros2_src/ch2/launch/simulation.launch.py](../ros2_src/ch2/launch/simulation.launch.py) — depending on the `world_file:=` argument (default `empty_world`), includes either `turtlebot3_gazebo/launch/empty_world.launch.py` or `turtlebot3_world.launch.py`.
- [ros2_src/ch3/launch/simulation.launch.py](../ros2_src/ch3/launch/simulation.launch.py) — includes `ch2/launch/simulation.launch.py` and passes `world_file` through.

Net effect: ch2 and ch3 produce **the same node/topic graph**. The exact set of nodes depends on `turtlebot3_gazebo`'s Jazzy launch — do not enumerate them rigidly; capture them with `ros2 node list` and verify the expected topics + TF tree below.

## Expected topics (both launches)

| Topic | Message type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | TB3 Jazzy ships the clock bridge inside its launch |
| `/scan` | `sensor_msgs/msg/LaserScan` | LDS-01 lidar bridge, ~5 Hz |
| `/imu` | `sensor_msgs/msg/Imu` | ~200 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | from gz-sim diff-drive, ~30 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | wheel joints |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | actuator input |
| `/tf` | `tf2_msgs/msg/TFMessage` | dynamic TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | URDF static frames |
| `/robot_description` | `std_msgs/msg/String` | latched, from `robot_state_publisher` |

If `world_file:=turtlebot3_world`, expect the same set — only the Gazebo world geometry changes.

## Expected nodes (skeleton — confirm by capture)

At minimum these must appear (names may have namespaces depending on TB3 Jazzy):

- `/robot_state_publisher`
- `/joint_state_publisher` (if TB3 launch includes it; Jazzy version typically delegates to gz-sim's joint state)
- `/ros_gz_bridge` (or `/parameter_bridge`) — at least one for `/clock` plus the sensor bridges
- `/gz_*` (gz-sim helper nodes are not ROS nodes, but the `gzserver`/`gz sim` process must be alive)

To confirm: `ros2 node list` and verify nodes related to TB3 / bridges / state publishers all appear with no Python tracebacks.

## TF tree (expected)

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

## Verification commands

For **each** of `ros2 launch ch2 simulation.launch.py` and `ros2 launch ch3 simulation.launch.py`:

```bash
# Terminal 1: run the launch (Gazebo Harmonic window must open)
ros2 launch ch2 simulation.launch.py
# or:  ros2 launch ch3 simulation.launch.py
# Optional world swap:  world_file:=turtlebot3_world

# Terminal 2: source overlay, then check graph
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash

ros2 node list                                 # capture for the report
ros2 topic list                                # must contain all 9 topics above
ros2 topic hz /clock                           # non-zero (sim time running)
ros2 topic hz /scan                            # ~5 Hz (LDS-01)
ros2 topic hz /odom                            # >0 Hz
ros2 topic info /cmd_vel                       # ≥1 subscriber (diff-drive bridge)
ros2 run tf2_tools view_frames                 # produces frames.pdf with the tree above
```

Sanity drive check (still launch-scope — confirms `/cmd_vel` is wired to the drive bridge, not algorithm validation):

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
# Observe TB3 inch forward in Gazebo, then send a zero-twist to stop.
```

## Acceptance criteria

For **each** launch (ch2 and ch3), smoke test is green when ALL of:

- [ ] Launch starts with no Python tracebacks; Gazebo Harmonic window opens with TB3 visible.
- [ ] `ros2 topic list` contains every topic in the table above.
- [ ] `ros2 topic hz /clock` reports a non-zero rate (simulated time advancing).
- [ ] `ros2 topic hz /scan` reports a non-zero rate.
- [ ] `ros2 topic hz /odom` reports a non-zero rate.
- [ ] `view_frames` PDF shows the TF tree above with no orphaned frames.

Both ch2 and ch3 must pass for **Phase B3** to be green.

## Known caveats

- **Names may shift between TB3 Jazzy releases.** If `empty_world.launch.py` or `turtlebot3_world.launch.py` has been renamed upstream, the wrapper launches will fail with a file-not-found error. Per the migration plan, update the `PathJoinSubstitution` strings; do not vendor TB3 code.
- **Namespacing.** Backend decision was "namespaces without leading slash." TB3 Jazzy's default is no namespace at all — topics are unprefixed (`/scan`, `/odom`). If a namespace appears (e.g. `/burger/scan`), that's likely a `TURTLEBOT3_MODEL` env quirk; record it but don't block.
- **No first-party nodes in either launch.** ch2 and ch3's chapter exercises run additional nodes (`closed_loop_p_square`, etc.) manually via `ros2 run`. Those are out of scope for this smoke test — they're covered by `ros2 launch ch3 square.launch.py` if you want to verify them, which is bonus.

## On pass

Tick both `- [ ] Smoke: ros2 launch ch2 simulation.launch.py` and `- [ ] Smoke: ros2 launch ch3 simulation.launch.py` lines under **Phase B3** in `migrate-ros2_src-to-jazzy-plan-update.md`.
