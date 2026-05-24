# Smoke Test — Phase B4 (`ch5`, `ch6`)

**Phase:** B4 (`ch5` Robotino, `ch6` SJTU drone)
**Launches under test:**
- `ros2 launch ch5 simulation.launch.py` (+ `ros2 run ch5 square_action_server` and `square_action_client`)
- `ros2 launch ch6 simulation.launch.py`

**Tracks:** the four "Smoke" checkboxes under **Phase B4** in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md).

Verification scope: **launch + topics only**. The action server/client roundtrip is verified at the *node + action exists* level — we do not require the robot to physically complete a square.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch5_interfaces ch5 ch6
source install/setup.bash
```

External packages (preinstalled in the container's `~/sim_ws`):

- For ch5: `ros_gz_sim`, `ros_gz_bridge`, `robotino_description`, `robotino_bringup`.
- For ch6: `sjtu_drone_bringup`, `sjtu_drone_control`, `xterm`.

Verify they resolve:

```bash
ros2 pkg prefix ros_gz_sim ros_gz_bridge robotino_description robotino_bringup    # ch5
ros2 pkg prefix sjtu_drone_bringup sjtu_drone_control                              # ch6
which xterm                                                                         # ch6 teleop runs under xterm -e
```

---

## Part 1 — `ros2 launch ch5 simulation.launch.py`

Source: [ros2_src/ch5/launch/simulation.launch.py](../ros2_src/ch5/launch/simulation.launch.py).

### Expected nodes

| Node name | Package | Executable | Source |
|---|---|---|---|
| `parameter_bridge` (clock) | `ros_gz_bridge` | `parameter_bridge` | Direct ([ch5/launch/simulation.launch.py:29-34](../ros2_src/ch5/launch/simulation.launch.py#L29-L34)) |
| `spawn_robotino` | `robotino_bringup` | `spawn_robotino` | Direct ([ch5/launch/simulation.launch.py:36-41](../ros2_src/ch5/launch/simulation.launch.py#L36-L41)) — passes `robot_desc` (xacro-processed URDF) + `model_ns='robotino'` |
| `gz_sim` process | `ros_gz_sim` | `gz sim` | Via include `gz_sim.launch.py` with `gz_args='-r empty.sdf'` ([line 23-27](../ros2_src/ch5/launch/simulation.launch.py#L23-L27)) |

### Expected topics

| Topic | Message type | Source |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | clock bridge (Node arguments: `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`) |
| `robotino/cmd_vel` | `geometry_msgs/msg/Twist` | bridged by `spawn_robotino` (see caveats) |
| `robotino/odom` | `nav_msgs/msg/Odometry` | bridged by `spawn_robotino` |
| `robotino/joint_states` | `sensor_msgs/msg/JointState` | bridged by `spawn_robotino` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | from robot_state_publisher inside `spawn_robotino` |

(Exact robotino topic list is owned by the container's `sim_ws`; capture with `ros2 topic list` and report. The `model_ns='robotino'` argument means topics are prefixed `robotino/` without leading slash.)

### Verification commands

```bash
# Terminal 1: launch
ros2 launch ch5 simulation.launch.py
#   expect: gz sim opens, Robotino spawns on the empty world

# Terminal 2: graph checks
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash
ros2 node list                    # must include /parameter_bridge, /spawn_robotino (or robotino-namespaced equivalents)
ros2 topic list                   # must include /clock + robotino/*
ros2 topic hz /clock              # non-zero (sim time advancing)
ros2 topic info robotino/cmd_vel  # confirm a subscriber from gz bridge
```

### Action server/client check

```bash
# Terminal 3: action server
ros2 run ch5 square_action_server
#   expect: 'drive_square_action_server' node logs and stays alive

# Terminal 4: action client
ros2 run ch5 square_action_client
#   expect: client sends a goal; server begins processing; no Python errors

# Terminal 5 (parallel): verify the action exists in the graph
ros2 action list                          # must contain /drive_square
ros2 action info /drive_square            # must show 1 server + 1 client, type ch5_interfaces/action/DriveSquareAction
ros2 node info /drive_square_action_server   # subscriber: /robotino/odom; publisher: /robotino/cmd_vel
```

The action server publishes `Twist` on `/robotino/cmd_vel` and subscribes `Odometry` on `/robotino/odom`. For the goal to make the robot drive, those topics must be bridged between Gazebo and ROS — see caveats.

### Acceptance criteria (Part 1)

- [ ] `ros2 launch ch5 simulation.launch.py` starts; gz sim window opens; Robotino visible.
- [ ] `ros2 node list` contains the clock bridge and spawn_robotino node (or robotino-namespaced equivalents from sim_ws).
- [ ] `ros2 topic list` contains `/clock` and robotino-namespaced cmd_vel/odom/joint_states/tf.
- [ ] `ros2 topic hz /clock` non-zero.
- [ ] `ros2 run ch5 square_action_server` starts without tracebacks.
- [ ] `ros2 action list` shows `/drive_square`.
- [ ] `ros2 run ch5 square_action_client` sends a goal and the server accepts it (any subsequent failure to physically drive is **out of scope** here — see caveats).

---

## Part 2 — `ros2 launch ch6 simulation.launch.py`

Source: [ros2_src/ch6/launch/simulation.launch.py](../ros2_src/ch6/launch/simulation.launch.py).

### Expected nodes

| Node name | Package | Executable | Source |
|---|---|---|---|
| `teleop` | `sjtu_drone_control` | `teleop` | Direct under `xterm -e` ([ch6/launch/simulation.launch.py:15-20](../ros2_src/ch6/launch/simulation.launch.py#L15-L20)) |
| `gz sim` + sjtu_drone bridges | (various) | (various) | Include `sjtu_drone_bringup/launch/sjtu_drone_gazebo.launch.py` ([line 22-26](../ros2_src/ch6/launch/simulation.launch.py#L22-L26)) |

The exact node set under `sjtu_drone_bringup/launch/sjtu_drone_gazebo.launch.py` lives in the container's `~/sim_ws` — capture with `ros2 node list`.

### Expected topics

Topics from `sjtu_drone_bringup` (no leading slash per backend decision):

| Topic | Message type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | sim time |
| `simple_drone/cmd_vel` | `geometry_msgs/msg/Twist` | `teleop` publishes here |
| `simple_drone/takeoff` | `std_msgs/msg/Empty` | trigger takeoff |
| `simple_drone/land` | `std_msgs/msg/Empty` | trigger land |
| `simple_drone/sonar/out` | `sensor_msgs/msg/Range` | downward sonar |
| `simple_drone/odom` | `nav_msgs/msg/Odometry` | (if bridged by sjtu_drone_bringup; verify) |
| `simple_drone/camera/image_raw` | `sensor_msgs/msg/Image` | (if bridged; verify) |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | drone TF tree |

### Verification commands

```bash
# Terminal 1: launch
ros2 launch ch6 simulation.launch.py
#   expect: gz sim opens with SJTU drone world; xterm opens with teleop

# Terminal 2: graph checks
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash
ros2 node list                            # /teleop plus sjtu_drone_bringup nodes
ros2 topic list | sort                    # capture for the report; must include /clock and simple_drone/*
ros2 topic hz /clock                      # non-zero
ros2 topic info simple_drone/cmd_vel      # ≥1 subscriber (gz bridge to Twist actuator)
```

### Acceptance criteria (Part 2)

- [ ] `ros2 launch ch6 simulation.launch.py` starts; gz sim window opens; xterm teleop window opens.
- [ ] `ros2 node list` contains `/teleop` plus sjtu_drone_bringup nodes.
- [ ] `ros2 topic list` contains `/clock` and `simple_drone/cmd_vel`, `simple_drone/takeoff`, `simple_drone/land`, `simple_drone/sonar/out`.
- [ ] `ros2 topic hz /clock` non-zero.

---

## Known caveats

- **ch5 has no topic bridges beyond `/clock`.** The launch file only declares the `/clock` parameter_bridge. The robotino's `cmd_vel`/`odom`/`joint_states` must be bridged by `spawn_robotino` (from `robotino_bringup` in `sim_ws`) — verify that those topics appear in `ros2 topic list` after launch. If they don't, the action server's publish on `/robotino/cmd_vel` won't reach gz-sim and the drive-square will silently fail. Flag this and check the `sim_ws/src/robotino_bringup/spawn_robotino` source for what it bridges.
- **ch5 action server depends on container-supplied robotino.** The `robotino_description` xacro file is at `$(ros2 pkg prefix robotino_description)/share/robotino_description/urdf/robotino_platform_kinect.urdf.xacro`. If that path 404s, `xacro.process_file` raises at launch parse time — that's an environment problem, not a migration regression.
- **ch6 fully delegates to sjtu_drone_bringup.** This package already runs Harmonic per the backend migration. Any failure here is upstream — file against `rclpy-from-zero-to-hero-container/`, not against ch6.
- **Executable name in setup.py is `square_action_server`/`square_action_client`** (not the longer module name `holonomic_closed_loop_square_action_server`). The migration update plan uses the long form — both refer to the same script.
- **xterm requirement (ch6).** If `xterm` is missing, the teleop Node fails to spawn. Container ships xterm; bare hosts may need `sudo apt install xterm`.

## On pass

Tick all four `- [ ] Smoke:` lines under **Phase B4** in `migrate-ros2_src-to-jazzy-plan-update.md`.
