# Behavior Test — Phase BL6 (project capstone: perception + Nav2 + controller)

**Status:** ⏳ Pending execution — prerequisites met as of 2026-05-24.

**Prerequisites now met:**
- `transforms3d ≥ 0.4.2` pip override in container Dockerfile (NumPy 2 safe)
- `ros_gz_sim create` used for all spawn operations (no service bridge needed)
- B1–B7 code migration committed on `jazzy-migration` branch
- `project.world` ambient-color bug fixed ✅

**Phase note:** Full capstone — depends on the `project` simulation launching cleanly. Pre-req: B5 smoke-test issues (ambient-color bug, spawn bridge) are resolved in the current code.

**Still required:** Execute all verification steps inside `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia`. Record results in `plans/executions/behavior-test-bl6-execution.md`.

---

**Phase:** BL6 (project capstone: TurtleBot3 + Gazebo Harmonic + OpenCV detection + Nav2 + controller)

**Launches under test:**
- `ros2 launch project simulation.launch.py`
- `ros2 launch project nav2.launch.py`
- `ros2 launch project project.launch.py` (composite)

**Tracks:** camera-based line detection, Nav2 planning, robot trajectory control to waypoints.

Verification scope: **perception algorithm (yellow-line detection), Nav2 path planning, controller convergence to waypoints**. (Launch-level smoke tests already verify topics/TF; this phase verifies the algorithms actually work.)

---

## Pre-flight

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select project
source install/setup.bash
```

**Python 3.12 / NumPy 2 canary:**

```bash
python -c "from project import detection, controller, spawn_robot, parameter_node; print('project.canary: ok')"
```

**Verify external dependencies:**

```bash
ros2 pkg prefix ros_gz_sim ros_gz_bridge ros_gz_interfaces nav2_bringup cv_bridge
# expect: all resolve to /opt/ros/jazzy or sim_ws equivalent
```

**Headless Gazebo setup:**

```bash
export LIBGL_ALWAYS_SOFTWARE=1
# launches below will use gz_args:='-r -s <world>' for server-only (no window)
```

---

## Part 1 — Perception: yellow-line detection on synthetic images

Source: [ros2_src/project/project/detection.py](../ros2_src/project/project/detection.py)

**Expected behavior:** Detection node subscribes to `/camera/image_raw`, publishes `/line_position` when yellow line is detected.

**Note: Known caveat from [smoke-test-b5-project.md:244](../smoke-test-b5-project.md#L244)** — detection.py subscribes to `/camera/image_raw` but Gazebo bridge publishes `/image_raw`. This test will verify the actual topic name and either use it as-is OR document the mismatch.

**Verification commands:**

```bash
# Terminal 1: run detection node alone
ros2 run project detection

# Terminal 2: publish synthetic yellow-line image via cv_bridge helper
# (plans/scripts/image_sidecar.py — to be created; for now, simulate with rospy or use rclpy to publish Image)
# Publish Image.msg with yellow pixels in center
python3 << 'EOF'
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

rclpy.init()
node = rclpy.create_node('test_image_pub')
pub = node.create_publisher(Image, '/image_raw', 10)  # match bridge topic
bridge = CvBridge()

# Create synthetic yellow-line image (480x640, yellow line in center)
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.line(img, (320, 100), (320, 400), (0, 255, 255), 5)  # BGR: yellow
msg = bridge.cv2_to_imgmsg(img, "bgr8")

for _ in range(10):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
EOF

# Terminal 3: monitor detection output
timeout 5 ros2 topic echo /line_position --once
# expect: Float32MultiArray or custom message indicating line detected, position near center

timeout 5 ros2 topic echo /no_line_detected --once
# if this topic exists: expect False when yellow image published
```

**Acceptance criteria:**

- [ ] Detection node launches without error.
- [ ] Yellow-line synthetic image → `/line_position` published within 1s.
- [ ] Line position estimate is near image center (within 20% of image width/height).
- [ ] Black frame published → `/no_line_detected = True` (or detection suppressed).
- [ ] Address topic mismatch: confirm detection.py subscribes to `/image_raw` or `/camera/image_raw`, and test publishes to the correct topic.

---

## Part 2 — Nav2 planning: NavigateToPose action

Source: [ros2_src/project/launch/nav2.launch.py](../ros2_src/project/launch/nav2.launch.py), [ros2_src/project/config/nav2_params.yaml](../ros2_src/project/config/nav2_params.yaml)

**Expected behavior:** Nav2 receives a goal pose, computes a plan, and returns action success.

**Verification commands:**

```bash
# Terminal 1: bring up simulation
ros2 launch project simulation.launch.py & SIM=$!
sleep 8

# Terminal 2: launch Nav2 (localization only; navigation may be commented out)
ros2 launch project nav2.launch.py & NAV=$!
sleep 5

# Terminal 3: seed initial pose (AMCL needs this)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, \
    pose: {pose: {position: {x: 5.901, y: 12.182, z: 0.0}, \
                  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}}, \
           covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.06853892326654787, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}"

# Terminal 4: send navigation goal via action
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map', stamp: {sec: 0}}, \
           pose: {position: {x: 6.2, y: 10.0, z: 0.0}, \
                  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}" \
  | tee /tmp/bl6_nav_goal.log

# Capture output; should show "Goal accepted" and eventually "Goal finished with result"

# Terminal 5: check /plan topic
timeout 10 ros2 topic echo /plan --once
# expect: nav_msgs/Path message with waypoints

kill $NAV $SIM
```

**Note:** If `nav2_navigation` is commented out in `nav2.launch.py` (per smoke-test-b5 caveat), uncomment lines 40–46 or supplement with `nav2_bringup/launch/bringup_launch.py` to enable planning.

**Acceptance criteria:**

- [ ] `/initialpose` published; AMCL transitions to `active` state.
- [ ] `map → odom` transform appears after seeding initial pose.
- [ ] NavigateToPose goal accepted by action server.
- [ ] `/plan` topic publishes a Path within 5s of goal send.
- [ ] Action returns `SUCCEEDED` within 90s for the ~5m goal.
- [ ] No planning failures logged (e.g., "unable to find a valid plan").

---

## Part 3 — Controller: waypoint following

Source: [ros2_src/project/project/controller.py](../ros2_src/project/project/controller.py)

**Expected behavior:** Controller subscribes to goal/waypoint inputs and publishes `/cmd_vel` to reach them.

**Verification commands:**

```bash
# Terminal 1: simulation + controller
ros2 launch project simulation.launch.py & SIM=$!
sleep 8
ros2 run project controller & CTL=$!

# Terminal 2: record odometry during trajectory
ros2 bag record -s mcap -o /tmp/bl6_ctrl_$$ /odom /cmd_vel &
REC=$!

# Terminal 3: publish waypoint sequence (read controller source to determine topic/format)
# Assume waypoints are published to /goal or similar
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"

sleep 10  # let controller reach first waypoint

kill $REC $CTL $SIM

python plans/scripts/assert_pose_reaches_goal.py /tmp/bl6_ctrl_$$ \
  --goal-x 1.0 --goal-y 0.0 --tolerance 0.1
```

**Acceptance criteria:**

- [ ] Controller publishes `/cmd_vel` when goal is active.
- [ ] Robot odometry shows motion toward goal.
- [ ] Final pose error < 0.1m from goal (or parameterized tolerance from controller source).
- [ ] Cumulative path length < 2× straight-line distance (no excessive oscillation).

---

## Part 4 — Full system end-to-end (project.launch.py)

Source: [ros2_src/project/launch/project.launch.py](../ros2_src/project/launch/project.launch.py)

**Expected behavior:** Single launch brings up simulation, Nav2, detection, controller; all subsystems active.

**Verification commands:**

```bash
# Terminal 1: composite launch
ros2 launch project project.launch.py

# Terminal 2 (after ~20s for everything to come up): verify subsystems
ros2 node list | grep -E "(detector|controller|amcl|spawn)"
# expect: all nodes present

ros2 topic list | grep -E "(plan|cmd_vel|line_position|image_raw)"
# expect: perception and planning topics present

# Publish a goal and observe robot motion
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0}, orientation: {w: 1.0}}}}"

# Verify goal is accepted and plan published
timeout 10 ros2 topic echo /plan --once
```

**Acceptance criteria:**

- [ ] project.launch.py starts cleanly (all nodes up within 30s).
- [ ] Simulation, Nav2, and controller all active and publishing.
- [ ] Perception output present on detection topics.
- [ ] Navigation goal triggers plan generation.

---

## Known caveats

- **`detection.py` topic mismatch** (`/camera/image_raw` vs `/image_raw`) — this test documents the actual behavior. If mismatch, either fix in source OR test publishes to both topics.
- **`nav2_navigation` may be commented out** — Part 2 requires planning; if disabled, uncomment lines 40–46 of `nav2.launch.py`.
- **AMCL initial pose seeding** — AMCL may not publish `map → odom` transform until `/initialpose` is explicitly published.
- **Container DRI permissions** — Gazebo window disabled; all checks via topic echo / action client.
- **Synthetic image helper** — `plans/scripts/image_sidecar.py` (to be created) or inline Python used for perception testing.

---

## On pass

- [ ] All four parts report PASS.
- Proceed to Phase BL7 (advanced projects: ekf_localization, ekf_slam, bug_planning).

---

## Acceptance criteria summary

| Criterion | Status |
|-----------|--------|
| Yellow-line image → /line_position within 1s | [ ] |
| Line position near image center (±20%) | [ ] |
| Black image → /no_line_detected = True | [ ] |
| Detection topic name verified (image_raw or camera/image_raw) | [ ] |
| Nav2 /plan published within 5s of goal | [ ] |
| NavigateToPose action SUCCEEDED within 90s | [ ] |
| map → odom transform exists after /initialpose | [ ] |
| Controller reaches waypoint within 0.1m | [ ] |
| Path length < 2× straight-line | [ ] |
| project.launch.py composite succeeds | [ ] |
