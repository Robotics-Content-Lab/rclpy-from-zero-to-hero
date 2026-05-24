# Behavior Test — Phase BL1 (pub/sub basics)

**Phase:** BL1 (pub/sub basics)
**Launches / runs under test:**
- `ros2 launch ch1 first_pub_sub.launch.py`
- `ros2 launch ch1 pub-sub.launch.py` (end-to-end via `/house_points` remap)
- `ros2 run ch1 simple_publisher` + `ros2 run ch1 simple_subscriber` (on `/example_topic`)
- `ros2 run ch1 first_node` (timer-only smoke)
- `ros2 launch hw1_pkg hw1.launch.py` (without Gazebo include — see Known caveats)
- `ros2 launch preface params.launch.py`
- `ros2 launch preface namespace.launch.py`
- `ros2 bag info` / `ros2 bag play` on `ros2_src/preface/bagfiles/car_demo/`
- `colcon test --packages-select utils` (existing `test_tf_transformations.py`)

**Tracks:** the "pub/sub basics" verification surface from [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md). Per-criterion patterns (counter monotonicity, drop rate, namespace remap, params load/read, canary imports) live in [conventions](behavior-test-conventions.md); this file references but does not duplicate them.

Verification scope: **message flow, parameter loading, namespace remap, and bag playback** — i.e. the business-logic surface that launch-only smoke tests (B1) skip. Headless-first: no `rqt`, `rviz2`, or Gazebo window required.

## Pre-flight

Inside the Jazzy container (`ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia`):

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch1 hw1_pkg preface utils
source install/setup.bash
```

Python 3.12 / NumPy 2 import canary for the four packages (per [conventions](behavior-test-conventions.md#canary-imports)):

```bash
python -c "import ch1.first_publisher, ch1.first_subscriber, ch1.simple_publisher, ch1.simple_subscriber, ch1.first_node; print('ch1 ok')"
python -c "import hw1_pkg.publisher_node, hw1_pkg.subscriber_node; print('hw1_pkg ok')"
python -c "import preface.params, preface.namespace_example1, preface.namespace_example2; print('preface ok')"
python -c "from utils.utils import math, rviz_markers; print('utils ok')"
```

Each canary must print its `ok` line and exit 0. A `ModuleNotFoundError` or numpy-2 traceback here aborts BL1.

---

## Part 1 — `ch1` pub/sub message flow

Source files (entry points per [ros2_src/ch1/setup.py:24-32](../ros2_src/ch1/setup.py#L24-L32)):
- `first_publisher` → [ros2_src/ch1/ch1/first_publisher.py](../ros2_src/ch1/ch1/first_publisher.py). Node `dumbledore`, publishes `example_interfaces/msg/String` on `/points`, 1 Hz timer ([L15](../ros2_src/ch1/ch1/first_publisher.py#L15)), `self.points` increments by 10 per tick ([L18](../ros2_src/ch1/ch1/first_publisher.py#L18)), message body `"{points} points for Gryffindor"` ([L20](../ros2_src/ch1/ch1/first_publisher.py#L20)).
- `first_subscriber` → [ros2_src/ch1/ch1/first_subscriber.py](../ros2_src/ch1/ch1/first_subscriber.py). Node `ron`, subscribes to `/some_points` ([L12](../ros2_src/ch1/ch1/first_subscriber.py#L12)), logs each message with an incrementing `self.msg_cnt`.
- `simple_publisher` → `/example_topic`, counter formatted into `data` ([ch1/simple_publisher.py:9-19](../ros2_src/ch1/ch1/simple_publisher.py#L9-L19)).
- `simple_subscriber` → subscribes `/example_topic`, logs each message ([ch1/simple_subscriber.py:9-18](../ros2_src/ch1/ch1/simple_subscriber.py#L9-L18)).
- `first_node` → no pub/sub, 1 Hz log-only timer ("You are a wizard Harry") ([ch1/first_node.py:13-18](../ros2_src/ch1/ch1/first_node.py#L13-L18)).

Topic wiring per [ros2_src/ch1/launch/first_pub_sub.launch.py](../ros2_src/ch1/launch/first_pub_sub.launch.py):
- `first_pub_sub.launch.py` does **not** wire publisher to subscriber (`/points` ≠ `/some_points` — intentional book pedagogy).
- End-to-end flow is in [ros2_src/ch1/ch1/pub-sub.launch.py](../ros2_src/ch1/ch1/pub-sub.launch.py): both endpoints remap to `/house_points`.

### Verification commands

```bash
# 1A. Counter monotonicity + non-negative payload, plus 10 s count.
ros2 run ch1 first_publisher & PUB_PID=$!
sleep 2
ros2 topic echo --once /points   # expect data: "<N> points for Gryffindor", N >= 10
ros2 topic echo /points > /tmp/bl1_points.log & ECHO_PID=$!
sleep 10; kill $ECHO_PID $PUB_PID 2>/dev/null; wait 2>/dev/null
grep -oE 'data: .[0-9]+ points' /tmp/bl1_points.log | grep -oE '[0-9]+' \
  | awk 'NR==1{p=$1;next} {if($1<p){print "REGRESSION";exit 1} p=$1} END{print "ok"}'
COUNT=$(grep -c '^data:' /tmp/bl1_points.log); echo "count=$COUNT"; test "$COUNT" -ge 9

# 1B. End-to-end flow via pub-sub.launch.py (snape → /house_points → harry).
#     Subscriber prints "I heard msg #..." per received message; drop rate from echo vs log.
ros2 launch ch1 pub-sub.launch.py 2>&1 | tee /tmp/bl1_pubsub.log & LAUNCH_PID=$!
sleep 3
ros2 topic list | grep -x /house_points
COUNT_HP=$(timeout 10 ros2 topic echo /house_points 2>/dev/null | grep -c '^data:')
sleep 2; kill $LAUNCH_PID 2>/dev/null; wait 2>/dev/null
HEARD=$(grep -c 'I heard msg #' /tmp/bl1_pubsub.log)
echo "house_points=$COUNT_HP heard=$HEARD"
test "$COUNT_HP" -ge 9 && test "$HEARD" -ge 9
# Drop rate = (COUNT_HP - HEARD) / COUNT_HP — assert < 5%.

# 1C. simple_publisher / simple_subscriber on /example_topic.
ros2 run ch1 simple_publisher & SP=$!
ros2 run ch1 simple_subscriber 2>&1 | tee /tmp/bl1_simple.log & SS=$!
sleep 10; kill $SP $SS 2>/dev/null; wait 2>/dev/null
test "$(grep -c 'Received: Hello, ROS 2' /tmp/bl1_simple.log)" -ge 9

# 1D. first_node 1 Hz log smoke.
ros2 run ch1 first_node 2>&1 | tee /tmp/bl1_firstnode.log & FN=$!
sleep 5; kill $FN 2>/dev/null; wait 2>/dev/null
test "$(grep -c 'You are a wizard Harry' /tmp/bl1_firstnode.log)" -ge 4
```

### Acceptance criteria (Part 1)

- [ ] `first_publisher` publishes ≥ 9 messages in 10 s on `/points` (1 Hz × 10 s × 0.95).
- [ ] Counter field in `/points` is monotone non-decreasing and ≥ 10 on first echo.
- [ ] `pub-sub.launch.py` wires publisher `/points` and subscriber `/some_points` both to `/house_points`; `/house_points` appears in `ros2 topic list`.
- [ ] `first_subscriber` logs an "I heard msg #" line for each received message (≥ 9 over 10 s) — drop rate < 5 %.
- [ ] `simple_publisher` → `simple_subscriber` over `/example_topic` shows ≥ 9 received messages in 10 s.
- [ ] `first_node` log line "You are a wizard Harry" appears ≥ 4 times in 5 s (1 Hz × 5 × 0.8).

---

## Part 2 — `hw1_pkg` pub/sub

Source files (entry points per [ros2_src/hw1_pkg/setup.py:24-29](../ros2_src/hw1_pkg/setup.py#L24-L29)):
- `publisher_node` → [hw1_pkg/publisher_node.py](../ros2_src/hw1_pkg/hw1_pkg/publisher_node.py). Publishes `geometry_msgs/msg/Twist` on `/cmd_vel`, 1 Hz, fixed `linear.x=0.5, angular.z=0.5` ([L13-17](../ros2_src/hw1_pkg/hw1_pkg/publisher_node.py#L13-L17)).
- `subscriber_node` → [hw1_pkg/subscriber_node.py](../ros2_src/hw1_pkg/hw1_pkg/subscriber_node.py). Subscribes to `nav_msgs/msg/Odometry` on `/odom` ([L9](../ros2_src/hw1_pkg/hw1_pkg/subscriber_node.py#L9)), logs each `Received Odometry message: ...`.

[ros2_src/hw1_pkg/launch/hw1.launch.py](../ros2_src/hw1_pkg/launch/hw1.launch.py) also includes `turtlebot3_gazebo/empty_world.launch.py` ([L11-18](../ros2_src/hw1_pkg/launch/hw1.launch.py#L11-L18)) — that side is **out of scope** for BL1 (Gazebo behavior is BL6/BL7's job). BL1 verifies only the pub/sub plumbing.

### Verification commands

```bash
# 2A. Publisher emits non-zero Twist at ~1 Hz.
ros2 run hw1_pkg publisher_node &
PUB_PID=$!
sleep 2
ros2 topic echo --once /cmd_vel
#   expect linear.x: 0.5, angular.z: 0.5 (positive, non-zero per Twist constants)
COUNT=$(timeout 10 ros2 topic echo /cmd_vel 2>/dev/null | grep -c '^linear:')
kill $PUB_PID 2>/dev/null; wait 2>/dev/null
echo "cmd_vel count=$COUNT"
test "$COUNT" -ge 9

# 2B. Subscriber receives a synthetic /odom message and logs it.
#     (No Gazebo: publish a single /odom message ourselves.)
ros2 run hw1_pkg subscriber_node 2>&1 | tee /tmp/bl1_hw1_sub.log &
SUB_PID=$!
sleep 2
ros2 topic pub --once /odom nav_msgs/msg/Odometry \
  "{header: {frame_id: 'odom'}, child_frame_id: 'base_link', \
    pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}"
sleep 2
kill $SUB_PID 2>/dev/null; wait 2>/dev/null
grep -q 'Received Odometry message' /tmp/bl1_hw1_sub.log && echo "ok"
```

### Acceptance criteria (Part 2)

- [ ] `hw1_pkg.publisher_node` publishes ≥ 9 `Twist` messages on `/cmd_vel` in 10 s with `linear.x=0.5, angular.z=0.5`.
- [ ] `hw1_pkg.subscriber_node` logs a `Received Odometry message: …` line for a synthetic `/odom` publication.

---

## Part 3 — `preface` (params + namespaces + tf bag)

Source files (entry points per [ros2_src/preface/setup.py:25-31](../ros2_src/preface/setup.py#L25-L31)):
- `params` → [preface/params.py](../ros2_src/preface/preface/params.py). Node `differential_drive_node`, declares `wheel_radius`, `baseline_distance`, `max_linear_velocity`, `max_angular_velocity`. Values loaded from YAML or launch overrides.
- `namespace_example1` → [preface/namespace_example1.py](../ros2_src/preface/preface/namespace_example1.py). Uses `~/topic` (relative to node namespace).
- `namespace_example2` → [preface/namespace_example2.py](../ros2_src/preface/preface/namespace_example2.py). Hard-codes `namespace='example_namespace'` in the Node constructor ([L8](../ros2_src/preface/preface/namespace_example2.py#L8)).

Param YAML values per [ros2_src/preface/params.yaml](../ros2_src/preface/params.yaml):
- `wheel_radius=0.2`, `baseline_distance=0.4`, `max_linear_velocity=0.8`, `max_angular_velocity=2.0`.

[ros2_src/preface/launch/params.launch.py](../ros2_src/preface/launch/params.launch.py) starts two nodes:
- `diff_drive_controller_default` with overrides `wheel_radius=0.5, baseline_distance=1.9, max_linear_velocity=0.6, max_angular_velocity=1.0` ([L11-15](../ros2_src/preface/launch/params.launch.py#L11-L15)).
- `differential_drive_node` with `parameters=['/home/ros_user/ros2_ws/src/preface/params.yaml']` ([L22](../ros2_src/preface/launch/params.launch.py#L22)) — i.e. the YAML defaults.

[ros2_src/preface/launch/namespace.launch.py](../ros2_src/preface/launch/namespace.launch.py) starts four nodes; the `robot2`/`robot22` instances run under `namespace='robot2_namespace'` ([L16, L29](../ros2_src/preface/launch/namespace.launch.py#L16)).

### Verification commands — params

```bash
ros2 launch preface params.launch.py &
LAUNCH_PID=$!
sleep 3

# 3A.1 — node names exist
ros2 node list | grep -E '/differential_drive_node|/diff_drive_controller_default'

# 3A.2 — YAML-driven node has params from params.yaml.
ros2 param list /differential_drive_node | sort
#   expect at least: baseline_distance, max_angular_velocity, max_linear_velocity, wheel_radius
test "$(ros2 param get /differential_drive_node wheel_radius | grep -oE '[0-9.]+$')" = "0.2"
test "$(ros2 param get /differential_drive_node baseline_distance | grep -oE '[0-9.]+$')" = "0.4"
test "$(ros2 param get /differential_drive_node max_linear_velocity | grep -oE '[0-9.]+$')" = "0.8"
test "$(ros2 param get /differential_drive_node max_angular_velocity | grep -oE '[0-9.]+$')" = "2.0"

# 3A.3 — launch-overridden node has the override values.
test "$(ros2 param get /diff_drive_controller_default wheel_radius | grep -oE '[0-9.]+$')" = "0.5"
test "$(ros2 param get /diff_drive_controller_default baseline_distance | grep -oE '[0-9.]+$')" = "1.9"

kill $LAUNCH_PID 2>/dev/null; wait 2>/dev/null
```

### Verification commands — namespaces

```bash
ros2 launch preface namespace.launch.py &
LAUNCH_PID=$!
sleep 3

# 3B.1 — robot2 in namespace
ros2 node list | grep -x /robot2_namespace/robot2
ros2 node list | grep -x /robot2_namespace/robot22

# 3B.2 — namespace_example2 hard-codes namespace='example_namespace' (overrides the
#         launch-file namespace for the non-namespaced instance). Verify topics:
ros2 topic list | grep -E '/example_namespace/robot11/topic|/robot2_namespace/example_namespace/robot22/topic|/robot2_namespace/robot2/topic'
#   expect at least one namespaced topic in the output.

kill $LAUNCH_PID 2>/dev/null; wait 2>/dev/null
```

### Verification commands — car_demo tf bag

```bash
# 3C.1 — bag is parsable.
ros2 bag info /workspaces/rclpy-from-zero-to-hero/ros2_src/preface/bagfiles/car_demo/
#   expect: storage=mcap, topics include /tf, /tf_static, /prius/front_camera/image_raw,
#   /prius/center_laser/scan; message_count=836 (per metadata.yaml).

# 3C.2 — playback delivers tf, and tf2_echo returns a transform.
ros2 bag play /workspaces/rclpy-from-zero-to-hero/ros2_src/preface/bagfiles/car_demo/ --loop &
PLAY_PID=$!
sleep 2
# Pick any frame pair guaranteed by the bag's /tf_static. base_link → some sensor
# is common in Prius bags; if you don't know the frames, dump tf_static first:
ros2 topic echo --once /tf_static | grep frame_id | head -10
# Then assert a transform shows up between two of those frames within 5 s:
timeout 5 ros2 run tf2_ros tf2_echo <parent_frame> <child_frame> | grep -q 'Translation' && echo "tf ok"
kill $PLAY_PID 2>/dev/null; wait 2>/dev/null
```

### Acceptance criteria (Part 3)

- [ ] `params.launch.py` starts both nodes; `ros2 node list` shows `/differential_drive_node` and `/diff_drive_controller_default`.
- [ ] `differential_drive_node` `wheel_radius=0.2`, `baseline_distance=0.4`, `max_linear_velocity=0.8`, `max_angular_velocity=2.0` — matches [params.yaml](../ros2_src/preface/params.yaml).
- [ ] `diff_drive_controller_default` `wheel_radius=0.5`, `baseline_distance=1.9` — matches launch-file overrides.
- [ ] `namespace.launch.py` exposes namespaced nodes/topics under `/robot2_namespace/...`.
- [ ] `ros2 bag info` parses `bagfiles/car_demo/` without error and reports the four expected topics (`/tf`, `/tf_static`, `/prius/front_camera/image_raw`, `/prius/center_laser/scan`).
- [ ] During `ros2 bag play`, `tf2_echo <parent> <child>` returns a transform within 5 s for at least one frame pair from `/tf_static`.

---

## Part 4 — `utils` library

`utils` is a library-only package (no `console_scripts`; entry list is empty in [ros2_src/utils/setup.py:25-28](../ros2_src/utils/setup.py#L25-L28)). The Python module layout is doubly nested — package `utils`, modules under `utils/utils/` — so the import path is `from utils.utils import math, rviz_markers` (see [conventions § canary imports](behavior-test-conventions.md#canary-imports)).

Coverage:

- **Canary import** (already issued in Pre-flight): `from utils.utils import math, rviz_markers`. Guards numpy-2 / transforms3d regressions for any caller.
- **Existing unit test**: [ros2_src/utils/test/test_tf_transformations.py](../ros2_src/utils/test/test_tf_transformations.py) — round-trips Euler↔quaternion via `tf_transformations.quaternion_from_euler` / `euler_from_quaternion` and asserts < 1e-9 error per axis. Per [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md), this test already passes; BL1 re-asserts it as a guardrail.

### Verification commands

```bash
# 4A. Canary (already in Pre-flight; repeat here for the local record).
python -c "from utils.utils import math, rviz_markers; print('utils canary ok')"

# 4B. Re-run the existing pytest under colcon.
cd /workspaces/rclpy-from-zero-to-hero
colcon test --packages-select utils --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/utils \
  | grep -E 'test_tf_transformations|0 errors, 0 failures'
```

### Acceptance criteria (Part 4)

- [ ] `from utils.utils import math, rviz_markers` succeeds under Python 3.12 + NumPy 2.
- [ ] `colcon test --packages-select utils` exits 0 with `test_tf_transformations.py` reporting 0 errors, 0 failures.

---

## Known caveats

- **Counter starts at 10, not 0.** [first_publisher.py:18](../ros2_src/ch1/ch1/first_publisher.py#L18) increments *before* publishing, so the first message has `data: "10 points for Gryffindor"`. The "non-negative counter" criterion is therefore "≥ 10 on first echo, monotone non-decreasing after". Do not flag this as a regression.
- **Topic mismatch in `first_pub_sub.launch.py` is intentional.** `/points` (publisher) ≠ `/some_points` (subscriber); end-to-end flow is only achievable via `pub-sub.launch.py`. Part 1's drop-rate criterion uses `pub-sub.launch.py`, not `first_pub_sub.launch.py`.
- **Node-name typo in [first_pub_sub.launch.py:8](../ros2_src/ch1/launch/first_pub_sub.launch.py#L8).** The launch file overrides the publisher's node name to `'doumbledore'` (with an extra `u`). [first_publisher.py:9](../ros2_src/ch1/ch1/first_publisher.py#L9) hard-codes `'dumbledore'`. The launch-file value wins. B1's smoke test already accepts `/doumbledore`; keep using that spelling for `ros2 node list` checks.
- **`hw1.launch.py` pulls in Gazebo.** [hw1_pkg/launch/hw1.launch.py:11-18](../ros2_src/hw1_pkg/launch/hw1.launch.py#L11-L18) includes `turtlebot3_gazebo/empty_world.launch.py`. BL1 verifies only pub/sub plumbing — run the two nodes directly via `ros2 run` and synthesize the `/odom` message, as in Part 2. Full launch behavior is BL6/BL7.
- **`car_demo` bag is NOT metadata-only.** The inventory in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md) says metadata-only, but the directory contains `metadata.yaml` plus two payload mcap files (`car_demo_0.mcap`, `car_demo_1.mcap`) referenced from `relative_file_paths` in the metadata; `message_count: 836` over four topics including `/tf` and `/tf_static`. The original inventory entry is therefore wrong — playback + `tf2_echo` *is* exercisable, so Part 3 requires the live-tf check rather than degrading to "bag info parses without error".
- **`namespace_example2.py` hard-codes its namespace.** [namespace_example2.py:8](../ros2_src/preface/preface/namespace_example2.py#L8) sets `namespace='example_namespace'` inside the Node constructor; this stacks with the launch-file namespace (so the launch-file `robot2_namespace` instance ends up under `/robot2_namespace/example_namespace/robot22`). Don't be surprised by the nested namespace in `ros2 topic list`.
- **Param YAML path is container-absolute.** [params.launch.py:22](../ros2_src/preface/launch/params.launch.py#L22) hardcodes `/home/ros_user/ros2_ws/src/preface/params.yaml`. Verification commands assume the workspace is mounted at that path inside the Jazzy container; outside the container the launch will fail loading the YAML. This is the same restriction observed in B1's smoke test and is not a BL1-introduced regression.
- **`utils` import path is doubly nested.** The package and inner module share the name `utils`, so the right import is `from utils.utils import math, rviz_markers`, **not** `from utils import math`. Driven by [ros2_src/utils/setup.py:4](../ros2_src/utils/setup.py#L4) (`modules = f'{package_name}/utils'`).

## On pass

Tick the BL1 row in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md) and record the run in `plans/executions/behavior-test-bl1-execution.md` (use `behavior-test-template.md`). Next phase: [behavior-test-bl2-ch2-qos-threading.md](behavior-test-bl2-ch2-qos-threading.md) — QoS profiles, callback groups, and `MultiThreadedExecutor` behavior on `ch2` / `hw2_pkg`.
