# Behavior Test — Phase BL5a (ch5 services + actions on Robotino)

**Phase:** BL5a (behavior layer, builds on B4 launch-level smoke test)
**Packages under test:**
- [ros2_src/ch5](../ros2_src/ch5) — 6 entry points from [setup.py:24-33](../ros2_src/ch5/setup.py#L24-L33):
  - Service pair: `square_service_server` ([holonomic_closed_loop_square_service_server.py](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py)) + `square_service_client`.
  - Action pair: `square_action_server` ([holonomic_closed_loop_square_action_server.py](../ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py)) + `square_action_client`.
  - Drone mission: `drone_mission_node_server` ([drone_mission_node_server.py](../ros2_src/ch5/ch5/drone_mission_node_server.py)) + `drone_mission_node` (Takeoff/Land services + DriveCircle action).
- [ros2_src/ch5_interfaces](../ros2_src/ch5_interfaces) — `.srv` and `.action` definitions; canary build only, no further verification needed beyond the type-resolution checks done implicitly by the ros2 CLI calls below.

Verification scope: **business logic of the DriveInSquare service, DriveSquareAction (incl. feedback + cancellation), Takeoff/Land services, and DriveCircle action**. Launch graph is already covered by [smoke-test-b4-ch5-ch6.md](smoke-test-b4-ch5-ch6.md) (per [smoke-test-b4-ch5-ch6-execution.md](executions/smoke-test-b4-ch5-ch6-execution.md) — ch6 partial-pass, ch5 was blocked on a container-side `robotino_description` install at the time the smoke ran; bl5a assumes that has since been resolved per the master plan).

Shared templates (Service / Action acceptance criteria, headless-Gazebo recipe, rosbag capture pattern, message-count helper) live in [conventions](behavior-test-conventions.md) — reference, do not duplicate.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch5_interfaces ch5
source install/setup.bash

# Canary — confirms Py 3.12 / NumPy 2 / transforms3d imports cleanly via utils.utils.math.
# This is the import path that crashed B4 smoke (see executions/smoke-test-b4-ch5-ch6-execution.md
# Issue I2); if transforms3d-0.4.2 / numpy-2-safe utils.math is in place, this returns "canary ok".
python -c "import ch5_interfaces.srv; import ch5_interfaces.action; print('iface canary ok')"
python -c "import ch5.holonomic_closed_loop_square_service_server; print('svc server canary ok')"
python -c "import ch5.holonomic_closed_loop_square_service_client; print('svc client canary ok')"
python -c "import ch5.holonomic_closed_loop_square_action_server;  print('act server canary ok')"
python -c "import ch5.holonomic_closed_loop_square_action_client;  print('act client canary ok')"
python -c "import ch5.drone_mission_node_server; print('drone server canary ok')"
python -c "import ch5.drone_mission_node;        print('drone client canary ok')"

# Container-side packages required (vendored in ~/sim_ws, not in this repo)
ros2 pkg prefix robotino_description robotino_bringup    # bl5a Parts 1+2
ros2 pkg prefix sjtu_drone_bringup                       # bl5a Part 3
which xterm                                              # drone_mission.launch.py uses xterm -e
```

If `robotino_description` is missing, **stop the phase** and re-run the B4 container fix (Issue I1 in [smoke-test-b4-ch5-ch6-execution.md](executions/smoke-test-b4-ch5-ch6-execution.md)).

Bring up the Robotino sim **headlessly** per [conventions § Headless](behavior-test-conventions.md#headless-gazebo-snippet):

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ch5 simulation.launch.py &       # ch5/launch/simulation.launch.py L23-L27 hardcodes
SIM_PID=$!                                   # gz_args='-r empty.sdf' so the GUI tries to open.
sleep 5                                      # GUI failure is acceptable; we drive verification
                                             # off ROS topics, not the gz window. If a -s override
                                             # is needed later, wrap the launch in a temporary
                                             # launch file per the conventions doc.

# Confirm sim is alive and Robotino topics are flowing
ros2 topic hz /clock           # ≥0 Hz (substitute `ros2 topic echo /clock --once` if hz hangs —
                               # known Jazzy/numpy-2 interaction documented in B4 execution Issue I5)
ros2 topic hz /robotino/cmd_vel  # 0 Hz at rest (no publisher), or a heartbeat if a controller is up
ros2 topic hz /robotino/odom     # >0 Hz once gz-sim publishes odometry (Robotino diff/holonomic plugin)
ros2 topic info /robotino/cmd_vel  # subscriber: gz parameter_bridge (proves cmd_vel→gz plumbing exists)
ros2 topic info /robotino/odom     # publisher: gz parameter_bridge
```

If `/robotino/odom` shows no publisher within 10 s after launch, abort — the action / service nodes both subscribe to `/robotino/odom` and will hang on `odometry_received_event.wait()` ([holonomic_closed_loop_square_service_server.py:68-70](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py#L68-L70), [holonomic_closed_loop_square_action_server.py:148-151](../ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py#L148-L151)).

---

## Part 1 — DriveInSquare service (service template per [conventions § Service](behavior-test-conventions.md#service))

Source: [holonomic_closed_loop_square_service_server.py](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py).

- Service name: `start_square` ([line 41](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py#L41)).
- Type: `ch5_interfaces/srv/DriveInSquare`; request fields `times` (int32) + `length` (float32); response `success` (bool) per [DriveInSquare.srv](../ros2_src/ch5_interfaces/srv/DriveInSquare.srv).
- Internals: a 60 Hz control timer drives `/robotino/cmd_vel` until `current_iteration >= times*4` (4 sides per repetition). Position tolerance is **0.02 m** ([line 25](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py#L25)); velocity is clipped to `[min_vel=0.1, max_vel=0.5]` ([lines 31-32](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py#L31-L32)).
- Default client args from [holonomic_closed_loop_square_service_client.py:51](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_client.py#L51) — `square_side=2.0, times=2`. We override to a smaller payload to keep the test under ~60 s of sim time.

### Verification

```bash
# Sim already running from Pre-flight; start the service server.
ros2 run ch5 square_service_server &
SRV_PID=$!
sleep 2

# Confirm graph
ros2 service list | grep start_square           # /start_square present
ros2 service type /start_square                 # ch5_interfaces/srv/DriveInSquare
ros2 node info /holonomic_closed_loop_square_service_server   # subscriber: /robotino/odom; publisher: /robotino/cmd_vel

# Record cmd_vel + odom for offline assertion
OUT=/tmp/bl5a_service_$$
ros2 bag record -s mcap -o "$OUT" /robotino/cmd_vel /robotino/odom &
REC_PID=$!
sleep 1

# Call the service with a small payload (1 lap, 0.5 m side -> ~4 sides, ~2 m total path,
# ~ (4*0.5)/0.5 m·s⁻¹ = 4 s nominal + tolerance overhead; allow 60 s wall-clock per the
# Service template in conventions).
timeout 75 ros2 service call /start_square ch5_interfaces/srv/DriveInSquare \
    "{times: 1, length: 0.5}" | tee /tmp/bl5a_service_resp_$$.log

# Drain
kill "$REC_PID"; wait "$REC_PID" 2>/dev/null
kill "$SRV_PID"; wait "$SRV_PID" 2>/dev/null

# Inspect response
grep -E "success=True|success: True|^response:" /tmp/bl5a_service_resp_$$.log

# Closure check (per Controller template § Square-closure error in conventions, < 0.3 m)
python plans/scripts/assert_pose_reaches_goal.py "$OUT" \
    --goal 0,0 --tol 0.3 \
    --note "closure: ‖p_end − p_start‖ < 0.3 m after one square"
#   (helper takes the first and last /robotino/odom pose from the bag and asserts the
#   Euclidean distance is below --tol. TO BE WRITTEN per conventions § Helper scripts catalog.)
```

### Acceptance criteria (Part 1)

- [ ] Service `/start_square` appears in `ros2 service list` within 2 s of the server starting.
- [ ] `ros2 service call /start_square ... "{times: 1, length: 0.5}"` returns `success: True` within 60 s wall-clock (per Service template default timeout).
- [ ] Square-closure error: ‖p_end − p_start‖ < 0.3 m on the recorded `/robotino/odom` (Controller template § Square-closure in [conventions](behavior-test-conventions.md#controller)).
- [ ] `/robotino/cmd_vel` did not saturate above `max_vel=0.5` m/s ([line 32](../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py#L32)) for more than 5 % of the run window (no rail-to-rail oscillation).
- [ ] `assert_message_count.py /robotino/cmd_vel --bag $OUT --min 60` (≥60 Twists at the 60 Hz control loop over ≥1 s of motion).

---

## Part 2 — DriveSquareAction action (action template per [conventions § Action](behavior-test-conventions.md#action))

Source: [holonomic_closed_loop_square_action_server.py](../ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py).

- Action name: `drive_square` ([line 57](../ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py#L57)).
- Type: `ch5_interfaces/action/DriveSquareAction`; goal `side_length` (float32) + `repetitions` (int32) + `time_limit` (**int32**, per [DriveSquareAction.action:4](../ros2_src/ch5_interfaces/action/DriveSquareAction.action#L4)); feedback `current_iteration` (int32) + `current_side` (int32); result `success` (bool).
- Feedback semantics ([publish_feedback @ line 248-258](../ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py#L248-L258)): a 1 Hz timer publishes `current_iteration` (cumulative side counter, 0..repetitions*4) and `current_side = current_iteration % 4`. **One feedback per second**, not one per side — so expect ≥ time_limit feedbacks, *and* ≥ 4 distinct `current_iteration` values per repetition (one per side traversed).
- The action server runs on the **same `/robotino/cmd_vel` and `/robotino/odom`** as Part 1; stop the Part 1 service server before starting it.

### Verification — happy-path with feedback

```bash
# Make sure the service server from Part 1 is dead before starting the action server
# (both publish to /robotino/cmd_vel — running both at once would cross-contaminate the test).
pgrep -f square_service_server && kill $(pgrep -f square_service_server)

# Sim still running; start the action server.
ros2 run ch5 square_action_server &
ACT_PID=$!
sleep 2

# Confirm graph (Action template § first criterion in conventions)
ros2 action list                                # /drive_square present
ros2 action info /drive_square                  # 1 server, type ch5_interfaces/action/DriveSquareAction
ros2 node info /drive_square_action_server      # subscriber: /robotino/odom; publisher: /robotino/cmd_vel

# Record cmd_vel + odom; tee feedback log
OUT=/tmp/bl5a_action_$$
ros2 bag record -s mcap -o "$OUT" /robotino/cmd_vel /robotino/odom &
REC_PID=$!

# Send goal with --feedback so the CLI prints feedback lines to stdout
timeout 75 ros2 action send_goal --feedback /drive_square ch5_interfaces/action/DriveSquareAction \
    "{side_length: 0.5, repetitions: 1, time_limit: 60}" \
    | tee /tmp/bl5a_action_feedback_$$.log

# Drain
kill "$REC_PID"; wait "$REC_PID" 2>/dev/null
kill "$ACT_PID"; wait "$ACT_PID" 2>/dev/null

# Feedback assertions
grep -c "current_iteration" /tmp/bl5a_action_feedback_$$.log
#   expect ≥ 4 occurrences  (Action template § "≥4 feedback messages" + one per side per repetition)

grep -E "current_side: [0-3]" /tmp/bl5a_action_feedback_$$.log | sort -u
#   expect at least the values {0, 1, 2, 3} to appear over a full repetition

grep "STATUS_SUCCEEDED" /tmp/bl5a_action_feedback_$$.log     # final status
grep -E "success=True|success: True" /tmp/bl5a_action_feedback_$$.log
```

### Verification — cancellation path

Per Action template § Cancellation path in [conventions](behavior-test-conventions.md#action) — send a goal, wait 5 s, cancel via Python helper, assert STATUS_CANCELED + `/cmd_vel` zeros within 1 s.

> ROS 2 Jazzy `ros2 action` does **not** ship a built-in `--cancel` flag (`ros2 action send_goal --help` lists only `--feedback`). Use a 20-line Python action-client helper that calls `goal_handle.cancel_goal_async()`. The conventions doc anticipates this — *"Cancellation path: ... `ros2 action send_goal` with `--cancel` (or via a Python action-client)"*.

```bash
# Restart action server (still using the same sim)
ros2 run ch5 square_action_server &
ACT_PID=$!
sleep 2

OUT2=/tmp/bl5a_cancel_$$
ros2 bag record -s mcap -o "$OUT2" /robotino/cmd_vel /robotino/odom &
REC_PID=$!

# Tiny Python helper — inline, written into /tmp for the run.
cat > /tmp/cancel_drive_square_$$.py <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ch5_interfaces.action import DriveSquareAction

rclpy.init()
node = Node('bl5a_cancel_helper')
cli = ActionClient(node, DriveSquareAction, 'drive_square')
cli.wait_for_server(timeout_sec=10.0)
goal = DriveSquareAction.Goal(side_length=0.5, repetitions=2, time_limit=60)
send_future = cli.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future)
handle = send_future.result()
assert handle and handle.accepted, 'goal not accepted'
node.get_logger().info('goal accepted; sleeping 5 s before cancel')
time.sleep(5.0)
cancel_future = handle.cancel_goal_async()
rclpy.spin_until_future_complete(node, cancel_future)
node.get_logger().info(f'cancel response: {cancel_future.result()}')
# Wait for the server to finalise; capture status
result_future = handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
node.get_logger().info(f'final status: {result_future.result().status}')   # 5 == STATUS_CANCELED
rclpy.shutdown()
PY

python /tmp/cancel_drive_square_$$.py 2>&1 | tee /tmp/bl5a_cancel_log_$$.log

kill "$REC_PID"; wait "$REC_PID" 2>/dev/null
kill "$ACT_PID"; wait "$ACT_PID" 2>/dev/null

# Status assertion (action_msgs/msg/GoalStatus.STATUS_CANCELED == 5)
grep "final status: 5" /tmp/bl5a_cancel_log_$$.log

# /robotino/cmd_vel goes to zero within 1 s of cancel — server's stop_robot() publishes Twist()
# in cancel_callback (holonomic_closed_loop_square_action_server.py:127-134). Use a small helper:
python plans/scripts/assert_message_count.py "$OUT2" \
    --topic /robotino/cmd_vel --window-end "after-cancel+1s" \
    --predicate "linear.x == 0 and linear.y == 0 and angular.z == 0" --expected-min 1
#   (or compute manually: last Twist within +1 s of the cancel timestamp must be all-zero.)
```

### Acceptance criteria (Part 2)

- [ ] `ros2 action list` shows `/drive_square` within 2 s of server start.
- [ ] **Happy path:** `ros2 action send_goal --feedback /drive_square ... "{side_length: 0.5, repetitions: 1, time_limit: 60}"` returns `STATUS_SUCCEEDED` within `time_limit` (Action template § "STATUS_SUCCEEDED within per-phase timeout").
- [ ] **Feedback channel alive:** ≥4 feedback messages received between accept and complete (Action template § "≥4 feedback messages"), each with `current_iteration ≥ 0` and `current_side ∈ {0, 1, 2, 3}`.
- [ ] **Final result:** `result.success == True`.
- [ ] **Cancellation:** the cancel helper receives goal status `5` (`STATUS_CANCELED`) per `action_msgs/msg/GoalStatus`.
- [ ] **Cancellation actuator zero:** within 1 s of the cancel timestamp, `/robotino/cmd_vel` reaches all-zero Twist (Action template § "/cmd_vel reaches zero within 1 s of cancellation").
- [ ] No deadlock: both server and helper exit cleanly without external SIGKILL.

---

## Part 3 — Drone mission action (DriveCircle) + Takeoff/Land services

Source: [drone_mission_node_server.py](../ros2_src/ch5/ch5/drone_mission_node_server.py) (server); [drone_mission_node.py](../ros2_src/ch5/ch5/drone_mission_node.py) (client wrapper).

- Service names: `Takeoff` (capital T) at [line 55](../ros2_src/ch5/ch5/drone_mission_node_server.py#L55) and `Land` at [line 61](../ros2_src/ch5/ch5/drone_mission_node_server.py#L61). **Note:** both services are created without a leading slash, so they appear as `/Takeoff` and `/Land` in `ros2 service list`.
- Service types: [Takeoff.srv](../ros2_src/ch5_interfaces/srv/Takeoff.srv) (`float32 altitude` → `bool success`), [Land.srv](../ros2_src/ch5_interfaces/srv/Land.srv) (no request → `bool success`).
- Action name: `DriveCircle` ([line 70](../ros2_src/ch5/ch5/drone_mission_node_server.py#L70)); type `ch5_interfaces/action/DriveCircle` — goal `radius` (float32) + `repetitions` (int32) + `time_limit` (int32) per [DriveCircle.action](../ros2_src/ch5_interfaces/action/DriveCircle.action); feedback `current_iteration` (int32); result `success` (bool).
- **Goal-callback gate** ([line 201-207](../ros2_src/ch5/ch5/drone_mission_node_server.py#L201-L207)): DriveCircle is **rejected** unless `_airborne == True` and operation is `IDLE`. Takeoff must be called *first*, and must succeed.
- Drone topic namespace per [drone_mission_node_server.py:37-39](../ros2_src/ch5/ch5/drone_mission_node_server.py#L37-L39): publishers on `/simple_drone/takeoff`, `/simple_drone/land`, `/simple_drone/cmd_vel`; sonar subscriber on `/simple_drone/sonar/out`. **Caveat — see Known caveats below**: the B4 smoke execution found the container bridges the gz sonar topic as `/simple_drone/sonar`, not `/simple_drone/sonar/out`. If the server's sonar subscriber receives no data, takeoff times out at 5 s ([line 127](../ros2_src/ch5/ch5/drone_mission_node_server.py#L127)).

### Pre-flight — drone sim

```bash
# Stop the Robotino sim from Parts 1+2 first — sjtu_drone_bringup spawns its own gz instance.
kill "$SIM_PID" 2>/dev/null
sleep 3

# Bring up only the drone half of drone_mission.launch.py (the launch also auto-starts
# both the server and the client under xterm; we want manual control of the client so we
# can call Takeoff/Land/DriveCircle ourselves).
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py world:=empty &
DRONE_SIM_PID=$!
sleep 8

# Confirm graph
ros2 topic list | grep -E "simple_drone/(odom|cmd_vel|takeoff|land|sonar)"
ros2 topic hz /simple_drone/odom               # >0 Hz
ros2 topic echo --once /simple_drone/sonar     # or /simple_drone/sonar/out — confirm which one
                                                # the bridge advertised this run (B4 caveat I4).

# Start drone_mission server (the action+service half)
ros2 run ch5 drone_mission_node_server &
DRONE_SRV_PID=$!
sleep 3

ros2 service list | grep -E "Takeoff|Land"     # /Takeoff and /Land
ros2 action list  | grep DriveCircle           # /DriveCircle
```

### Verification — Takeoff service

```bash
OUT3=/tmp/bl5a_takeoff_$$
ros2 bag record -s mcap -o "$OUT3" /simple_drone/odom /simple_drone/cmd_vel &
REC_PID=$!

timeout 15 ros2 service call /Takeoff ch5_interfaces/srv/Takeoff "{altitude: 1.0}" \
    | tee /tmp/bl5a_takeoff_resp_$$.log

kill "$REC_PID"; wait "$REC_PID" 2>/dev/null

grep -E "success=True|success: True" /tmp/bl5a_takeoff_resp_$$.log

# Altitude check — server's check_fn is `lambda x: x >= request.altitude` ([line 128]) on
# the sonar topic; we cross-check with /simple_drone/odom.pose.position.z because sonar is
# topic-name-fragile (caveat below). Use ≥ 0.9 m to allow for sonar-vs-pose offset.
python plans/scripts/assert_pose_reaches_goal.py "$OUT3" \
    --topic /simple_drone/odom \
    --field pose.pose.position.z --predicate ">= 0.9" \
    --window-end "after-takeoff+10s"
```

### Verification — DriveCircle action

```bash
OUT4=/tmp/bl5a_circle_$$
ros2 bag record -s mcap -o "$OUT4" /simple_drone/odom /simple_drone/cmd_vel &
REC_PID=$!

# linear_vel is hardcoded 0.5 m/s ([drone_mission_node_server.py:241]).
# One lap @ r=0.5 m → circumference π ≈ 3.14 m → ~6.3 s per lap.
# 2 laps + slack → 30 s time_limit is plenty.
timeout 45 ros2 action send_goal --feedback /DriveCircle ch5_interfaces/action/DriveCircle \
    "{radius: 0.5, repetitions: 2, time_limit: 30}" \
    | tee /tmp/bl5a_circle_feedback_$$.log

kill "$REC_PID"; wait "$REC_PID" 2>/dev/null

# Feedback / status assertions
grep -c "current_iteration" /tmp/bl5a_circle_feedback_$$.log    # ≥2 (one per completed lap)
grep "STATUS_SUCCEEDED" /tmp/bl5a_circle_feedback_$$.log
grep -E "success=True|success: True" /tmp/bl5a_circle_feedback_$$.log

# Trajectory shape — path length / (2π × radius) ∈ [0.5, 1.5] for "roughly circular".
# Lower bound 0.5 covers in-place drift; upper 1.5 covers ~50% over-arc from controller lag.
python plans/scripts/assert_pose_reaches_goal.py "$OUT4" \
    --topic /simple_drone/odom \
    --path-length-ratio "0.5,1.5" \
    --circular-target "radius=0.5,laps=2"
#   (helper: integrate Δ(x,y) over the recorded odom, divide by `2π·r·laps`,
#    assert ratio ∈ [--lo, --hi]. TO BE WRITTEN per conventions catalog.)
```

### Verification — Land service

```bash
OUT5=/tmp/bl5a_land_$$
ros2 bag record -s mcap -o "$OUT5" /simple_drone/odom /simple_drone/cmd_vel &
REC_PID=$!

timeout 20 ros2 service call /Land ch5_interfaces/srv/Land "{}" \
    | tee /tmp/bl5a_land_resp_$$.log

kill "$REC_PID"; wait "$REC_PID" 2>/dev/null
kill "$DRONE_SRV_PID" 2>/dev/null
kill "$DRONE_SIM_PID" 2>/dev/null

grep -E "success=True|success: True" /tmp/bl5a_land_resp_$$.log

# Altitude check — server's threshold is 0.1 m ([drone_mission_node_server.py:163]);
# allow 15 s wall-clock to settle from 1 m to ≤0.1 m. Cross-check via /simple_drone/odom.
python plans/scripts/assert_pose_reaches_goal.py "$OUT5" \
    --topic /simple_drone/odom \
    --field pose.pose.position.z --predicate "< 0.1" \
    --window-end "after-land+15s"
```

### Acceptance criteria (Part 3)

- [ ] Canary import of `ch5.drone_mission_node_server` and `ch5.drone_mission_node` succeeds.
- [ ] `/Takeoff` and `/Land` appear in `ros2 service list` within 3 s of server start; `/DriveCircle` appears in `ros2 action list`.
- [ ] **Takeoff:** `/Takeoff` call returns `success: True` within 10 s; `/simple_drone/odom.pose.position.z ≥ 0.9 m` observed within 10 s of the call.
- [ ] **DriveCircle:** action returns `STATUS_SUCCEEDED` within `time_limit=30 s`; ≥2 feedback messages received (one per completed lap, per [feedback publish @ line 277](../ros2_src/ch5/ch5/drone_mission_node_server.py#L277)); `result.success == True`.
- [ ] **Trajectory roughly circular:** integrated odom path length / (2π·radius·laps) ∈ [0.5, 1.5].
- [ ] **Land:** `/Land` call returns `success: True` within 15 s; `/simple_drone/odom.pose.position.z < 0.1 m` observed within 15 s of the call (matches the server's own 0.1 m sonar threshold).
- [ ] No server traceback over the full Takeoff → DriveCircle → Land sequence; the goal-callback gate ([line 201-207](../ros2_src/ch5/ch5/drone_mission_node_server.py#L201-L207)) was *not* triggered (it rejects DriveCircle pre-takeoff).

---

## Known caveats

- **`utils.utils.math` numpy-2 / transforms3d crash** ([smoke-test-b4-ch5-ch6-execution.md Issue I2](executions/smoke-test-b4-ch5-ch6-execution.md)). The square_action_server and square_service_server both import `euler_from_quaternion` via `utils.utils.math`. If the canary at Pre-flight fails on this, **stop the phase** — every Part below will fail at import time. Fix lives in the container repo, not in this plan.
- **`robotino_description` registration** ([smoke-test-b4-ch5-ch6-execution.md Issue I1](executions/smoke-test-b4-ch5-ch6-execution.md)). If `ros2 pkg prefix robotino_description` returns "not found", the ch5 sim cannot launch. Same fix path as above.
- **Drone sonar topic name mismatch** ([smoke-test-b4-ch5-ch6-execution.md Issue I4](executions/smoke-test-b4-ch5-ch6-execution.md)). The server source ([drone_mission_node_server.py:44](../ros2_src/ch5/ch5/drone_mission_node_server.py#L44)) subscribes to `/simple_drone/sonar/out`, but the gz bridge in this container advertises `/simple_drone/sonar`. If `/Takeoff` times out at 5 s with the drone visibly airborne, this is the cause — confirm with `ros2 topic info /simple_drone/sonar` and either remap on launch (`--ros-args -r /simple_drone/sonar/out:=/simple_drone/sonar`) or note `N/A — env (sonar topic)` for the altitude criteria and rely on `/simple_drone/odom.position.z` instead.
- **GUI rendering blocked by container DRI / Qt** (per [conventions § Common caveats](behavior-test-conventions.md#common-caveats)). gz GUI may fail to open; do not block on it. Topic content drives every acceptance criterion in this phase.
- **`ros2 topic hz` hangs in Jazzy + numpy 2.4** ([smoke-test-b4-ch5-ch6-execution.md Issue I5](executions/smoke-test-b4-ch5-ch6-execution.md)). Substitute `ros2 topic echo <topic> --once` to confirm a publisher is alive. Don't loop on `hz` for liveness.
- **`ros2 action send_goal --cancel` does not exist** on Jazzy. Use the inline Python helper in Part 2; `ros2 action send_goal --help` confirms only `--feedback`. Don't try to invoke a flag the conventions doc didn't promise.
- **`drone_mission.launch.py` auto-starts both the client and the server under `xterm -e`** ([drone_mission.launch.py:17-30](../ros2_src/ch5/launch/drone_mission.launch.py#L17-L30)). For bl5a we bring up only the server (we want manual control of Takeoff / DriveCircle / Land). Do **not** `ros2 launch ch5 drone_mission.launch.py` — that would race the auto-client against our manual calls.
- **`/Takeoff` and `/Land` are capital-T/L** ([drone_mission_node_server.py:55,61](../ros2_src/ch5/ch5/drone_mission_node_server.py#L55-L61)). The CLI is case-sensitive; `/takeoff` will not resolve.
- **Bag artefacts under `/tmp/`** are not persisted across container restarts ([conventions § Common caveats](behavior-test-conventions.md#common-caveats)). List them under `## Artefacts` in the execution record for the duration of the run only.

## On pass

Update the BL5a row in [i-m-working-on-porting-glistening-gray.md](../../../.claude/plans/i-m-working-on-porting-glistening-gray.md) and proceed to **[behavior-test-bl5b-ch6-lifecycle.md](behavior-test-bl5b-ch6-lifecycle.md)** (ch6 managed-node lifecycle on the sjtu_drone sim, depends on the drone half of Part 3 working).
