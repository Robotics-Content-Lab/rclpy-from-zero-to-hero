# Behavior Test — Phase BL5b (ch6 lifecycle nodes on sjtu_drone)

**Phase:** BL5b (behavior layer, builds on B4 launch-level smoke test).
**Packages under test:** [ros2_src/ch6](../ros2_src/ch6) — two executables registered in [setup.py:24-29](../ros2_src/ch6/setup.py#L24-L29): `drone_lifecycle` and `drone_lifecycle2`. Launch [ch6/launch/simulation.launch.py](../ros2_src/ch6/launch/simulation.launch.py) delegates entirely to `sjtu_drone_bringup/launch/sjtu_drone_gazebo.launch.py` plus a teleop Node under `xterm -e`.

Verification scope: **the unique signal lifecycle gives** — that `activate` makes the node functionally responsive and `deactivate` makes it stop responding, queried through the standard lifecycle CLI plus topic-level side-effects. State-word transitions alone are weak evidence; this phase pairs each state read with a behavioral probe.

Shared templates (Lifecycle § acceptance, headless Gazebo, message-count helper) live in [behavior-test-conventions.md](behavior-test-conventions.md) — reference, do not duplicate.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch6
source install/setup.bash

# Canary — confirms Py 3.12 / NumPy 2 imports cleanly through the lifecycle stack
python -c "import ch6.drone_lifecycle; print('canary v1 ok')"
python -c "import ch6.drone_lifecycle2; print('canary v2 ok')"
#   Both modules pull rclpy.lifecycle + lifecycle_msgs + std_msgs/geometry_msgs/sensor_msgs.
#   Failure here means a downstream rclpy or msg interface regression — see
#   plans/errors/fixed/tf_transformations-numpy2-incompatibility.md for the template.
```

sjtu_drone is vendored in the container's `~/sim_ws` (not this repo). Confirm both bringup and control packages resolve:

```bash
ros2 pkg prefix sjtu_drone_bringup sjtu_drone_control
which xterm    # teleop runs under `xterm -e` per simulation.launch.py:19
```

Bring sjtu_drone up headlessly (per [conventions § Headless Gazebo](behavior-test-conventions.md#headless-gazebo-snippet)). Because [simulation.launch.py:22-26](../ros2_src/ch6/launch/simulation.launch.py#L22-L26) hard-codes the gz invocation through the include, override via env + a separate xterm-free run rather than editing the launch file:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
# Direct sjtu_drone bringup launch (skips the ch6 xterm teleop Node, which would fail without DRI)
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py &
SIM=$!
sleep 8

# Confirm the simulator is alive and drone topics are flowing
ros2 topic hz /clock                                # non-zero
ros2 topic list | grep '^/simple_drone/' | sort     # expect /simple_drone/{cmd_vel,takeoff,land,...}
ros2 topic info /simple_drone/cmd_vel               # ≥1 subscriber (gz Twist bridge)
ros2 topic info /simple_drone/takeoff               # ≥1 subscriber (gz Empty bridge)
ros2 topic info /simple_drone/land                  # ≥1 subscriber (gz Empty bridge)
```

If `/clock` reports `no new messages` for >5 s, abort — gz-sim is not up and every downstream assertion would just register "node alive, no effect" (cf. [executions/smoke-test-b5-project-execution.md](executions/smoke-test-b5-project-execution.md) Issue A). If `/simple_drone/cmd_vel` shows zero subscribers, the gz bridge did not start; flag against `sim_ws` per the B4 caveat ("ch6 fully delegates to sjtu_drone_bringup; failures are upstream").

---

## Part 1 — `drone_lifecycle`: configure → activate → deactivate → cleanup → shutdown

Source: [ch6/drone_lifecycle.py](../ros2_src/ch6/ch6/drone_lifecycle.py). Default node name `drone_lifecycle` ([line 12](../ros2_src/ch6/ch6/drone_lifecycle.py#L12)). Behavior of each callback:

- `on_configure` ([L42-L60](../ros2_src/ch6/ch6/drone_lifecycle.py#L42-L60)): cleans any existing subs, reads required `droneNS` parameter, then creates subscribers on **global** `/takeoff`, `/land`, `/cmd_vel` (note: not namespaced — see [`_init_subscribers`, L37-L40](../ros2_src/ch6/ch6/drone_lifecycle.py#L37-L40)). Returns ERROR if `droneNS` is empty.
- `on_activate` ([L67-L88](../ros2_src/ch6/ch6/drone_lifecycle.py#L67-L88)): destroys existing pubs, creates publishers on `{droneNS}/takeoff`, `{droneNS}/land`, `{droneNS}/cmd_vel`, then **checks every publisher has ≥1 subscriber**; returns ERROR otherwise. With `droneNS=simple_drone`, those map onto the live gz-bridged topics from pre-flight.
- `on_deactivate` ([L90-L99](../ros2_src/ch6/ch6/drone_lifecycle.py#L90-L99)): publishes a zero `Twist` via `hover()` if currently flying. **Does not** destroy publishers/subscribers — but in `cb_cmd_vel` the node re-publishes incoming Twist via `pubCmd`; once *destroyed* in cleanup, the callback would NPE. The unique behavioral signal: subscriber callbacks themselves are torn down in cleanup, not deactivate. **Important:** state transition itself does not break the pub/sub graph — what we test instead is that `pubCmd` republishes Twist when callbacks are wired.
- `on_cleanup` ([L125-L137](../ros2_src/ch6/ch6/drone_lifecycle.py#L125-L137)): destroys subs and pubs.
- `on_shutdown` ([L139-L153](../ros2_src/ch6/ch6/drone_lifecycle.py#L139-L153)): calls `land()` (publishes Empty on `{ns}/land`), tears down resources, sets the `done_event` so the main thread can join.

The node parameter `droneNS` defaults to `''` (empty string); pass `simple_drone` on the CLI so `on_configure` succeeds and the activate-time publisher targets the live gz-bridged topics:

```bash
ros2 run ch6 drone_lifecycle --ros-args -p droneNS:=simple_drone &
NODE=$!
sleep 2

# Initial state
ros2 lifecycle get /drone_lifecycle
#   expect: unconfigured  [1]

# configure → inactive
ros2 lifecycle set /drone_lifecycle configure                       # expect: SUCCESS
ros2 lifecycle get /drone_lifecycle                                 # expect: inactive  [2]

# activate → active
ros2 lifecycle set /drone_lifecycle activate                        # expect: SUCCESS
#   internally _check_sub_count requires each of pubTakeOff/pubLand/pubCmd to have ≥1 subscriber;
#   the gz parameter_bridge on /simple_drone/{takeoff,land,cmd_vel} satisfies this — see pre-flight.
ros2 lifecycle get /drone_lifecycle                                 # expect: active  [3]

# Behavioral side-effect probes are in Part 3; complete the state cycle first.

# deactivate → inactive
ros2 lifecycle set /drone_lifecycle deactivate                      # expect: SUCCESS
ros2 lifecycle get /drone_lifecycle                                 # expect: inactive  [4]

# cleanup → unconfigured
ros2 lifecycle set /drone_lifecycle cleanup                         # expect: SUCCESS
ros2 lifecycle get /drone_lifecycle                                 # expect: unconfigured  [5]

# Re-configure and re-activate (round-trip — proves cleanup did not strand state)
ros2 lifecycle set /drone_lifecycle configure                       # expect: SUCCESS
ros2 lifecycle set /drone_lifecycle activate                        # expect: SUCCESS

# shutdown → finalized (terminal)
ros2 lifecycle set /drone_lifecycle shutdown                        # expect: SUCCESS
ros2 lifecycle get /drone_lifecycle                                 # expect: finalized  [6]

# The node sets done_event on shutdown and the main thread joins, so NODE exits on its own.
wait $NODE 2>/dev/null
```

Use the [Lifecycle template in conventions](behavior-test-conventions.md#lifecycle) — every `lifecycle set` returning SUCCESS plus matching `lifecycle get` covers state-word verification.

---

## Part 2 — `drone_lifecycle2`: same flow + v2-specific behavior

Source: [ch6/drone_lifecycle2.py](../ros2_src/ch6/ch6/drone_lifecycle2.py). **Same default node name `drone_lifecycle`** ([L19](../ros2_src/ch6/ch6/drone_lifecycle2.py#L19)) — run **only one** lifecycle node at a time, kill Part 1's node first. Key differences from v1:

| Aspect | v1 (`drone_lifecycle`) | v2 (`drone_lifecycle2`) |
|---|---|---|
| `droneNS` param | required, errors if empty | not used; topic names hard-coded to `/simple_drone/...` |
| Subscribers (on_configure) | 3 global topics: `/takeoff`, `/land`, `/cmd_vel` | 7 topics: `/simple_drone/{sonar, imu, front/image_raw, bottom/image_raw, gt_pose, state, cmd_mode}` ([L53-L59](../ros2_src/ch6/ch6/drone_lifecycle2.py#L53-L59)) |
| `on_configure` pub-sub check | none | reads `self.pubTakeOff.get_subscription_count()` etc. **before** any publisher is created ([L62-L65](../ros2_src/ch6/ch6/drone_lifecycle2.py#L62-L65)) — this dereferences `None` and raises `AttributeError`, caught by the try/except at [L69](../ros2_src/ch6/ch6/drone_lifecycle2.py#L69) and returns `TransitionCallbackReturn.FAILURE`. **Configure is expected to fail.** |
| Publishers (on_activate) | 3 namespaced: `{ns}/takeoff`, `{ns}/land`, `{ns}/cmd_vel` | 6 hard-coded: `/simple_drone/{takeoff,land,reset,posctrl,cmd_vel,dronevel_mode}` ([L82-L87](../ros2_src/ch6/ch6/drone_lifecycle2.py#L82-L87)) |
| `on_activate` precondition | publishers must have ≥1 sub | refuses if `isFlying` already true ([L78-L80](../ros2_src/ch6/ch6/drone_lifecycle2.py#L78-L80)) |
| `on_deactivate` precondition | none | refuses if **not** `isFlying` ([L98-L100](../ros2_src/ch6/ch6/drone_lifecycle2.py#L98-L100)) — and `isFlying` is never set inside lifecycle callbacks, so the CLI deactivate transition will FAIL on a fresh activate |
| `on_shutdown` action | calls `land()`, signals done_event, joins | logs and returns SUCCESS only — no actuation; node keeps spinning under `rclpy.spin()` ([L200-L204](../ros2_src/ch6/ch6/drone_lifecycle2.py#L200-L204)) |

**Expected v2-specific behavior under the same CLI flow:**

```bash
# Kill Part 1's node first (lifecycle nodes share the default name "drone_lifecycle")
kill $NODE 2>/dev/null; wait $NODE 2>/dev/null

ros2 run ch6 drone_lifecycle2 &
NODE=$!
sleep 2

ros2 lifecycle get /drone_lifecycle                                 # expect: unconfigured

# v2: configure is expected to return FAILURE (see table) — record and continue
ros2 lifecycle set /drone_lifecycle configure
#   expected: returns FAILURE (publisher-count check on None publishers)
#   The node returns to 'unconfigured' (FAILURE in on_configure rolls back).

# If configure does succeed (e.g. a future fix lands), continue with the same flow as Part 1.
# Otherwise record this as the v2-specific signature and note it as a regression candidate against
# /ros2_src/ch6/ch6/drone_lifecycle2.py:62-65 (publishers referenced before creation).

ros2 lifecycle set /drone_lifecycle activate    # only reachable from inactive; from unconfigured this is invalid
#   expected (from unconfigured): the lifecycle service rejects the transition
#   ("transition 'activate' not registered for state 'unconfigured'")

# Drive the rest of the flow manually only if configure succeeded.
ros2 lifecycle set /drone_lifecycle deactivate   # if reachable, expected FAILURE because isFlying is never set
ros2 lifecycle set /drone_lifecycle cleanup
ros2 lifecycle set /drone_lifecycle shutdown
ros2 lifecycle get /drone_lifecycle              # expect: finalized

kill $NODE 2>/dev/null; wait $NODE 2>/dev/null
```

If v2 has been fixed (the publisher-count check moved to `on_activate` post-creation), re-run Part 1's full positive flow against v2 and tick the equivalent criteria. Either way, the v2 record captures **whether the state machine itself behaves**, distinct from the asymmetric `isFlying` guard around deactivate.

---

## Part 3 — Behavioral side-effects (the unique signal lifecycle gives)

State-word verification (Parts 1–2) covers "the FSM works"; this section covers "the node actually responds when active and stops responding when not". The probe topic for `drone_lifecycle` v1 is the **subscriber** side: publishing on `/takeoff` (v1's global subscriber) makes `cb_takeoff_cmd → takeOff()` republish an Empty on `{ns}/takeoff`. Record both sides offline so the assertion is re-runnable.

**Setup (assume sim and v1 node from Part 1 are running, state = active):**

```bash
ros2 lifecycle set /drone_lifecycle deactivate   # if state was active from Part 1
ros2 lifecycle set /drone_lifecycle cleanup
ros2 lifecycle set /drone_lifecycle configure
ros2 lifecycle set /drone_lifecycle activate     # known-good baseline
```

### Active path — input triggers output

```bash
# Record the publisher-side output topic while injecting on the subscriber-side input topic.
# v1 cb_cmd_vel republishes Twist via pubCmd onto {ns}/cmd_vel, and cb_takeoff_cmd
# republishes Empty via pubTakeOff onto {ns}/takeoff.

ros2 bag record -s mcap -o /tmp/bl5b_active_$$ /simple_drone/cmd_vel /simple_drone/takeoff &
REC=$!
sleep 1

# Probe A: cmd_vel republish (Twist on /cmd_vel → Twist on /simple_drone/cmd_vel)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Probe B: takeoff republish (Empty on /takeoff → Empty on /simple_drone/takeoff)
ros2 topic pub --once /takeoff std_msgs/msg/Empty "{}"

sleep 2
kill $REC; wait $REC 2>/dev/null

python plans/scripts/assert_message_count.py /tmp/bl5b_active_$$ \
  --topic /simple_drone/cmd_vel  --min-count 1
python plans/scripts/assert_message_count.py /tmp/bl5b_active_$$ \
  --topic /simple_drone/takeoff  --min-count 1
```

### Inactive path — same input, no output

```bash
ros2 lifecycle set /drone_lifecycle deactivate     # state → inactive

ros2 bag record -s mcap -o /tmp/bl5b_inactive_$$ /simple_drone/cmd_vel /simple_drone/takeoff &
REC=$!
sleep 1

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /takeoff std_msgs/msg/Empty "{}"

sleep 2
kill $REC; wait $REC 2>/dev/null

# Note: v1's on_deactivate does NOT destroy the subscribers; the lifecycle managed publishers
# are simply silent because the managed-entity lifecycle gates publish() in the inactive state.
# The bag may contain a single hover-zero Twist emitted by on_deactivate (hover()) right before
# the topic recorder started. Pass criterion is therefore: AT MOST one message in the inactive
# bag, and that message's linear.x is 0.0 (the hover frame), not 0.1 (the probe).
python plans/scripts/assert_message_count.py /tmp/bl5b_inactive_$$ \
  --topic /simple_drone/cmd_vel  --max-count 1
python plans/scripts/assert_message_count.py /tmp/bl5b_inactive_$$ \
  --topic /simple_drone/takeoff  --max-count 0
```

### Re-activate path

```bash
ros2 lifecycle set /drone_lifecycle activate

ros2 bag record -s mcap -o /tmp/bl5b_reactive_$$ /simple_drone/cmd_vel /simple_drone/takeoff &
REC=$!
sleep 1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /takeoff std_msgs/msg/Empty "{}"
sleep 2
kill $REC; wait $REC 2>/dev/null

python plans/scripts/assert_message_count.py /tmp/bl5b_reactive_$$ \
  --topic /simple_drone/cmd_vel  --min-count 1
python plans/scripts/assert_message_count.py /tmp/bl5b_reactive_$$ \
  --topic /simple_drone/takeoff  --min-count 1
```

This is the bl5b acceptance signal lifecycle is meant to give — **without** an active/inactive output-rate gap, "the FSM works" is just state words.

---

## Acceptance criteria

State transitions (drone_lifecycle v1, [Lifecycle template in conventions](behavior-test-conventions.md#lifecycle)):

- [ ] Initial state is `unconfigured`.
- [ ] `lifecycle set configure` returns SUCCESS; `get` reports `inactive`.
- [ ] `lifecycle set activate` returns SUCCESS; `get` reports `active`.
- [ ] `lifecycle set deactivate` returns SUCCESS; `get` reports `inactive`.
- [ ] `lifecycle set cleanup` returns SUCCESS; `get` reports `unconfigured`.
- [ ] Re-`configure` + `activate` round-trip both return SUCCESS.
- [ ] `lifecycle set shutdown` returns SUCCESS; `get` reports `finalized`.

Behavioral side-effects (drone_lifecycle v1, Part 3):

- [ ] Active: input publish on `/cmd_vel` → ≥1 message on `/simple_drone/cmd_vel` within 2 s.
- [ ] Active: input publish on `/takeoff` → ≥1 message on `/simple_drone/takeoff` within 2 s.
- [ ] Inactive: same `/cmd_vel` publish → 0 non-hover messages on `/simple_drone/cmd_vel` (single zero-Twist hover frame from `on_deactivate` is permitted).
- [ ] Inactive: same `/takeoff` publish → 0 messages on `/simple_drone/takeoff`.
- [ ] Re-activate: input publish on both topics → ≥1 message on each output within 2 s.

drone_lifecycle2 (Part 2):

- [ ] v2-specific `on_configure` behavior recorded: either SUCCESS (run all v1 criteria against v2) or FAILURE with the `pubTakeOff.get_subscription_count() on None` regression noted in `plans/errors/<short>.md`.
- [ ] If v2 configure succeeds: same state-transition and behavioral checklist as v1 passes (substituting hard-coded `/simple_drone/...` topics on both sides, since v2 owns no `droneNS` param).
- [ ] v2 `on_shutdown` returns SUCCESS even though it does not `land()` — confirm via the SUCCESS return code, not via downstream actuator effect.

## Known caveats

- **sjtu_drone is vendored in `~/sim_ws`, not this repo.** Verify with `ros2 pkg prefix sjtu_drone_bringup`. Any failure inside the bringup launch is upstream — file against `rclpy-from-zero-to-hero-container/`, not against ch6 (cf. [smoke-test-b4-ch5-ch6.md](smoke-test-b4-ch5-ch6.md) Known caveats, third bullet).
- **Lifecycle CLI syntax must be verified on Jazzy.** `ros2 lifecycle set <node> <transition>` is the documented Jazzy form (same as Iron/Humble). If `ros2 lifecycle list <node>` does not include `configure`/`activate`/`deactivate`/`cleanup`/`shutdown`, the underlying `lifecycle_msgs` rebuild is incomplete — re-source the workspace and re-run.
- **Both executables register `drone_lifecycle` as the default node name** ([drone_lifecycle.py:12](../ros2_src/ch6/ch6/drone_lifecycle.py#L12), [drone_lifecycle2.py:19](../ros2_src/ch6/ch6/drone_lifecycle2.py#L19)). Do not run them simultaneously — kill v1 before launching v2 (and vice versa) or the lifecycle CLI will target a non-deterministic instance.
- **v1 `on_activate` checks `pub.get_subscription_count() > 0`** for every managed publisher ([drone_lifecycle.py:72-84](../ros2_src/ch6/ch6/drone_lifecycle.py#L72-L84)). If the gz parameter_bridge has not advertised subscriptions on `/simple_drone/{takeoff,land,cmd_vel}` by the time we issue `activate`, the transition returns ERROR. Pre-flight's `ros2 topic info` checks gate this.
- **v2 `on_configure` references publishers before they are created** ([drone_lifecycle2.py:62-65](../ros2_src/ch6/ch6/drone_lifecycle2.py#L62-L65)) and v2 `on_deactivate` refuses when `not isFlying` ([L98-L100](../ros2_src/ch6/ch6/drone_lifecycle2.py#L98-L100)) but `isFlying` is never set inside lifecycle callbacks. Both are expected v2 signatures — record them, raise as `plans/errors/<short>.md` only if scope is broadened beyond verification.
- **v1 subscribers are not lifecycle-managed.** `_init_subscribers` ([L37-L40](../ros2_src/ch6/ch6/drone_lifecycle.py#L37-L40)) uses `self.create_subscription`, not `self.create_lifecycle_subscription`. Subscribers fire even in `inactive`; the active/inactive gap shows up only on the **publisher** side because publishers come from the lifecycle base class via `create_publisher`. The Part 3 probe records the publisher-side topics for exactly this reason.
- **Headless caveat — `xterm -e teleop` from [simulation.launch.py:15-20](../ros2_src/ch6/launch/simulation.launch.py#L15-L20) fails without DRI.** Bringing up sjtu_drone via `sjtu_drone_bringup/launch/sjtu_drone_gazebo.launch.py` directly (as in pre-flight) skips that Node and is the recommended path for bl5b.

## On pass

Tick all checkboxes in this file; copy [executions/behavior-test-template.md](executions/behavior-test-template.md) to `executions/behavior-test-bl5b-ch6-lifecycle-execution.md` and record the run. Then proceed to [behavior-test-bl6-project-capstone.md](behavior-test-bl6-project-capstone.md) (project package — Nav2, perception, controller).
