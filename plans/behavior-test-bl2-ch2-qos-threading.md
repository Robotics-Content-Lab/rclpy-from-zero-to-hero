# Behavior Test — Phase BL2 (ch2 QoS + threading, hw2 marker counts)

**Phase:** BL2 (behavior layer, builds on B3 launch-level smoke test)
**Packages under test:**
- [ros2_src/ch2](../ros2_src/ch2) — 11 nodes from [setup.py:24-37](../ros2_src/ch2/setup.py#L24-L37): blocking, qos_publisher1, qos_publisher, qos_subscriber1, qos_subscriber2, qos_subscriber, mutually_exclusive, concurrent, long_running_operation, manual_spin, async_spin.
- [ros2_src/hw2_pkg](../ros2_src/hw2_pkg) — `sensor_proc` ([setup.py:26](../ros2_src/hw2_pkg/setup.py#L26)) plus RViz marker config.

Verification scope: **business logic of QoS rejection, callback-group concurrency, executor patterns, and DBSCAN-driven scan clustering**. Launch graph is already covered by [smoke-test-b3-ch2-ch3.md](smoke-test-b3-ch2-ch3.md); we assume B3 is green.

Shared templates (rosbag pattern, headless Gazebo snippet, message-count helper, log-overlap detector) live in [conventions](behavior-test-conventions.md) — reference, do not duplicate.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch2 hw2_pkg
source install/setup.bash

# Canary — confirms Py 3.12 / NumPy 2 / sklearn imports cleanly
python -c "import ch2.qos_publisher1, ch2.qos_subscriber1, ch2.qos_subscriber2"
python -c "import ch2.mutually_exclusive, ch2.concurrent, ch2.long_running_operation"
python -c "import ch2.manual_spin"
python -c "import hw2_pkg.sensor_proc; print('hw2 canary ok')"
#   sensor_proc imports sklearn.cluster.DBSCAN — exposes any numpy-2 / sklearn ABI break.

# hw2 depends on TurtleBot3 sim (see hw2.launch.py:28-35) — confirm available
ros2 pkg prefix turtlebot3_gazebo                   # must succeed
export TURTLEBOT3_MODEL=burger
```

Headless-first reminder (per [conventions § Headless](behavior-test-conventions.md#headless-gazebo)): all GUIs are optional. RViz from `hw2.launch.py:45-50` is allowed to fail without blocking the test — we drive verification off topic content.

---

## Part 1 — QoS profile mismatch & match

Publishers/subscribers on topic `topic` (`example_interfaces/msg/Float32`):

| Node | QoS reliability | Source |
|---|---|---|
| `qos_publisher1` / `qos_publisher` | `BEST_EFFORT` | [qos_publisher1.py:12](../ros2_src/ch2/ch2/qos_publisher1.py#L12), [qos_publisher.py:13](../ros2_src/ch2/ch2/qos_publisher.py#L13) |
| `qos_subscriber1` | `RELIABLE`     | [qos_subscriber1.py:12](../ros2_src/ch2/ch2/qos_subscriber1.py#L12) |
| `qos_subscriber2` | `BEST_EFFORT`  | [qos_subscriber2.py:11](../ros2_src/ch2/ch2/qos_subscriber2.py#L11) |
| `qos_subscriber`  | configurable via `reliability_policy` param | [qos_subscriber.py:12-22](../ros2_src/ch2/ch2/qos_subscriber.py#L12-L22) |

The launch file [qos.launch.py:9-13](../ros2_src/ch2/launch/qos.launch.py#L9-L13) drives the parametrized pair; default `best_effort`.

### Verification — mismatch path (publisher BEST_EFFORT × subscriber RELIABLE)

```bash
# Terminal A
ros2 run ch2 qos_publisher1
# Terminal B
ros2 run ch2 qos_subscriber1
# Terminal C
ros2 topic info /topic --verbose
#   expect: Publisher with reliability=BEST_EFFORT, Subscriber with reliability=RELIABLE
#   In Jazzy the daemon log emits an incompatible-QoS warning; subscriber's "Heard:" line never appears.
```

Use the message-count helper from [conventions § Message counting](behavior-test-conventions.md#message-counting) to capture 10 s of subscriber callbacks:

```bash
ros2 topic echo --once --timeout 10 /topic > /tmp/bl2_mismatch.log || echo "no data (expected)"
```

### Verification — matched path (publisher BEST_EFFORT × subscriber BEST_EFFORT)

```bash
ros2 launch ch2 qos.launch.py reliability_policy:=best_effort
ros2 topic hz /topic --window 10
#   Publisher ticks at 1.0 Hz (qos_publisher1.py:14); over 10 s expect ≥9 messages received.
```

`ros2 launch ch2 qos.launch.py reliability_policy:=reliable` re-creates the mismatch case (publisher remains BEST_EFFORT). Record observed delivery rate either way.

### Acceptance criteria (Part 1)

- [ ] `ros2 topic info /topic --verbose` shows publisher `Reliability: BEST_EFFORT` and subscriber `Reliability: RELIABLE` in the mismatch run.
- [ ] Mismatch subscriber receives **zero** messages over a 10 s window.
- [ ] Matched (BEST_EFFORT/BEST_EFFORT) subscriber receives ≥9 messages over 10 s (≥90 % of the 1 Hz publish rate).
- [ ] No `rclpy` traceback during either run.

---

## Part 2 — Callback groups (Reentrant vs MutuallyExclusive)

`mutually_exclusive` ([mutually_exclusive.py:21](../ros2_src/ch2/ch2/mutually_exclusive.py#L21)) registers four callbacks (two subscriptions + two timers) on a single `MutuallyExclusiveCallbackGroup`:
- `joint_state_callback` on `/joint_states`
- `imu_callback` on `/imu`
- `command_callback` on a 1 Hz timer ([line 49-53](../ros2_src/ch2/ch2/mutually_exclusive.py#L49-L53))
- `update_distance` on a 1 Hz timer ([line 56-60](../ros2_src/ch2/ch2/mutually_exclusive.py#L56-L60))

Each callback emits a log line tagged `[self.callback_cnt=N]` ([lines 71, 87, 97, 107](../ros2_src/ch2/ch2/mutually_exclusive.py#L71)).

`concurrent` ([concurrent.py:14-15](../ros2_src/ch2/ch2/concurrent.py#L14-L15)) uses two `ReentrantCallbackGroup` instances, a `MultiThreadedExecutor` ([line 55](../ros2_src/ch2/ch2/concurrent.py#L55)), and inserts a 1 s `self.rate.sleep()` inside `sensor_callback` ([line 43](../ros2_src/ch2/ch2/concurrent.py#L43)) to force overlap.

### Verification — MutuallyExclusive: zero overlap

```bash
# Terminal A: start TB3 sim (provides /joint_states, /imu)
ros2 launch ch2 simulation.launch.py world_file:=turtlebot3_world
# Terminal B: run the node, capture log
ros2 run ch2 mutually_exclusive 2>&1 | tee /tmp/bl2_mex.log
```

Use the log-overlap detector from [conventions § Log-overlap](behavior-test-conventions.md#log-overlap-detector) to scan `bl2_mex.log` for any `[callback_cnt=N]` interval whose start precedes the prior interval's completion. Acceptance: zero overlapping intervals across ≥10 callback invocations.

### Verification — Reentrant: observable overlap

```bash
# Terminal C: drive concurrent_processor with two scans + two commands within a 1 s window
ros2 run ch2 concurrent 2>&1 | tee /tmp/bl2_concurrent.log

# Terminal D: publish two scans back-to-back, then a command
ros2 topic pub --once /scan sensor_msgs/msg/LaserScan "{header: {frame_id: 'base_scan'}, ranges: [1.0, 2.0]}"
ros2 topic pub --once /scan sensor_msgs/msg/LaserScan "{header: {frame_id: 'base_scan'}, ranges: [3.0, 4.0]}"
ros2 topic pub --once /user_command example_interfaces/msg/String "{data: 'aggressive'}"
```

Acceptance: log shows the `command_callback` line **interleaved between** two `sensor_callback` entries, despite each sensor callback sleeping 1 s. Equivalent simpler check — time the wall-clock window from the first `Processing sensor data` to the matching `Processed msg` line for two back-to-back scans: with a reentrant group + multi-threaded executor the total is ≈ 1.0 s; under a mutually-exclusive group it would be ≈ 2.0 s.

### Acceptance criteria (Part 2)

- [ ] `mutually_exclusive` log shows **zero** overlapping `[callback_cnt=N]` intervals over ≥10 callbacks.
- [ ] `concurrent` log shows the user-command callback firing **between** the start and end of a still-running sensor callback (overlap > 0).
- [ ] Two back-to-back `/scan` publishes complete `Processed msg` for both within ~1 s wall-clock (reentrant), not ~2 s (would indicate serial execution).
- [ ] No deadlock — `ros2 run` exits cleanly on Ctrl-C in both runs.

---

## Part 3 — Async spin and manual spin

| Node | Pattern | Source |
|---|---|---|
| `long_running_operation` | `MultiThreadedExecutor` + `ReentrantCallbackGroup`; blocking 10 s `time.sleep` inside `__init__` while a 1 Hz timer publishes `chatter` | [long_running_operation.py:24-33](../ros2_src/ch2/ch2/long_running_operation.py#L24-L33) |
| `manual_spin` | Same body as `long_running_operation` but with explicit `rclpy.spin_once(self, timeout_sec=0.1)` inside the blocking loop | [manual_spin.py:32](../ros2_src/ch2/ch2/manual_spin.py#L32) |
| `async_spin` | `ThreadPoolExecutor(max_workers=1)`; a 1 Hz timer submits a 5 s task and uses `add_done_callback` for the result | [async.py:13-19](../ros2_src/ch2/ch2/async.py#L13-L19) |

> **Caveat — entry-point mapping**: [setup.py:34](../ros2_src/ch2/setup.py#L34) registers `async_spin = ch2.async_spin:main`, but the source file is [ch2/async.py](../ros2_src/ch2/ch2/async.py). If `ros2 run ch2 async_spin` fails with `ModuleNotFoundError`, fall back to `python -m ch2.async`. Record the result either way; do **not** fix as part of bl2 — log to `plans/errors/`.

### Verification — long_running_operation does not stall the timer

```bash
# Terminal A
ros2 run ch2 long_running_operation 2>&1 | tee /tmp/bl2_lro.log &
LRO_PID=$!

# Terminal B: while the 10 s computation runs, /chatter must keep ticking
ros2 topic hz /chatter --window 5
#   expect: ~1 Hz throughout, including the first 10 s.
```

Acceptance: `/chatter` rate stays ≥0.8 Hz during the long-running computation; the node also auto-shuts down after `computation_complete` ([long_running_operation.py:44-45](../ros2_src/ch2/ch2/long_running_operation.py#L44-L45)).

### Verification — manual_spin and async_spin

`manual_spin.py` is the textbook variant that interleaves `rclpy.spin_once` inside the blocking loop. Run it the same way as `long_running_operation` and confirm `/chatter` hz stays near 1 Hz throughout.

`async_spin` schedules a 5 s task per timer tick via `ThreadPoolExecutor`:

```bash
ros2 run ch2 async_spin 2>&1 | tee /tmp/bl2_async.log    # see caveat above
sleep 10
# Inspect log: expect alternating "Starting asynchronous task" and "Asynchronous task completed"
grep -c "Starting asynchronous task" /tmp/bl2_async.log   # ≥9 in 10 s
grep -c "Asynchronous task completed" /tmp/bl2_async.log  # ≥1 (first 5 s task finishes inside the window)
```

Acceptance: no deadlock over a 10 s window; both `Starting` and `completed` lines present.

### Acceptance criteria (Part 3)

- [ ] `long_running_operation`: `/chatter` rate stays ≥0.8 Hz during the 10 s blocking computation.
- [ ] `long_running_operation` exits cleanly once `computation_complete` is set.
- [ ] `manual_spin`: same `/chatter` rate stability as `long_running_operation` (verifies `rclpy.spin_once` integration).
- [ ] `async_spin`: both `Starting asynchronous task` and `Asynchronous task completed` log lines observed within a 10 s window (or, if entry-point caveat triggers, the same is true under `python -m ch2.async`).
- [ ] None of the three nodes deadlock during the test window.

---

## Part 4 — hw2_pkg sensor processing → markers

`sensor_proc` ([sensor_proc.py:93-145](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L93-L145)) subscribes to:
- `/scan` (`sensor_msgs/msg/LaserScan`, BEST_EFFORT) — [line 121-127](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L121-L127)
- `/odom` (`nav_msgs/msg/Odometry`, RELIABLE+TRANSIENT_LOCAL) — [line 129-135](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L129-L135)

It publishes:
- `/clustered_scan` (`sensor_msgs/msg/LaserScan`, BEST_EFFORT) at 30 Hz — [line 138-145](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L138-L145)

**Marker-vs-scan note**: the prompt asks for a `MarkerArray`, but `sensor_proc` actually publishes a *re-clustered LaserScan* on `/clustered_scan` (no `visualization_msgs` topic). RViz renders that scan directly. We therefore treat "marker count" as **cluster count = number of finite ranges in the published scan** (one finite value per centroid; see [sensor_proc.py:33-50](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L33-L50)). Skip RViz visual inspection — already covered by B3.

Clustering: DBSCAN on `(x, y)` ranges ([line 75-85](../ros2_src/hw2_pkg/hw2_pkg/sensor_proc.py#L75-L85)). Default args [hw2.launch.py:16-25](../ros2_src/hw2_pkg/launch/hw2.launch.py#L16-L25): `min_samples=20`, `dist_thresh=0.1`.

### Verification

```bash
# Terminal A: full launch (includes turtlebot3_world + sensor_proc + RViz)
ros2 launch hw2_pkg hw2.launch.py
#   RViz may fail to open in headless mode — acceptable per the headless-first policy.

# Terminal B: confirm subscriptions wired
ros2 topic info /scan --verbose       # publisher (gz bridge), subscriber (sensor_proc)
ros2 topic info /clustered_scan --verbose
#   sensor_proc reliability=BEST_EFFORT — matches the inbound /scan; should not be marked incompatible.

# Capture one /clustered_scan
ros2 topic echo --once /clustered_scan > /tmp/bl2_clusters.yaml

# Count finite ranges (= cluster centroids; everything else is `inf` per sensor_proc.py:33)
python - <<'PY'
import yaml, math
with open('/tmp/bl2_clusters.yaml') as f:
    msg = yaml.safe_load(f)
ranges = msg['ranges']
finite = [r for r in ranges if math.isfinite(r)]
print(f'finite ranges (cluster count): {len(finite)}')
print(f'total ranges: {len(ranges)}')
assert 1 <= len(finite), 'no clusters produced'
assert len(finite) < len(ranges), 'clustering did not collapse any rays'
PY

# Also confirm publish rate is alive
ros2 topic hz /clustered_scan --window 5    # expect ~30 Hz (timer at 1/30 s, sensor_proc.py:145)
```

### Acceptance criteria (Part 4)

- [ ] Canary import of `hw2_pkg.sensor_proc` succeeds (covers numpy-2 / sklearn ABI).
- [ ] `ros2 launch hw2_pkg hw2.launch.py` starts `sensor_proc` without traceback (RViz failure tolerated).
- [ ] `/clustered_scan` publishes at ≥20 Hz (~30 Hz target, allow scheduling slack).
- [ ] A single `/clustered_scan` message contains **≥1** finite range (clusters detected) and **< total ranges** (clustering actually collapsed rays).
- [ ] Cluster count is bounded — no `inf` flood: `finite < len(ranges)` and `finite < 100` (sanity upper bound; TurtleBot3 world rarely shows more than a few dozen DBSCAN cores at `min_samples=20`).
- [ ] Parameter override works: relaunch with `min_samples:=5 dist_thresh:=0.05` and observe higher cluster count than the default-args run.

---

## Known caveats

- **`async_spin` entry-point mismatch**: [setup.py:34](../ros2_src/ch2/setup.py#L34) references `ch2.async_spin`, but only [ch2/async.py](../ros2_src/ch2/ch2/async.py) exists. `ros2 run ch2 async_spin` may fail with `ModuleNotFoundError`. Document the observed behavior; do not patch under bl2 — file under `plans/errors/`.
- **`manual_spin` vs `long_running_operation`**: the two files are near-duplicates (manual_spin adds one `rclpy.spin_once` line). Both must verify cleanly; if a regression hits only one, it's a real defect, not a duplicate.
- **TB3 dependency in hw2 + ch2 mutually_exclusive**: both pre-require `turtlebot3_gazebo` ([hw2.launch.py:28-35](../ros2_src/hw2_pkg/launch/hw2.launch.py#L28-L35), [ch2/simulation.launch.py](../ros2_src/ch2/launch/simulation.launch.py)). If TB3 Jazzy renamed `turtlebot3_world.launch.py`, the launches fail before any business logic runs — same caveat as B3.
- **`use_sim_time` propagation**: `mutually_exclusive` self-sets `use_sim_time=True` ([line 11-12](../ros2_src/ch2/ch2/mutually_exclusive.py#L11-L12)). It expects `/clock` to be live; without TB3 sim running first, timer callbacks may not fire. Always start the sim **before** the node.
- **RViz GUI is not the verification surface**: per [conventions § Headless](behavior-test-conventions.md#headless-gazebo), do not block on RViz opening; topic content drives Part 4 acceptance.

## On pass

Update the BL2 row in [i-m-working-on-porting-glistening-gray.md](../../../.claude/plans/i-m-working-on-porting-glistening-gray.md) and proceed to **[behavior-test-bl3-ch3-control-odometry.md](behavior-test-bl3-ch3-control-odometry.md)** (closed-loop control + odometry traverse, depends on ch2 callback patterns working).
