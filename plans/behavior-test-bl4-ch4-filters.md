# Behavior Test — Phase BL4 (ch4 KF/EKF filters + hw4 EKF holonomic; rosbag-driven offline)

**Status:** ⏳ Pending execution — prerequisites met as of 2026-05-24.

**Prerequisites now met:**
- `transforms3d ≥ 0.4.2` pip override in container Dockerfile (NumPy 2 safe)
- `ros_gz_sim create` used for all spawn operations (no service bridge needed)
- B1–B7 code migration committed on `jazzy-migration` branch

**Phase note:** ⭐ Rosbag-driven and fully offline — no Gazebo required. Bags already in `ros2_src/ch4/bag/`. Good candidate to run early and independently of Gazebo-dependent phases.

**Still required:** Execute all verification steps inside `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia`. Record results in `plans/executions/behavior-test-bl4-execution.md`.

---

**Phase:** BL4 (filters)
**Packages under test:**
- [ros2_src/ch4](../ros2_src/ch4) — 5 nodes (`odometry_comparison`, `kf`, `kf2`, `ekf`, `ekf2`); 4 launches (`kf.launch.py`, `kf2.launch.py`, `ekf.launch.py`, `ekf2.launch.py`).
- [ros2_src/hw4_pkg](../ros2_src/hw4_pkg) — 2 nodes (`ekf_holonomic`, `timer`); 1 launch (`hw4.launch.py`).

**Verification surface:** rosbag-driven, fully offline. Replay `ch4/bag/{circle,square,straight}/`; record the filter's output Odometry; assert (a) RMSE vs ground-truth `/odom` within bound, (b) covariance trace monotone decreasing for the first 5 update steps, (c) **no `NaN`/`Inf`** anywhere over the full bag. Conventions per the **Filter** template in [behavior-test-conventions.md](behavior-test-conventions.md#filter).

**Out of scope:** GUI verification via `odometry_comparison`'s matplotlib pane (headless container); benchmarking; tuning Q/R away from source defaults.

## Pre-flight

Inside the Jazzy container:

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch4 hw4_pkg utils
source install/setup.bash
```

### Canary imports (Python 3.12 / NumPy 2 — per [conventions](behavior-test-conventions.md#python-312--numpy-2-import-canary))

```bash
python -c "import ch4.kf; import ch4.kf2; import ch4.ekf; import ch4.ekf2; import ch4.odometry_comparison; print('ch4 canary ok')"
python -c "import hw4_pkg.ekf_holonomic_drive; import hw4_pkg.kf_2d; import hw4_pkg.timer; print('hw4_pkg canary ok')"
python -c "import utils.utils.math; print('utils.math canary ok')"
```

`hw4_pkg.ekf_holonomic_drive` imports `tf_transformations` directly ([ekf_holonomic_drive.py:10](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L10)); `ch4.ekf` / `ch4.ekf2` import via `utils.utils.math`. If either canary fails with `AttributeError: module 'numpy' has no attribute 'maximum_sctype'`, stop — see [plans/errors/fixed/tf_transformations-numpy2-incompatibility.md](errors/fixed/tf_transformations-numpy2-incompatibility.md). A passing canary means the upstream `transforms3d` apt fix (≥ 0.4.2) is in place and the rest of the phase can proceed.

### Bag inventory — confirm payloads present

```bash
ls /workspaces/rclpy-from-zero-to-hero/ros2_src/ch4/bag/{circle,square,straight}/
# expect each to contain: metadata.yaml + one *.mcap file
```

Observed (from repo inspection):

| Bag | mcap | duration | topics (msg counts) |
|---|---|---|---|
| `circle/` | `circle.mcap` (10228 msgs, 35.45 s) | 35 s | `/cmd_vel` 366, `/joint_states` 925, `/imu` 5884, `/odom` 925, `/tf` 1519, `/clock` 336, `/robot_description` 1 |
| `square/` | `square.mcap` (4869 msgs, 24.65 s) | 25 s | `/robotino/cmd_vel` 1418, `/robotino/imu/out` 2465, `/robotino/odom` 493, `/tf` 493 |
| `straight/` | `straight.mcap` (6871 msgs, 25.18 s) | 25 s | `/cmd_vel` 232, `/joint_states` 606, `/imu` 3923, `/odom` 606, `/scan` 103, `/tf` 979, `/clock` 236 |

Note the topic schism — `circle/` and `straight/` carry **TurtleBot3-style** topics (`/cmd_vel`, `/joint_states`, `/imu`, `/odom`); `square/` carries **Robotino-style** namespaced topics (`/robotino/*`). This dictates which filter runs against which bag (see Parts below).

Sanity check one bag at runtime:

```bash
ros2 bag info /workspaces/rclpy-from-zero-to-hero/ros2_src/ch4/bag/circle/
# Storage id: mcap; Duration: ~35s; ten topics including /odom, /joint_states, /imu, /cmd_vel
```

### Per-filter output topic & state dimensions (cite source)

| Node | Output topic | Msg type | State dim (P shape) | Source |
|---|---|---|---|---|
| `ch4 kf` | `odom_kf` | `nav_msgs/Odometry` | 1 (`[x]`) — but published as 6×6 cov with `P[0,0]` placed at index 0 and a `0.1` placeholder at index 7 | [kf.py:13](../ros2_src/ch4/ch4/kf.py#L13), [kf.py:94](../ros2_src/ch4/ch4/kf.py#L94), [kf.py:183-186](../ros2_src/ch4/ch4/kf.py#L183-L186) |
| `ch4 kf2` | `odom_kf` | `nav_msgs/Odometry` | 1 (`[x]`) with outlier rejection at Mahalanobis > 8 | [kf2.py:14](../ros2_src/ch4/ch4/kf2.py#L14), [kf2.py:112](../ros2_src/ch4/ch4/kf2.py#L112), [kf2.py:201-204](../ros2_src/ch4/ch4/kf2.py#L201-L204) |
| `ch4 ekf` | `odom_ekf` | `nav_msgs/Odometry` | 6 (`[x, y, θ, ẋ, ẏ, θ̇]`); pose-cov is the 6×6 P flattened | [ekf.py:13](../ros2_src/ch4/ch4/ekf.py#L13), [ekf.py:157](../ros2_src/ch4/ch4/ekf.py#L157), [ekf.py:241-243](../ros2_src/ch4/ch4/ekf.py#L241-L243) |
| `ch4 ekf2` | `odom_ekf` | `nav_msgs/Odometry` | 6, with outlier rejection (Mahalanobis > 16) + a hard `|a_x|>0.30 m/s²` cap | [ekf2.py:14](../ros2_src/ch4/ch4/ekf2.py#L14), [ekf2.py:204](../ros2_src/ch4/ch4/ekf2.py#L204), [ekf2.py:79-95](../ros2_src/ch4/ch4/ekf2.py#L79-L95) |
| `hw4_pkg ekf_holonomic` | `odom_ekf` | `nav_msgs/Odometry` | 6 (`[x, y, θ, vₓ, vᵧ, ω]`); subscribes to `/robotino/cmd_vel` + `/robotino/imu/out` (hard-coded, see [ekf_holonomic_drive.py:261-268](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L261-L268)) | [ekf_holonomic_drive.py:14](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L14), [ekf_holonomic_drive.py:271](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L271) |

### Approach choice — package launches vs. manual capture

Every `ch4` launch file wraps `ros2 bag play … --start-paused --rate 1.0` inside `xterm -hold`, e.g. [ekf.launch.py:17-21](../ros2_src/ch4/launch/ekf.launch.py#L17-L21). Two issues for headless CI:

1. `xterm` requires a display.
2. `--start-paused` blocks until a key-press in that terminal.
3. They also launch `odometry_comparison`, which calls `plt.ion()` ([odometry_comparison.py:40](../ros2_src/ch4/ch4/odometry_comparison.py#L40)) and `plt.pause()` — also requires a display.

→ **Recommended:** use the **manual capture-then-assert** sequence below (a small adaptation of the package launch sans xterm/odometry_comparison). The launch files are still useful for interactive demos; do not modify them.

## Verification pattern — capture-then-assert

The canonical filter recipe per [conventions §Rosbag capture-then-assert pattern](behavior-test-conventions.md#rosbag-capture-then-assert-pattern). Concrete instantiation for ch4 / hw4:

```bash
OUT=/tmp/bl4_<filter>_<bag>_$$
BAG=/workspaces/rclpy-from-zero-to-hero/ros2_src/ch4/bag/<bag>/

# 1. Filter node — keep PID for later kill
ros2 run <pkg> <node> --ros-args -p use_sim_time:=true &
NODE_PID=$!

# 2. Record both estimate and ground truth on the *same* mcap
ros2 bag record -s mcap -o "$OUT" <estimate-topic> <gt-topic> &
REC_PID=$!
sleep 1   # let publishers/subscribers wire up before play starts

# 3. Replay input with --clock so the filter sees sim time
ros2 bag play "$BAG" --clock --rate 1.0
sleep 2   # drain in-flight messages

# 4. Stop record + node (NODE first so it stops publishing)
kill "$NODE_PID" "$REC_PID"
wait "$REC_PID" 2>/dev/null

# 5. Assert
python plans/scripts/assert_filter_rmse.py "$OUT" \
  --estimate-topic <estimate-topic> \
  --groundtruth-topic <gt-topic> \
  --max-rmse <m> \
  --check-covariance-monotone-first 5 \
  --reject-nan
```

`assert_filter_rmse.py` is **TO BE WRITTEN** during execution — see helper catalog in [conventions §Helper scripts catalog](behavior-test-conventions.md#helper-scripts-catalog). It must:

1. Open the mcap, decode both topics, time-align by `header.stamp`.
2. Compute per-axis RMSE on the pose-position fields the filter actually populates (x for kf/kf2; x,y,θ for ekf/ekf2/ekf_holonomic).
3. Track `trace(P_xy)` from the flattened `pose.covariance` 6×6, fail if it strictly increases for any of the first `--check-covariance-monotone-first` ticks after the first non-zero estimate.
4. Walk every published message, fail if any field of pose/twist/covariance is `NaN` or `Inf`.
5. Print `PASS / FAIL <metric=value>` and exit 0/1.

---

## Part 1 — Linear Kalman filter (`kf`, `kf2`)

Both are 1-D filters on `x`. Source uses `cmd_vel` linear-x as control and `joint_states` wheel angle as measurement (see [kf.py:81-92](../ros2_src/ch4/ch4/kf.py#L81-L92), [kf2.py:99-110](../ros2_src/ch4/ch4/kf2.py#L99-L110)). The only bag whose **straight-line motion** matches a 1-D filter is `straight/`. `circle/` is included to **exercise the failure mode** (a 1-D linear filter cannot represent the circular ground-truth y-component, so RMSE on x alone is the only meaningful metric — see Known caveats).

### Combos

| Filter | Bag | Estimate topic | GT topic | Max RMSE-x (m) | Notes |
|---|---|---|---|---|---|
| `kf` | `straight/` | `/odom_kf` | `/odom` | 0.50 | Canonical case — 1-D KF on straight bag. |
| `kf` | `circle/` | `/odom_kf` | `/odom` | 1.50 | Stress case; expect inflation (see caveats). |
| `kf2` | `straight/` | `/odom_kf` | `/odom` | 0.50 | Adds outlier rejection (Mahalanobis > 8). |
| `kf2` | `circle/` | `/odom_kf` | `/odom` | 1.50 | Same stress case as `kf`. |

`square/` is **not used here** — it carries `/robotino/*` topics, not `/cmd_vel` or `/joint_states`; the linear KF subscribes to bare topic names and would receive nothing. Document this in the execution record as **N/A**.

### Recipe

```bash
# kf on straight
OUT=/tmp/bl4_kf_straight_$$
ros2 run ch4 kf --ros-args -p use_sim_time:=true & NODE=$!
ros2 bag record -s mcap -o "$OUT" /odom_kf /odom & REC=$!; sleep 1
ros2 bag play /workspaces/rclpy-from-zero-to-hero/ros2_src/ch4/bag/straight/ --clock --rate 1.0
sleep 2; kill "$NODE" "$REC"; wait "$REC" 2>/dev/null

python plans/scripts/assert_filter_rmse.py "$OUT" \
  --estimate-topic /odom_kf --groundtruth-topic /odom \
  --estimate-axes x --max-rmse 0.50 \
  --check-covariance-monotone-first 5 --reject-nan
```

Repeat with `ch4 kf2` and with bag `circle/` (raising `--max-rmse 1.50`).

### Acceptance (Part 1)

- [ ] Each combo's recording contains ≥ 1 message on the estimate topic — proves the filter ran and published.
- [ ] Estimate rate ≥ 0.8 × input rate. Input `/joint_states` is 925/35 ≈ 26 Hz on circle, 606/25 ≈ 24 Hz on straight; the filter timer ticks at 50 Hz (`self.rate = 50`, [kf.py:103](../ros2_src/ch4/ch4/kf.py#L103)) → estimate ≥ 20 Hz is acceptable.
- [ ] RMSE-x bound per table.
- [ ] `trace(P)` monotone decreasing for first 5 updates after the prediction event first fires.
- [ ] Zero `NaN`/`Inf` over the full bag.

---

## Part 2 — Extended Kalman filter (`ekf`, `ekf2`)

6-D state (`[x, y, θ, ẋ, ẏ, θ̇]`); subscribes to `cmd_vel` + `imu` (see [ekf.py:144-155](../ros2_src/ch4/ch4/ekf.py#L144-L155)). Compatible with `circle/` and `straight/` bags (both carry `/cmd_vel` and `/imu`). Not compatible with `square/` (Robotino topics, holonomic motion).

### Combos

| Filter | Bag | Estimate topic | GT topic | Max RMSE-xy (m) | Notes |
|---|---|---|---|---|---|
| `ekf` | `circle/` | `/odom_ekf` | `/odom` | 1.00 | Canonical: nonlinear EKF, circular motion. |
| `ekf` | `straight/` | `/odom_ekf` | `/odom` | 0.50 | Sanity case (low nonlinearity). |
| `ekf2` | `circle/` | `/odom_ekf` | `/odom` | 1.00 | Adds Mahalanobis-16 + `|aₓ|≤0.30` gates. Innovation rejection rate should be 0–20% per [conventions](behavior-test-conventions.md#filter). |
| `ekf2` | `straight/` | `/odom_ekf` | `/odom` | 0.50 | — |

### Recipe

```bash
# ekf on circle
OUT=/tmp/bl4_ekf_circle_$$
ros2 run ch4 ekf --ros-args -p use_sim_time:=true & NODE=$!
ros2 bag record -s mcap -o "$OUT" /odom_ekf /odom & REC=$!; sleep 1
ros2 bag play /workspaces/rclpy-from-zero-to-hero/ros2_src/ch4/bag/circle/ --clock --rate 1.0
sleep 2; kill "$NODE" "$REC"; wait "$REC" 2>/dev/null

python plans/scripts/assert_filter_rmse.py "$OUT" \
  --estimate-topic /odom_ekf --groundtruth-topic /odom \
  --estimate-axes x,y,theta --max-rmse 1.00 \
  --check-covariance-monotone-first 5 --reject-nan
```

### Acceptance (Part 2)

- [ ] Estimate rate ≥ 0.8 × 30 Hz = 24 Hz on each combo (filter timer is 30 Hz, [ekf.py:165](../ros2_src/ch4/ch4/ekf.py#L165)).
- [ ] RMSE-xy bound per table.
- [ ] `trace(P)` monotone decreasing for first 5 updates.
- [ ] θ stays normalized in [-π, π] across all published messages (helper logs out-of-range count; expect 0).
- [ ] Zero `NaN`/`Inf`.
- [ ] (ekf2 only) Innovation-gating rejection fraction in 0–20% on `straight/`; on `circle/` may climb higher — log but do not fail.

---

## Part 3 — `odometry_comparison`

Source: [ros2_src/ch4/ch4/odometry_comparison.py](../ros2_src/ch4/ch4/odometry_comparison.py).

The node subscribes to **three** odometry topics — `odom_ekf`, `odom_kf`, `odom` ([line 15-35](../ros2_src/ch4/ch4/odometry_comparison.py#L15-L35)) — and renders all three trajectories plus KF/EKF covariance ellipses in a single matplotlib figure (`plt.ion()` at [line 40](../ros2_src/ch4/ch4/odometry_comparison.py#L40)). It **does not publish** any topic; its output is the figure.

In a headless container the matplotlib backend cannot draw an interactive window. The verification here is therefore **lifecycle-only**: confirm the node starts, subscribes, ingests messages, and does not crash. The visual comparison itself is **N/A (env)** for this phase.

### Verification

```bash
# With kf running and bag playing as in Part 1, additionally start odometry_comparison:
MPLBACKEND=Agg ros2 run ch4 odometry_comparison &
COMP=$!
sleep 1
# (Run the kf + bag-play sequence here as in Part 1.)

# Check the comparator did not die:
kill -0 $COMP && echo "comparator alive" || echo "comparator crashed"

# Check it received messages on all three subscriptions:
ros2 node info /odom_comparison | grep -E "Subscribers:|odom"
# expect lines listing /odom, /odom_kf, /odom_ekf

kill $COMP
```

`MPLBACKEND=Agg` swaps to a non-interactive matplotlib backend so `plt.ion()` does not require X. Some matplotlib versions still call `plt.pause()` which sleeps but does not draw — acceptable for liveness checking. If `Agg` causes the call to crash, switch to `MPLBACKEND=pdf` or wrap the invocation with `xvfb-run -a` and capture stderr.

### Acceptance (Part 3)

- [ ] `odometry_comparison` reaches steady state (PID is alive ≥ 5 s after launch).
- [ ] `ros2 node info /odom_comparison` lists subscriptions on `/odom`, `/odom_kf`, `/odom_ekf`.
- [ ] No traceback in the node's stderr during the bag-play duration. (Single `Agg`-backend warning about `plt.pause` is acceptable.)
- [ ] Bag-play sequence in Part 1 / Part 2 still meets its own acceptance criteria with the comparator co-running (regression check on inter-node coexistence).

---

## Part 4 — hw4 `ekf_holonomic` on holonomic robot

Source: [ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py).

6-D state, **holonomic** motion model — control is `[vₓ, vᵧ, ω]` from `/robotino/cmd_vel`, measurement is the Robotino IMU on `/robotino/imu/out`. Topics are **hard-coded** in the source ([line 261, 267](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L261-L268)) — no remap required, but only one bag matches: `square/`. The packaged `hw4.launch.py` has the `ekf_holonomic` Node **commented out** ([line 12-20](../ros2_src/hw4_pkg/launch/hw4.launch.py#L12-L20)); we run the node directly with `ros2 run`.

The `square/` bag's ground-truth is `/robotino/odom`, not `/odom`. Remap accordingly when recording.

### Recipe

```bash
OUT=/tmp/bl4_ekf_holonomic_square_$$
BAG=/workspaces/rclpy-from-zero-to-hero/ros2_src/ch4/bag/square/

ros2 run hw4_pkg ekf_holonomic --ros-args -p use_sim_time:=true & NODE=$!
ros2 bag record -s mcap -o "$OUT" /odom_ekf /robotino/odom & REC=$!; sleep 1
ros2 bag play "$BAG" --clock --rate 1.0
sleep 2; kill "$NODE" "$REC"; wait "$REC" 2>/dev/null

python plans/scripts/assert_filter_rmse.py "$OUT" \
  --estimate-topic /odom_ekf --groundtruth-topic /robotino/odom \
  --estimate-axes x,y,theta --max-rmse 1.00 \
  --check-covariance-monotone-first 5 --reject-nan
```

### Acceptance (Part 4)

- [ ] Canary at preflight (`import hw4_pkg.ekf_holonomic_drive`) succeeded — that is the only check that `tf_transformations` works for this node.
- [ ] Estimate rate ≥ 0.8 × 30 Hz = 24 Hz on `/odom_ekf`.
- [ ] RMSE-xy (vs `/robotino/odom`) < 1.00 m over the 25-s square trajectory.
- [ ] hw4 ekf_holonomic state vector is `[x, y, θ, vₓ, vᵧ, ω]` (verify dim via the helper's `--check-state-dim 6` flag): both `pose.twist.linear.y` and `pose.twist.angular.z` are populated in the published Odometry, not just `linear.x`.
- [ ] `trace(P)` monotone decreasing for first 5 updates.
- [ ] Zero `NaN`/`Inf`.

`circle/` and `straight/` are **not used** for hw4 ekf_holonomic — they lack `/robotino/cmd_vel` and `/robotino/imu/out`, the hard-coded subscribers (see [ekf_holonomic_drive.py:261, 267](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L261-L268)).

---

## Common assertions across all filters

These apply to every combo in Parts 1, 2, 4 — `assert_filter_rmse.py` enforces them with a single invocation per recorded mcap:

- **Liveness:** estimate-topic message count > 0 over the bag duration.
- **Rate:** estimate-message rate ≥ 0.8 × declared filter timer rate (50 Hz for kf/kf2, 30 Hz for ekf/ekf2/ekf_holonomic).
- **RMSE:** per-axis position RMSE bound per table; helper additionally reports the raw-IMU-only baseline RMSE (computed by replaying only the IMU integration component, no Kalman update) so the report can quote the ratio per [conventions §Filter](behavior-test-conventions.md#filter).
- **Covariance monotone:** `trace(P_xy)` of the published 6×6 cov decreases tick-over-tick for the first 5 updates after a non-zero estimate first appears. One off-by-one drop allowed only if an outlier was rejected on that tick (the helper cross-references `is_outlier` log lines from the node).
- **No NaN/Inf:** walk every published message, every field of `pose.position`, `pose.orientation`, `twist.linear`, `twist.angular`, and the 36-element `pose.covariance` array. Single non-finite value → FAIL.

For Part 3 (`odometry_comparison`), only liveness applies; trajectory rendering is N/A in the headless env.

## Known caveats

- **Bag-payload topic schism:** `circle/` and `straight/` use `/cmd_vel`/`/joint_states`/`/imu`/`/odom`; `square/` uses `/robotino/cmd_vel`/`/robotino/imu/out`/`/robotino/odom`. ch4 filters (`kf`, `kf2`, `ekf`, `ekf2`) bind to bare names → only `circle/` and `straight/` work for them. hw4 `ekf_holonomic` binds to `/robotino/*` → only `square/` works. Mixing combos yields zero estimates (the filter never receives a `cmd_vel` and the predict step never fires) — flag as N/A in the execution record rather than letting RMSE explode.
- **Linear KF on circular motion is a stress test, not a sanity check.** The 1-D linear KF (state `[x]` only) has no y or θ representation, so on `circle/` the ground-truth position diverges along y while the filter only tracks x — RMSE-x stays bounded but is **not** a measure of "is the filter correct". The wide `--max-rmse 1.50` bound for kf/kf2 + circle reflects this; treat as a smoke verification that the filter doesn't crash, not a quality metric. The book intentionally uses this as a pedagogical example of "the linear KF is inadequate for nonlinear systems"; document accordingly.
- **`xterm`-based launch files do not work headless.** The four `ch4` launches and `hw4.launch.py` wrap `ros2 bag play` in `xterm -hold`. Running them in this phase **will fail** silently (xterm errors to stderr; nothing publishes). Use the manual sequence above; do not edit the launch files.
- **`odometry_comparison` requires matplotlib display.** `plt.ion()` + `plt.pause()` in [odometry_comparison.py:118-119, 136-179](../ros2_src/ch4/ch4/odometry_comparison.py#L118-L179) require a Qt/Tk/X11 backend. Use `MPLBACKEND=Agg` for liveness-only verification (Part 3). Visual confirmation of trajectories overlap is **out of scope** for behavioral verification — done interactively if needed.
- **`tf_transformations` + NumPy 2 + transforms3d 0.4.2.** `hw4_pkg.ekf_holonomic_drive` imports `tf_transformations` at the top of the module ([ekf_holonomic_drive.py:10](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py#L10)); `ch4.ekf` and `ch4.ekf2` import via `utils.utils.math`. The canary at preflight catches this regression upfront. See [plans/errors/fixed/tf_transformations-numpy2-incompatibility.md](errors/fixed/tf_transformations-numpy2-incompatibility.md) for context — fix is upstream (`python3-transforms3d` ≥ 0.4.2 apt).
- **Bag storage format:** all three bags are **mcap** (`storage_identifier: mcap` in each `metadata.yaml`; payload is `<scenario>.mcap`). Recording in this phase also uses mcap — `ros2 bag record -s mcap` — so the assertion helper only needs to read one format.
- **`/clock` only present in two of three bags.** `circle/` and `straight/` publish `/clock`; `square/` does **not**. For the hw4 combo (`square/`), the bag-play `--clock` flag *publishes* `/clock` itself; the filter's `use_sim_time` parameter will still resolve. If timing alignment looks suspicious in the execution log, cross-check whether `ros2 bag play` reported "publishing clock" at startup.
- **`hw4.launch.py` has the filter node commented out** ([lines 12-20](../ros2_src/hw4_pkg/launch/hw4.launch.py#L12-L20)). Treat this as an upstream curiosity; we run `ros2 run hw4_pkg ekf_holonomic` directly. Do not uncomment as part of this verification (would diverge from the published launch).
- **`assert_filter_rmse.py` is TO BE WRITTEN.** Per [conventions §Helper scripts catalog](behavior-test-conventions.md#helper-scripts-catalog), the helper is written when the phase first exercises it. Reuse the same script across all 10 combos in this phase plus the bl7 EKF localization combos.
- **Per-axis RMSE bounds in the tables are conservative starting points,** chosen so that *any* working filter passes. If a freshly-built node grossly exceeds them (e.g., RMSE > 2× table value), file an incident under `plans/errors/`; do not tighten the bound to make a buggy filter pass.

## On pass

All four parts green → tick **BL4** in the campaign tracker ([/home/georg/.claude/plans/i-m-working-on-porting-glistening-gray.md](../../../.claude/plans/i-m-working-on-porting-glistening-gray.md)) and proceed to **bl5a** ([behavior-test-bl5a-ch5-services-actions.md](behavior-test-bl5a-ch5-services-actions.md)). Record artefacts (the `/tmp/bl4_*` mcaps for each combo) in the per-phase execution log at `executions/behavior-test-bl4-ch4-filters-execution.md`, copied from [executions/behavior-test-template.md](executions/behavior-test-template.md).
