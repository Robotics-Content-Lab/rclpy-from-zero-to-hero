# Smoke Test — Phase B1 (`ch1`)

**Phase:** B1 (Trivial packages)
**Launch under test:** `ros2 launch ch1 first_pub_sub.launch.py`
**Tracks:** the "Smoke: `ros2 launch ch1 first_pub_sub.launch.py`" checkbox in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md) under Phase B1.

Verification scope: **launch starts cleanly and the expected nodes/topics appear**. No behavioral checks (message content, throughput beyond "non-zero") — those are out of scope per the migration plan's "launch + topics only" decision.

## Pre-flight

Inside the Jazzy container (`ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia`):

```bash
cd /workspaces/rclpy-from-zero-to-hero
colcon build --packages-select ch1
source install/setup.bash
```

No external packages are required — `ch1` is pure rclpy with `example_interfaces`.

## Expected nodes

| Node name | Package | Executable | Source |
|---|---|---|---|
| `doumbledore` | `ch1` | `first_publisher` | Direct in [ros2_src/ch1/launch/first_pub_sub.launch.py:5-10](../ros2_src/ch1/launch/first_pub_sub.launch.py#L5-L10) |
| `ron` | `ch1` | `first_subscriber` | Direct in [ros2_src/ch1/launch/first_pub_sub.launch.py:11-16](../ros2_src/ch1/launch/first_pub_sub.launch.py#L11-L16) |

## Expected topics

| Topic | Message type | Direction | Owner |
|---|---|---|---|
| `/points` | `example_interfaces/msg/String` | pub | `doumbledore` (`ch1.first_publisher`) |
| `/some_points` | `example_interfaces/msg/String` | sub | `ron` (`ch1.first_subscriber`) |

QoS: both endpoints use `qos_profile_system_default`. Publisher timer fires every 1.0 s.

## Expected services / actions

None.

## TF frames

None.

## Verification commands

After running `ros2 launch ch1 first_pub_sub.launch.py` in one terminal, open a second terminal and run:

```bash
source /workspaces/rclpy-from-zero-to-hero/install/setup.bash

# 1. Both nodes alive
ros2 node list
#   expected to contain: /doumbledore, /ron

# 2. Both topics advertised
ros2 topic list
#   expected to contain: /points, /some_points

# 3. Publisher is actually publishing
ros2 topic hz /points
#   expected: ~1.0 Hz

# 4. Subscriber is connected to its (mismatched) topic
ros2 topic info /some_points
#   expected: 1 Subscriber count

# 5. No error output in the launch terminal
```

## Acceptance criteria

Smoke test is **green** when ALL of:

- [ ] `ros2 launch ch1 first_pub_sub.launch.py` runs without Python tracebacks.
- [ ] `ros2 node list` returns both `/doumbledore` and `/ron`.
- [ ] `ros2 topic list` returns both `/points` and `/some_points`.
- [ ] `ros2 topic hz /points` reports ~1 Hz.
- [ ] `ros2 topic info /some_points` shows ≥1 subscriber count.

## Known caveats

- **Topics intentionally mismatched.** `first_publisher.py` writes to `/points`; `first_subscriber.py` reads from `/some_points`. The book uses this to demonstrate why you need remappings — they are connected end-to-end in `pub-sub.launch.py` (which remaps both to `/house_points`), not in `first_pub_sub.launch.py`. The smoke test verifies the *launch wiring*, not message flow. Do not "fix" the topic names.
- If you also want to verify end-to-end message flow, run `ros2 launch ch1 pub-sub.launch.py` and `ros2 topic echo /house_points` — but this is bonus, not required for B1.

## On pass

Tick the `- [ ] Smoke: ros2 launch ch1 first_pub_sub.launch.py` line under **Phase B1** in `migrate-ros2_src-to-jazzy-plan-update.md`.
