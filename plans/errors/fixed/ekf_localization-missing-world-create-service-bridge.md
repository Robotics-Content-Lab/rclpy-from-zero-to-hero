# Error: `landmark_spawner` cannot reach `/world/default/create` — no ROS service bridge configured

**Discovered:** 2026-05-16 during B6 smoke test execution ([../executions/smoke-test-b6-ekf_localization-execution.md](../executions/smoke-test-b6-ekf_localization-execution.md))
**Severity:** **Critical for B6 only** — blocks the landmark-spawning portion of the EKF localization launch entirely. TB3 spawn still works (it uses a different code path); all sensor/odom topics still flow. But zero landmarks ever appear in gz, so the EKF localization scenarios that depend on them are unusable.
**Type:** Code-side migration regression (launch file wiring) — not a container or env issue.

## Symptom

`ros2 launch ekf_localization simulation.launch.py` brings the launch up; TB3 burger spawns at the origin; `/scan`, `/odom`, `/clock`, `/tf` all carry data. But the `landmark_spawner` node loops forever:

```
[landmark_spawner-6] [INFO] [...] [landmark_spawner]: service not available, waiting again...
[landmark_spawner-6] [INFO] [...] [landmark_spawner]: service not available, waiting again...
[landmark_spawner-6] [INFO] [...] [landmark_spawner]: service not available, waiting again...
...  (repeats indefinitely)
```

Downstream:

- `gz model --list` → only `ground_plane` and `burger`; **0 cylinders**.
- `ros2 service list` → `/world/default/create` is listed (✅)
- `ros2 service info /world/default/create` →
  ```
  Type: ros_gz_interfaces/srv/SpawnEntity
  Clients count: 1
  Services count: 0       ← no ROS-side server
  ```
- `gz service -l | grep create` →
  ```
  /world/default/create
  /world/default/create/blocking
  /world/default/create_multiple
  /world/default/create_multiple/blocking
  ```
  The service exists, but only on the **gz-transport** side; nothing bridges it to ROS, so rclpy's `wait_for_service` never returns true.

The same race-condition wording in the plan ("re-run if it happens once") does not apply — this is not a race; it's structurally missing.

## Root cause

The launch file [ros2_src/projects/ekf_localization/launch/simulation.launch.py](../../ros2_src/projects/ekf_localization/launch/simulation.launch.py) bridges only `/clock` between gz and ROS:

```python
clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    output='screen',
)
```

The rest of the TB3 sensor/cmd_vel bridges are inherited from `spawn_turtlebot3.launch.py` (another `parameter_bridge` covering `joint_states`, `odom`, `tf`, `cmd_vel`, `imu`, `scan` — confirmed by `ros2 node list` showing **two** `/ros_gz_bridge` nodes).

**But neither bridge declares any service mapping**, and `parameter_bridge`'s topic-only YAML wouldn't cover services anyway. In Gazebo Harmonic, `/world/<world_name>/create` is an `ignition`/`gz`-transport service, not a ROS service. Bridging it to ROS requires either:

1. **`ros_gz_bridge`'s `bridge.yaml`** with a `service:` block (added in jazzy), or
2. **A dedicated `ros_gz_bridge` executable** with `--ros-args -p config_file:=…` listing the service, or
3. Calling `ros_gz_sim::create` (the same approach TB3's `spawn_turtlebot3.launch.py` uses internally — see `[create-4]` lines in the launch log) instead of building a rclpy service client.

The Classic Iron version of this launch relied on `gazebo_ros_factory`, which advertised `/spawn_entity` as a ROS service directly. Harmonic does not — services are gz-transport-native and must be bridged explicitly. The migration converted the message namespace (`gazebo_msgs/SpawnEntity` → `ros_gz_interfaces/SpawnEntity`) and the service name (`/spawn_entity` → `/world/default/create`) but did not add the bridge wiring.

Note: the in-tree test [ros2_src/projects/ekf_localization/test/test_jazzy_migration.py](../../ros2_src/projects/ekf_localization/test/test_jazzy_migration.py) only asserts that the Classic endpoint string `/spawn_entity` no longer appears — it does not verify that the new endpoint actually works at runtime.

## Affected files

```text
ros2_src/projects/ekf_localization/launch/simulation.launch.py   # the launch — missing service bridge wiring
ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py  # the rclpy client side (currently correct, but no server)
ros2_src/projects/ekf_localization/ekf_localization/landmark_spawner.py  # entry point — the hanging node
```

Other launches that spawn entities via the same pattern (grep `ros_gz_interfaces.*SpawnEntity` across the repo):

```text
$ grep -rln "ros_gz_interfaces.*SpawnEntity\|ros_gz_interfaces.srv.*SpawnEntity" ros2_src/
ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py
ros2_src/project/project/spawn_robot.py
```

The `project` package's `spawn_robot.py` uses the same client pattern but its launch was already FAIL-blocked by B5 (world parse error + tf_transformations bug). When B5 is fixed, it will surface the **same** missing-service-bridge issue unless the project launch is also patched.

## Fix options

### Option A — switch `landmark_spawner` to call the gz-transport service directly (recommended)

Replace the rclpy `create_client(SpawnEntity, …)` pattern with a `gz` CLI invocation or use `ros_gz_sim`'s `create` executable, the same way `spawn_turtlebot3.launch.py` already does.

A minimal change in `landmark_spawner`:

```python
import subprocess
import os, tempfile

def spawn_one(landmark, model_xml, i):
    # Write model_xml to a temp file (gz needs an SDF path)
    with tempfile.NamedTemporaryFile(suffix='.sdf', mode='w', delete=False) as f:
        f.write(model_xml); path = f.name
    subprocess.run([
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', 'default', '-file', path,
        '-name', f'cylinder_{i}',
        '-x', str(landmark['x']), '-y', str(landmark['y']), '-z', '0',
    ], check=True)
    os.unlink(path)
```

**Pros:** Uses the same well-tested path as TB3 spawn. No bridge wiring needed.
**Cons:** Subprocess per landmark (~130 spawns), slower than batched async calls. For 80–130 cylinders this is still seconds, not minutes.

### Option B — add a service bridge in the launch file

Add a second `parameter_bridge` or `ros_gz_bridge` instance with a config YAML covering the service:

```yaml
# config/gz_service_bridge.yaml
- topic_name: "/world/default/create"
  ros_type_name: "ros_gz_interfaces/srv/SpawnEntity"
  gz_type_name: "gz.msgs.EntityFactory"
  direction: BIDIRECTIONAL  # service
```

Then in `simulation.launch.py`:

```python
service_bridge = Node(
    package='ros_gz_bridge',
    executable='bridge',  # ros_gz_bridge >= 1.0 has a 'bridge' executable for YAML
    arguments=['--ros-args', '-p',
               f'config_file:={get_package_share_directory("ekf_localization")}/config/gz_service_bridge.yaml'],
)
```

⚠️ Service bridging via YAML landed in jazzy's `ros_gz_bridge` (commit 8d3f0a8 in `gazebo-ros2-control`). Confirm the installed version supports `service:` entries before relying on this; otherwise fall back to Option A.

**Pros:** Keeps the pure-rclpy spawn loop in `landmark_spawner`, which is pedagogically nicer for the book.
**Cons:** Adds a YAML file + extra Node; depends on `ros_gz_bridge` version pinning.

### Option C — call `gz-transport` directly from Python via `gz.msgs10`/`ign-msgs`

Python bindings exist (`pip install gz-transport13`) but are not packaged in the Jazzy apt set. Not recommended.

**Recommendation:** Option A. Fewest moving parts, matches the proven path TB3 already uses, doesn't depend on `ros_gz_bridge` feature flags.

## Impact on each remaining smoke test

| Test | Impact |
|---|---|
| **B6** | **Blocks landmark spawning.** Everything else (TB3 spawn, sensors, TF, teleop) works. 9 / 10 acceptance criteria still pass; only "Multiple landmark cylinders appear" fails. |
| **B5** (already FAIL for unrelated reasons) | Same pattern — `spawn_robot.py` will hit it next, once the world-parse and tf_transformations issues are fixed. Already noted in [project-world-ambient-color-out-of-range.md](project-world-ambient-color-out-of-range.md). |
| **B1–B4** | No impact — none of these launches spawn entities via this pattern. |

This is the **last smoke test in the queue** (B6 is the final phase). No downstream phases are blocked by it.

## Action items

1. Apply Option A to `ros2_src/projects/ekf_localization/ekf_localization/landmark_spawner.py` (and the shared helper `utils/gazebo.py`). After the fix, expect `gz model --list` to show ~130 cylinders.
2. Apply the same fix (or Option B) to `ros2_src/project/project/spawn_robot.py` while it's open — this surfaces in B5 too.
3. Extend `ros2_src/projects/ekf_localization/test/test_jazzy_migration.py` so it asserts that *either* the launch includes a service bridge for `/world/default/create`, *or* `landmark_spawner` does not use `rclpy.create_client` for that service. (i.e. assert that the runtime path is wired, not just that the legacy string is gone.)
4. After re-run, verify that the EKF localization YAML's ~130 entries → ~130 unique `cylinder_N` models (note: many duplicates in the YAML mean the gz `create` calls will fail-with-name-collision for repeats; either deduplicate the YAML or accept the failures and report them).
