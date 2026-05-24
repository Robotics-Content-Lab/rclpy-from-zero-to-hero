# Error: `tf_transformations` crashes at import on Jazzy (numpy 2.0 incompatibility)

**Discovered:** 2026-05-16 during B4 smoke test execution ([../executions/smoke-test-b4-ch5-ch6-execution.md](../executions/smoke-test-b4-ch5-ch6-execution.md))
**Severity:** High — blocks the ch5 action server (B4) and `project/spawn_robot` (B5 launch). Behavioural verification of every ch3/ch4 controller broken.
**Type:** Code-side migration regression (not a container or env issue)

## Symptom

Any rclpy node that imports `tf_transformations` (directly or transitively) crashes at module-load time:

```
Traceback (most recent call last):
  File ".../ch5/holonomic_closed_loop_square_action_server.py", line N, in <module>
    from utils.utils.math import euler_from_quaternion
  File ".../utils/utils/utils/math.py", line 2, in <module>
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
  File "/usr/lib/python3/dist-packages/tf_transformations/__init__.py", line N, in <module>
    import transforms3d
  File "/usr/lib/python3/dist-packages/transforms3d/quaternions.py", line N, in <module>
    _MAX_FLOAT = np.maximum_sctype(np.float64)
AttributeError: module 'numpy' has no attribute 'maximum_sctype'
```

## Root cause

- Jazzy container ships `python3-transforms3d` **0.4.1** (apt).
- `transforms3d 0.4.1` calls `numpy.maximum_sctype`, removed in **NumPy 2.0**.
- Jazzy container ships `numpy 2.4.4`.
- `tf_transformations` (the ROS 2 wrapper) re-exports `transforms3d`, so the failure cascades to every consumer.

## Affected files

### Direct importers of `tf_transformations`

| File | Symbol |
|---|---|
| [ros2_src/utils/utils/utils/math.py:2](../../ros2_src/utils/utils/utils/math.py#L2) | `euler_from_quaternion`, `quaternion_from_euler` (re-exported) |
| [ros2_src/utils/utils/utils/rviz_markers.py](../../ros2_src/utils/utils/utils/rviz_markers.py) | `quaternion_from_euler` |
| [ros2_src/project/project/spawn_robot.py](../../ros2_src/project/project/spawn_robot.py) | `quaternion_from_euler` — **blocks B5 launch** |
| [ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py](../../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py) | both |

### Indirect importers (via `utils.utils.math`)

| File | Symbol |
|---|---|
| [ros2_src/ch3/ch3/closed_loop_p.py](../../ros2_src/ch3/ch3/closed_loop_p.py) | `euler_from_quaternion`, `normalize` |
| [ros2_src/ch3/ch3/closed_loop_p_square.py](../../ros2_src/ch3/ch3/closed_loop_p_square.py) | `euler_from_quaternion`, `normalize` |
| [ros2_src/ch3/ch3/closed_loop_pid.py](../../ros2_src/ch3/ch3/closed_loop_pid.py) | `euler_from_quaternion`, `normalize` |
| [ros2_src/ch3/ch3/go_to.py](../../ros2_src/ch3/ch3/go_to.py) | `euler_from_quaternion`, `normalize` |
| [ros2_src/ch4/ch4/ekf.py](../../ros2_src/ch4/ch4/ekf.py) | both, `normalize` |
| [ros2_src/ch4/ch4/ekf2.py](../../ros2_src/ch4/ch4/ekf2.py) | both, `normalize` |
| [ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py](../../ros2_src/ch5/ch5/holonomic_closed_loop_square_action_server.py) | `euler_from_quaternion` — **blocked B4 action server** |
| [ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py](../../ros2_src/ch5/ch5/holonomic_closed_loop_square_service_server.py) | `euler_from_quaternion` |
| [ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py](../../ros2_src/hw3_pkg/hw3_pkg/trajectory_controller.py) | `euler_from_quaternion`, `normalize` |
| [ros2_src/hw4_pkg/hw4_pkg/kf_2d.py](../../ros2_src/hw4_pkg/hw4_pkg/kf_2d.py) | `quaternion_from_euler` |

### Not affected (define their own conversion)

- `ros2_src/projects/ekf_localization/ekf_localization/ekf_localization_with_{known,unknown}_correspondences*.py` — inline `euler_from_quaternion`
- `ros2_src/projects/ekf_localization/ekf_localization/ekf_localization_wukc.py` — inline
- `ros2_src/projects/bug_planning/bug_planning/utils/transforms.py` — inline

## Fix options

### Option A — replace `tf_transformations` with inline math (recommended)

Drop the external dependency. The four functions needed are tiny:

```python
# utils/utils/utils/math.py
from math import atan2, sin, cos, asin
import numpy as np

def quaternion_from_euler(roll=0.0, pitch=0.0, yaw=0.0):
    cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
    cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
    cr, sr = cos(roll * 0.5), sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)

def normalize(angle):
    return atan2(sin(angle), cos(angle))
```

Same pattern for the four direct importers — replace `from tf_transformations import ...` with the local helper or inline it.

Then drop `tf_transformations` and `transforms3d` from every `package.xml` / `setup.py` `install_requires` that mentions them.

**Pros:** No external dependency, ROS-distro-independent, fits the book's pedagogy (students see the math, not a black box).
**Cons:** Minor risk of sign convention drift — pin the math to the test cases of any existing controller that already worked.

### Option B — `pip install --upgrade transforms3d`

Newer `transforms3d` (≥ 0.4.2) should remove the `np.maximum_sctype` call. Quick fix but mixes pip into an apt-managed Python environment — discouraged in ROS containers.

### Option C — wait for Debian/upstream

`ros-jazzy-tf-transformations` will eventually pin a numpy-2-safe `transforms3d`. Blocking on this delays the book indefinitely.

**Recommendation:** Option A. Track as a follow-up under the migration plan, ideally in a new phase B8 (or as part of B4/B5 closeout).

## How smoke tests should treat this

- **B1–B3:** unaffected (ch1 has no math; ch2/ch3 launches don't auto-run controllers; smoke scope is launch + topics only).
- **B4 (ch5):** action server `square_action_server` crashes at startup. Already documented in [../executions/smoke-test-b4-ch5-ch6-execution.md](../executions/smoke-test-b4-ch5-ch6-execution.md). Mark the matching checkbox FAIL until fixed.
- **B5 (project):** `spawn_robot` is invoked by `project/launch/simulation.launch.py:105-111` — it will fail to spawn the robot. The launch itself starts (the other Nodes initialize), but `/world/default/create` is never called. Acceptance criterion "TurtleBot3 burger spawned via `/world/default/create`" will fail.
- **B6 (ekf_localization):** unaffected — `landmark_spawner` doesn't touch `tf_transformations`; the EKF nodes that do are not in the smoke launch.

## Action items

1. Apply Option A to the 4 direct importers (`utils/utils/utils/math.py`, `utils/utils/utils/rviz_markers.py`, `project/spawn_robot.py`, `hw4_pkg/ekf_holonomic_drive.py`).
2. Strip `tf_transformations`/`transforms3d` from `package.xml`s of `utils`, `project`, `hw4_pkg`.
3. Re-run B4 action server/client and B5 launch after the fix to confirm.
4. Add a regression test under each affected package's `test/` that imports the math helper, so future numpy bumps don't silently regress.
