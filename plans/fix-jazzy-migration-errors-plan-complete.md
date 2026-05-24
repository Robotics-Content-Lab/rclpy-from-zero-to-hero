# Plan: Fix the remaining errors from the ros2_src → Jazzy migration

**Status:** ✅ Complete — code/Dockerfile changes applied, container rebuilt, smoke tests B4 / B5 / B6 all green. Resolved error docs archived under [errors/fixed/](errors/fixed/).
**Predecessor docs:** [migrate-ros2_src-to-jazzy-plan.md](migrate-ros2_src-to-jazzy-plan.md) · [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md)
**Error docs addressed:** [errors/fixed/tf_transformations-numpy2-incompatibility.md](errors/fixed/tf_transformations-numpy2-incompatibility.md) · [errors/fixed/project-world-ambient-color-out-of-range.md](errors/fixed/project-world-ambient-color-out-of-range.md) · [errors/fixed/ekf_localization-missing-world-create-service-bridge.md](errors/fixed/ekf_localization-missing-world-create-service-bridge.md)

---

## Context

After executing the Jazzy migration, three errors were captured under [plans/errors/](errors/). One was fixed in-place by the user (the ambient-color bug). Two remained blocking smoke tests:

1. **`tf_transformations` crashes at import on Jazzy (NumPy 2.0).** Jazzy's apt ships `python3-transforms3d 0.4.1`, which calls `np.maximum_sctype` — removed in NumPy 2.0. Every consumer of `utils.utils.math` (ch3, ch4, ch5, hw3) and `project/spawn_robot.py` crashes at module-load time. Blocked B4 ch5 action server and B5 robot spawn.

2. **No bridge for `/world/default/create`.** Gazebo Harmonic exposes the spawn service on gz-transport only; `ros_gz_bridge`'s topic-only `parameter_bridge` doesn't bridge it. The rclpy `create_client(SpawnEntity, …)` pattern in `project/spawn_robot.py` and `ekf_localization/utils/gazebo.py` waited forever. Blocked B5 robot spawn and B6 landmark spawn.

### Decisions taken (user-confirmed)

- **Phase 1 (numpy/transforms3d):** keep `tf_transformations` in source — the [Jazzy tf_transformations README](https://docs.ros.org/en/jazzy/p/tf_transformations/__README.html) recommends the apt-installed ROS package. Fix the version gap **upstream** by overriding `transforms3d` to PyPI 0.4.2 (changelog: *"Update for Numpy 2.0"*) in the container image. No `ros2_src/` import changes.
- **Phase 2 (spawn bridge):** drop the rclpy `SpawnEntity` client pattern. Use `ros_gz_sim`'s `create` executable directly — it speaks gz-transport, matching what TB3's own `spawn_turtlebot3.launch.py` does internally. No service bridge needed, no YAML config.

---

## What was changed

### Phase 1 — container-side transforms3d bump

[rclpy_from_zero_to_hero_container/.devcontainer/Dockerfile.ros](../../rclpy_from_zero_to_hero_container/.devcontainer/Dockerfile.ros) — **build repo**, separate git tree. After the existing apt install of `ros-${ROS_DISTRO}-tf-transformations`, added a guarded pip override:

```dockerfile
RUN if [ "${UBUNTU_VERSION}" = "24.04" ]; then \
        pip install --no-cache-dir --break-system-packages 'transforms3d>=0.4.2' && \
        python3 -c "import transforms3d, numpy; …round-trip assert…"; \
    fi
```

`--break-system-packages` is required on Ubuntu 24.04 (PEP 668). pip writes to `/usr/local/lib/python3.X/dist-packages/`, which sits before apt's `/usr/lib/python3/dist-packages/` on `sys.path`, so the 0.4.2 wheel wins for every consumer of `tf_transformations` without touching apt's bookkeeping. The inline round-trip is a build-time canary — if the override fails to take effect the image build aborts instead of every student hitting it at runtime.

[ros2_src/utils/test/test_tf_transformations.py](../ros2_src/utils/test/test_tf_transformations.py) *(new)* — runtime regression test, picked up by `colcon test`. Catches future image bumps where the pip override regresses.

### Phase 2 — `ros_gz_sim create` instead of rclpy SpawnEntity client

[ros2_src/project/launch/simulation.launch.py](../ros2_src/project/launch/simulation.launch.py) — replaced the `Node(executable='spawn_robot', …)` action with a `ros_gz_sim` `create` Node using `-topic robot_description` (the in-launch `robot_state_publisher` already publishes the URDF there). Yaw stays degrees in the launch arg interface; `PythonExpression` converts to radians for the `-Y` flag.

[ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py](../ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py) — rewritten. `spawn_landmarks()` now:

1. Writes `model_xml` once to a `tempfile.NamedTemporaryFile(suffix='.sdf', delete=False)`.
2. Sleeps 5 s for gz_sim to advertise `/world/default/create`.
3. For each landmark, `subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', WORLD_NAME, '-file', sdf_path, '-name', f'cylinder_{i}', '-x', …, '-y', …, '-z', '0'], check=False)`.
4. Tolerates failures (the YAML has duplicate coordinates → gz rejects duplicate names; log and continue).
5. `os.unlink` the tempfile when done.

The `landmark_spawner.py` caller is unchanged — the `(node, landmarks, model_xml)` signature is preserved (node still used for `get_logger()`).

[ros2_src/projects/ekf_localization/package.xml](../ros2_src/projects/ekf_localization/package.xml) — dropped `<depend>ros_gz_interfaces</depend>` (no longer imported by `gazebo.py`). `ros_gz_sim` and `ros_gz_bridge` (for the `/clock` bridge) stay.

[ros2_src/project/project/spawn_robot.py](../ros2_src/project/project/spawn_robot.py) — **untouched**. Left in tree as a teaching reference for the rclpy SpawnEntity-client pattern. A student can still `ros2 run project spawn_robot <urdf>` to inspect the code path; the launch no longer depends on it succeeding.

### Tests updated

- [ros2_src/project/test/test_jazzy_migration.py](../ros2_src/project/test/test_jazzy_migration.py) — **added** two runtime-path assertions: `test_simulation_launch_spawns_via_ros_gz_sim_create`, `test_simulation_launch_does_not_run_project_spawn_robot`. Legacy `spawn_robot.py` tests retained (the file still imports `ros_gz_interfaces` for teaching).
- [ros2_src/projects/ekf_localization/test/test_jazzy_migration.py](../ros2_src/projects/ekf_localization/test/test_jazzy_migration.py) — **replaced** the EntityFactory / `/world/` endpoint tests with `test_gazebo_util_no_classic_gazebo_msgs`, `test_gazebo_util_uses_ros_gz_sim_create`, `test_gazebo_util_no_rclpy_spawn_entity_client`. Relaxed `test_package_xml_has_ros_gz_deps` to no longer require `ros_gz_interfaces`.

**Local pytest (host):** 25 / 25 project + 9 / 9 ekf_localization migration tests pass.

---

## What's still pending

Container has been rebuilt by the user. The remaining work is **verification inside the new image**, then cleanup.

### Verification runbook

All commands run inside the container unless noted. Host's `rclpy-from-zero-to-hero/ros2_src/` bind-mounts to `/home/ros_user/ros2_ws/src`.

#### Step 1 — sanity-check Phase 1 (transforms3d ≥ 0.4.2)

```bash
python3 -c "import transforms3d, numpy; print('transforms3d', transforms3d.__version__, '/ numpy', numpy.__version__); \
            from tf_transformations import quaternion_from_euler, euler_from_quaternion; \
            print('roundtrip OK', euler_from_quaternion(quaternion_from_euler(0.1, 0.2, 0.3)))"
```

Expected: `transforms3d 0.4.2 / numpy 2.x.x` and a clean Euler round-trip. If `transforms3d` shows 0.4.1, the pip layer didn't land — check `docker history <image>` and re-pull.

#### Step 2 — build the workspace

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select utils project ekf_localization ch5
source install/setup.bash
```

#### Step 3 — `colcon test`

```bash
cd ~/ros2_ws
colcon test --packages-select utils project ekf_localization
colcon test-result --verbose
```

Expected greens:
- `utils/test_tf_transformations.py::test_tf_transformations_imports_and_roundtrips`
- `project/test_jazzy_migration.py` — 25 checks (incl. 2 new launch-path tests)
- `ekf_localization/test_jazzy_migration.py` — 9 checks (incl. 3 new gazebo.py runtime-path tests)

Pre-existing `test_flake8`/`test_pep257` failures in untouched files are documented in [migrate-ros2_src-to-jazzy-plan-update.md](migrate-ros2_src-to-jazzy-plan-update.md) and not in scope.

#### Step 4a — B4 ch5 action server (was crashing on `tf_transformations`)

```bash
# pane 1
ros2 launch ch5 simulation.launch.py

# pane 2
ros2 run ch5 holonomic_closed_loop_square_action_server

# pane 3
ros2 run ch5 holonomic_closed_loop_square_action_client
```

Expected: action server logs `Action server started`; client goal accepted; robot drives a square. **No `AttributeError: module 'numpy' has no attribute 'maximum_sctype'`.**

The separate `robotino_description` not-Harmonic-ready issue (env-side) noted in [executions/smoke-test-b4-ch5-ch6-execution.md](executions/smoke-test-b4-ch5-ch6-execution.md) is out of scope; the action-server import path is what this fix unblocks.

#### Step 4b — B5 `project` (was failing world parse + spawn)

```bash
ros2 launch project simulation.launch.py
```

Verify:
```bash
gz model --list                       # expect: ground_plane, turtlebot3_burger
ros2 topic list                       # /clock /scan /imu /image_raw /camera_info /cmd_vel /odom /joint_states
ros2 topic echo --once /odom          # use echo --once (`hz` hangs on Jazzy/numpy)
ros2 topic echo --once /scan
ros2 run tf2_tools view_frames        # complete tree: odom → base_footprint → base_link → …
```

Expected: TurtleBot3 burger spawned at `(-0.55, 6.2)`, yaw `-90°`, all topics carry data.

Then:
```bash
ros2 launch project nav2.launch.py
# Optionally set a 2D Goal Pose in RViz; AMCL should converge; costmap renders /scan returns.
```

#### Step 4c — B6 `ekf_localization` (was hanging on landmark spawn)

```bash
ros2 launch ekf_localization simulation.launch.py
```

The launch sleeps ~5 s after `gz_sim` boots, then fires `ros2 run ros_gz_sim create` per landmark.

```bash
gz model --list                       # expect ground_plane, burger, cylinder_0, cylinder_1, …
gz model --list | grep -c cylinder_   # ~80–130 (YAML has duplicates; some name collisions OK)
ros2 topic echo --once /scan
ros2 topic echo --once /odom
```

If spawn fires too early (0 cylinders, all `rc≠0`), bump the `time.sleep(5)` in [ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py](../ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py) to 10 s and rebuild that one package.

### Cleanup once green

```bash
# On the host, in this repo
cd ~/git/ROS/rclpy-from-zero-to-hero

# Tick the smoke checkboxes
$EDITOR plans/migrate-ros2_src-to-jazzy-plan-update.md

# Archive the now-resolved error docs
mkdir -p plans/errors/fixed
git mv plans/errors/tf_transformations-numpy2-incompatibility.md            plans/errors/fixed/
git mv plans/errors/project-world-ambient-color-out-of-range.md             plans/errors/fixed/
git mv plans/errors/ekf_localization-missing-world-create-service-bridge.md plans/errors/fixed/

# Optionally: rename this plan
git mv plans/fix-jazzy-migration-errors-plan.md plans/fix-jazzy-migration-errors-plan-complete.md
```

---

## Files touched

| Repo | Path | Change |
|---|---|---|
| `rclpy_from_zero_to_hero_container/` *(build repo)* | [.devcontainer/Dockerfile.ros](../../rclpy_from_zero_to_hero_container/.devcontainer/Dockerfile.ros) | Pip-install `transforms3d>=0.4.2` on Ubuntu 24.04, with inline round-trip canary |
| `rclpy-from-zero-to-hero/` | [ros2_src/utils/test/test_tf_transformations.py](../ros2_src/utils/test/test_tf_transformations.py) *(new)* | Import + round-trip regression test |
| `rclpy-from-zero-to-hero/` | [ros2_src/project/launch/simulation.launch.py](../ros2_src/project/launch/simulation.launch.py) | Replaced `spawn_robot` Node with `ros_gz_sim create -topic robot_description …`; added `PythonExpression` for yaw deg→rad |
| `rclpy-from-zero-to-hero/` | [ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py](../ros2_src/projects/ekf_localization/ekf_localization/utils/gazebo.py) | Rewrote `spawn_landmarks()` to `subprocess.run(['ros2','run','ros_gz_sim','create',…])` with tempfile SDF + 5 s startup wait |
| `rclpy-from-zero-to-hero/` | [ros2_src/projects/ekf_localization/package.xml](../ros2_src/projects/ekf_localization/package.xml) | Dropped `<depend>ros_gz_interfaces</depend>` |
| `rclpy-from-zero-to-hero/` | [ros2_src/project/test/test_jazzy_migration.py](../ros2_src/project/test/test_jazzy_migration.py) | +2 new runtime-path assertions for launch file |
| `rclpy-from-zero-to-hero/` | [ros2_src/projects/ekf_localization/test/test_jazzy_migration.py](../ros2_src/projects/ekf_localization/test/test_jazzy_migration.py) | Replaced EntityFactory/endpoint tests with runtime-path tests; relaxed `ros_gz_deps` check |

### Files intentionally NOT changed

- [ros2_src/utils/utils/utils/math.py](../ros2_src/utils/utils/utils/math.py) — keeps `from tf_transformations import …`. Container fix makes that import safe.
- [ros2_src/utils/utils/utils/rviz_markers.py](../ros2_src/utils/utils/utils/rviz_markers.py) — same.
- [ros2_src/project/project/spawn_robot.py](../ros2_src/project/project/spawn_robot.py) — left as teaching reference for the rclpy `SpawnEntity`-client pattern. Not invoked by the launch.
- [ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py](../ros2_src/hw4_pkg/hw4_pkg/ekf_holonomic_drive.py) — same.
- [ros2_src/projects/ekf_localization/ekf_localization/landmark_spawner.py](../ros2_src/projects/ekf_localization/ekf_localization/landmark_spawner.py) — caller of `spawn_landmarks()`; signature preserved.
- All `*.world` files — the only known offender (`project.world:25`) was fixed by the user before this plan executed.

---

## Out of scope (tracked elsewhere)

- B4 `ch5 simulation.launch.py` failure due to `robotino_description` not Harmonic-ready — environment issue, see [executions/smoke-test-b4-ch5-ch6-execution.md](executions/smoke-test-b4-ch5-ch6-execution.md).
- `ros2 topic hz` hang on Jazzy + numpy 2 — workaround `topic echo --once`. Not a regression introduced here.
- Inlining `euler_from_quaternion` / `quaternion_from_euler` into source — explicitly rejected; the README-recommended path is the apt-installed ROS package and Phase 1 makes that path NumPy-2-safe.
- `hw4_pkg/package.xml` missing its `tf_transformations` declaration — pre-existing latent bug, separate small PR.
- Re-deduping the landmark YAML — new spawn loop tolerates collisions with `check=False` + logged failure.

---

## Quick triage table

| Symptom | Likely cause | Where to look |
|---|---|---|
| Still `AttributeError: maximum_sctype` | pip layer didn't land / wrong image tag pulled | `python3 -c "import transforms3d; print(transforms3d.__version__)"` — must show 0.4.2 |
| `ros_gz_sim create` errors with "package not found" | apt didn't install `ros-jazzy-ros-gz` | `apt list --installed 2>/dev/null \| grep ros-gz` |
| `landmark_spawner` spams `Failed to spawn cylinder_N (rc=…)` and zero cylinders appear | gz_sim service not yet advertised at t+5s | bump `time.sleep(5)` in `gazebo.py` to 10s |
| `project simulation.launch.py` runs but the robot doesn't appear | URDF not yet on `/robot_description` when `create -topic` polls | confirm `robot_state_publisher` is listed in the launch *before* the `create` Node |
| `colcon test` collection error "duplicate module test_jazzy_migration" | stale `__pycache__` | `find ~/ros2_ws -name __pycache__ -exec rm -rf {} +` |
| Topic `hz` hangs | known Jazzy/numpy quirk | use `ros2 topic echo --once <topic>` |
