# Error: `project.world` `<ambient>` colour values out of range — Gazebo Harmonic refuses to load the world

**Discovered:** 2026-05-16 during B5 smoke test execution ([../executions/smoke-test-b5-project-execution.md](../executions/smoke-test-b5-project-execution.md))
**Severity:** **Critical** — blocks B5 Part 1 entirely. `gz sim` exits during world parsing; no `/world/default/create` service is ever advertised; the bridge advertises sensor/odom topics but they carry no data; `spawn_robot` cannot find the spawn service even if its `tf_transformations` import were fixed.
**Type:** Code-side migration regression (world-file content) — not a container or env issue.

## Symptom

`ros2 launch project simulation.launch.py` brings the launch up, but `gz sim` aborts during SDF parsing:

```
[gazebo-3] Error [Param.cc:1288] Error Code 34: Msg: The value <ambient>255 255 255 1</ambient> is invalid.
[gazebo-3] [Err] [Server.cc:86] Error Code 9:
    [/sdf/world[@name="default"]/scene/ambient:.../worlds/project.world:L25]:
    Msg: Error reading element <ambient>
[gazebo-3] [Err] [Server.cc:86] Error Code 9:
    [/sdf/world[@name="default"]/scene:.../worlds/project.world:L24]:
    Msg: Error reading element <scene>
[gazebo-3] [Err] [Server.cc:86] Error Code 9:
    [/sdf/world[@name="default"]:.../worlds/project.world:L2]:
    Msg: Error reading element <world>
[gazebo-3] [Err] [Server.cc:86] Error Code 1:
    Msg: Unable to read file: [.../worlds/project.world]
[INFO] [gazebo-3]: process has finished cleanly [pid 9172]
```

Downstream symptoms in the running launch:

- `ros2 service list | grep create` → empty (no `/world/default/create`)
- `ros2 topic hz /clock /scan /odom /image_raw /imu /joint_states` → all time out with no data
- `ros2 topic info /odom` → 1 publisher (the bridge), 0 subscribers, but the bridge has no upstream gz topic, so no messages flow
- TF tree from `view_frames` contains only the robot_state_publisher chain (`base_footprint → base_link → …`) — no `odom` root, because gz-sim's diff-drive system never started

## Root cause

`project/worlds/project/project.world` line 25 declares:

```xml
<scene>
  <ambient>255 255 255 1</ambient>
  ...
</scene>
```

`<ambient>` in SDF 1.6+/Gazebo Harmonic is a normalised RGBA tuple in `[0, 1]`, *not* the 0-255 byte range that Gazebo Classic / RViz markers use. Harmonic's `sdf::Param::ValueFromString` calls `Vector3<T>::IsFinite` (or equivalent) and validates each channel ∈ `[0, 1]`. `255.0` rejects, the `<ambient>` element fails to parse, and the cascading parser errors abort world load.

The fix is `<ambient>1 1 1 1</ambient>` (white), or whatever subjective brightness is desired — likely the original intent in the Classic world.

This was not caught by B5.3 (`World + model SDF version bumps`) because the SDF version bump (1.6 → 1.9) doesn't itself revalidate colour ranges — both versions allow the same element, only Harmonic's runtime checks the value range.

## Affected files

```text
$ grep -n '<ambient>255' ros2_src/project/worlds/project/project.world ros2_src/projects/ekf_localization/worlds/**/*.world ros2_src/ch5/worlds/**/*.world 2>/dev/null
ros2_src/project/worlds/project/project.world:25:      <ambient>255 255 255 1</ambient>
```

`project.world` is the only confirmed offender. The remaining `<ambient>` occurrences in that file (lines 72, 114, 156, …, all `0.4 0.4 0.4 1`) are inside `<material>` blocks and are valid — only line 25's `<scene>/<ambient>` is wrong.

Other worlds to recheck before B6:

```text
ros2_src/projects/ekf_localization/worlds/...   # check before B6 smoke
ros2_src/ch5/worlds/...                          # not in smoke scope but worth grepping
```

I did **not** find similar out-of-range `<ambient>` in `ekf_localization` or `ch5` worlds, but the grep above should be run explicitly during B6 prep.

## Fix options

### Option A — fix the constant in `project.world` (recommended, ~1 line)

Edit `ros2_src/project/worlds/project/project.world` line 25:

```diff
-      <ambient>255 255 255 1</ambient>
+      <ambient>1 1 1 1</ambient>
```

After editing, `colcon build --packages-select project && source install/setup.bash` and re-run `ros2 launch project simulation.launch.py`. `gz sim` should reach "Loading world" and start publishing `/clock`.

Optional: while editing, also normalise any other `<scene>/<background>` or `<scene>/<grid>` 0-255 values. None observed in the current file, but a global `grep -E "<(ambient|background|diffuse|specular|emissive)>[^<]*25[0-9]"` is a worthwhile sanity check.

### Option B — sed in place during smoke prep

Not recommended — leaves the bug in source. Use Option A.

## Impact on each remaining smoke test

| Test | Impact |
|---|---|
| **B5 Part 1** (`project simulation.launch.py`) | **Blocks**. World never loads → no gz-sim services, no topic data, no spawn possible. Even fixing `tf_transformations`-numpy2 won't help here until the world parses. |
| **B5 Part 2** (`project nav2.launch.py`) | Independent — nav2 only needs the map yaml + AMCL. Verified PASS standalone. After Option A, the combined `project.launch.py` (sim + nav2) should also work. |
| **B6** (`ekf_localization simulation.launch.py`) | Independent unless that world has the same defect. Re-grep `ros2_src/projects/ekf_localization/worlds/` for `255` channel values before B6. |
| **B4 (already done)** | No impact — ch5 smoke didn't use this world. |
| **B1–B3** | No impact. |

## Action items

1. Apply Option A to `ros2_src/project/worlds/project/project.world` line 25. Single character-set fix.
2. Re-run B5 Part 1 after the fix, and only then can the `tf_transformations`-numpy2 issue ([tf_transformations-numpy2-incompatibility.md](tf_transformations-numpy2-incompatibility.md)) be observed independently of the world-load failure.
3. Add a regression test under `ros2_src/project/test/test_jazzy_migration.py` that greps the world file for any `<scene>` child colour element with a value `> 1.0`, so a future paste from a Classic world is caught at `colcon test` time.
4. Before B6, run:
   ```bash
   grep -rn -E "<(ambient|background|diffuse|specular|emissive)>[[:space:]]*[0-9]+" \
     ros2_src/projects/ekf_localization/worlds/ ros2_src/ch5/worlds/ \
     | awk -F: '{for (i=NF-2;i<=NF;i++) print}' | grep -v "0\\.[0-9]"
   ```
   Any line printed needs the same 0-255 → 0-1 conversion.
