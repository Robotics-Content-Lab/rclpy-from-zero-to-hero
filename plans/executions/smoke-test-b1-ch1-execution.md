---
name: jazzy-container-stale-packages
description: Jazzy container image ships stale ROS packages that break any node using rosidl-generated types — apt upgrades are required before smoke tests will run
metadata: 
  node_type: memory
  type: project
  originSessionId: f8aac1b3-f3b0-4326-b219-dcefe35db982
---

The `ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:jazzy-desktop-vnc-nvidia` container as pulled on 2026-05-16 ships a stale Jazzy package set that prevents *any* rclpy node using rosidl-generated types from initializing. Discovered while executing the B1 ch1 smoke test ([plans/smoke-test-b1-ch1.md](../../../../workspaces/rclpy-from-zero-to-hero/plans/smoke-test-b1-ch1.md)).

Symptoms before fix:
- Missing package: `ros-jazzy-example-interfaces` not installed (every ch* package depends on it).
- `eprosima::fastcdr::Cdr::serialize(char*)` undefined symbol — `ros-jazzy-fastcdr` 2.2.5 too old.
- `RuntimeError: This member is not been selected` from `_rclpy.Node`, root cause `'map is not initialized, at ./src/hash_map.c:537'` — `rclpy` 7.1.9 + `rcl` mismatch.

Fix (executed by subagent during B1 run):
1. `apt-get install ros-jazzy-example-interfaces`
2. Upgrade `ros-jazzy-fastcdr` 2.2.5 → 2.2.7.
3. Upgrade core stack: `rclpy` 7.1.9 → 7.1.11, plus `rcl`, `rmw-implementation`, `rmw-fastrtps-cpp`, `rosidl-*`, `type-description-interfaces`, `rcl-action` to the April-2026 sync.

**Why:** Blocks all five smoke-test plans in `plans/smoke-test-b*.md`. The migration code itself is correct — this is a container-image regression that needs a Dockerfile / devcontainer postCreate fix in [[jazzy-migration-source-of-truth]] (the container repos: `rclpy-from-zero-to-hero-container/`, `rclpy_from_zero_to_hero_container/`).

**How to apply:** Before launching any subsequent smoke test (B3/B4/B5/B6), assume these apt upgrades are already applied in the running container — the subagent fixed them in-place during B1. If a fresh container is pulled later, re-apply or bump the container image. Surface this as an action item in any migration retrospective.
