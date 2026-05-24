# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Repo Is

Educational codebase for the book *RCLPY: From Zero to Hero* by Georg A. Novotny. All ROS 2 packages live under `ros2_src/` and are organized by chapter, progressing from basic nodes through SLAM and path planning.

## Build & Test Commands

All commands assume the ROS 2 workspace is sourced (`source /opt/ros/iron/setup.bash`) and you are in `/workspaces/rclpy-from-zero-to-hero/`.

```bash
# Build everything
colcon build

# Build a single package
colcon build --packages-select <package_name>

# Run all tests
colcon test

# Run tests for a single package
colcon test --packages-select <package_name>

# View test results
colcon test-result --verbose
```

Each package's tests cover copyright headers, flake8 (PEP 8), and pep257 docstring conventions. There are no unit tests for node behavior — test packages are in `<pkg>/test/`.

After building, source the workspace overlay:
```bash
source install/setup.bash
```

## Package Architecture

All packages are `ament_python` (Python, `setup.py`), except `ch5_interfaces` which is `ament_cmake` and generates message/service/action types using `rosidl_generate_interfaces`.

**Chapter packages** (learning progression):
- `ch1` — Nodes, publishers, subscribers, timers, launch files
- `ch2` — QoS profiles, callback groups (`ReentrantCallbackGroup`, `MutuallyExclusiveCallbackGroup`), `MultiThreadedExecutor`, async patterns
- `ch3` — Closed-loop control (P, PI, PID), differential drive odometry
- `ch4` — Kalman Filter and Extended Kalman Filter implementations
- `ch5` — Services and actions (uses custom types from `ch5_interfaces`)
- `ch6` — Node lifecycle management
- `ch5_interfaces` — Custom service/action definitions (CMake package)

**Homework packages**: `hw1_pkg`, `hw2_pkg`, `hw3_pkg`, `hw4_pkg`

**Advanced projects** under `ros2_src/projects/`:
- `ekf_localization` — EKF localization with known correspondences
- `ekf_slam` — EKF-based SLAM
- `bug_planning` — Bug algorithm path planning

**`project`** — Comprehensive capstone: Gazebo simulation, camera-based line detection (OpenCV/cv_bridge), Nav2, and TurtleBot3 control.

**`utils`** — Shared RViz marker utilities and math helpers; imported by other packages.

**`preface`** — Namespace and parameter examples.

## Common Node Patterns

All nodes follow a consistent structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # publishers, subscribers, timers, parameters declared here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Entry points are registered as `console_scripts` in each package's `setup.py`.

## Interface Definitions (ch5_interfaces)

Custom services: `DriveInSquare.srv`, `Takeoff.srv`, `Land.srv`
Custom actions: `DriveSquareAction.action`, `DriveCircle.action`

These must be built before packages that depend on them:
```bash
colcon build --packages-select ch5_interfaces
```

## Simulation (project package)

The `project` package integrates Gazebo Classic with TurtleBot3 Burger + camera. It uses:
- `GAZEBO_MODEL_PATH` (set via `colcon.pkg` hooks) for model resolution
- SDF 1.6 world files in `ros2_src/project/worlds/`
- URDF/Xacro robot descriptions
- Nav2 with a pre-built map (`ros2_src/project/map/`)

**Jazzy migration note**: A migration plan exists in `plans/` targeting ROS Jazzy + Ubuntu 24.04 + Gazebo Harmonic. The repo is currently on ROS Iron. Packages tagged `iron-final` before migration.

## Development Environment

The repo is set up for VS Code devcontainers. The container image (`ghcr.io/robotics-content-lab/rclpy-from-zero-to-hero:iron-desktop-terminal-nvidia`) has ROS 2 Iron pre-installed with GPU and VNC support. X11 forwarding and VNC are available on ports 6080/6900.
