#!/bin/bash

# Generate prefix from 4 levels up from env-hook directory (colcon ws install folder)
AMENT_CURRENT_PREFIX=$(ros2 pkg prefix project)

# Use ament functionality to prepend Gazebo Harmonic env var with custom package resources
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/project/models"
