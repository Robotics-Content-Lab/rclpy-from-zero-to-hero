#!/bin/bash

#----------------#
# Initialization #
#----------------#
# Get name of folder in which this script is present
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source drone environment
source "${SCRIPT_PATH}"/.drone_rc

echo "Drone Namespace: $DRONE_NAMESPACE"
echo "Drone takeoff topic: $DRONE_TAKEOFF_TOPIC"
echo "Drone land topic: $DRONE_LAND_TOPIC"
echo "Cmd_vel topic: $DRONE_CMDVEL_TOPIC"


#------------------#
# Helper Functions #
#------------------#
# Function to command drone takeoff
takeoff_drone() {
  echo "Drone taking off..."
  ros2 topic pub --once "$DRONE_TAKEOFF_TOPIC" std_msgs/msg/Empty "{}"
  ros2 topic pub --once "$DRONE_TAKEOFF_TOPIC" std_msgs/msg/Empty "{}"
  sleep 5
}

# Function to command drone landing
land_drone() {
  echo "Drone landing..."
  ros2 topic pub --once "$DRONE_LAND_TOPIC" std_msgs/msg/Empty "{}"
  ros2 topic pub --once "$DRONE_LAND_TOPIC" std_msgs/msg/Empty "{}"
  sleep 5
}

# Method to command drone to hover (no movement)
hover() {
  echo "Drone hovering..."
  ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  sleep 5

}

# Method to command the drone to move with given linear x and y velocities
move() {
  local linear_x=${1:-"0.0"}
  local linear_y=${2:-"0.0"}
  echo "Drone moving with linear x: $linear_x, linear y: $linear_y..."
  ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist "{linear: {x: $linear_x, y: $linear_y, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist "{linear: {x: $linear_x, y: $linear_y, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  sleep 5
}


main() {
    wait_for_topics
    sleep 5
    takeoff_drone
    move "0.3" "0.1"
    hover
    move "-0.2" "-0.1"
    hover
    land_drone
}

main
sleep 5