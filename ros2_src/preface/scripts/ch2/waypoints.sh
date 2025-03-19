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
# Change the mode to position control
change_mode(){
    echo "Changing mode to position control..."
    ros2 topic pub --once "$DRONE_NAMESPACE/posctrl" std_msgs/msg/Bool "{data: true}"
}

# Function to command drone takeoff
takeoff_drone() {
  echo "Drone taking off..."
  ros2 topic pub --once "$DRONE_TAKEOFF_TOPIC" std_msgs/msg/Empty "{}"
  ros2 topic pub --once "$DRONE_TAKEOFF_TOPIC" std_msgs/msg/Empty "{}"
  sleep 1
}

# Function to command drone landing
land_drone() {
  echo "Drone landing..."
  ros2 topic pub --once "$DRONE_LAND_TOPIC" std_msgs/msg/Empty "{}"
  ros2 topic pub --once "$DRONE_LAND_TOPIC" std_msgs/msg/Empty "{}"
  sleep 1
}

# Method to command drone to hover (no movement)
hover() {
  echo "Drone hovering..."
  ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  sleep 1

}
# Method to command the drone to move with given linear x and y velocities
move_to(){
    local x_pos=${1:-"0.0"}
    local y_pos=${2:-"0.0"}
    local z_pos=${3:-"0.0"}
    echo "Drone moving to x: $x_pos, linear y: $y_pos..."
    ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist "{linear: {x: $x_pos, y: $y_pos, z: $z_pos}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ros2 topic pub --once "$DRONE_CMDVEL_TOPIC" geometry_msgs/msg/Twist "{linear: {x: $x_pos, y: $y_pos, z: $z_pos}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sleep 5
}


main() {
    waypoints=(
        "3.0 0.0 2.0"
        "0.0 1.0 3.0"
        "-1.0 3.0 4.0"
    )
    wait_for_topics
    sleep 5
    takeoff_drone
    change_mode
    i=0
    for waypoint in "${waypoints[@]}"; do
        echo "Navigating to waypoint $i..."
        move_to $waypoint
        i=$((i+1))
        sleep 3
    done
    land_drone
}

main
sleep 5