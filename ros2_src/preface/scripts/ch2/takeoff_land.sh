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


#------------------#
#------ Main ------#
#------------------#
main() {
  # Start takeoff and landing sequence
  wait_for_topics
  sleep 5
  takeoff_drone
  land_drone
}

#-----------------------#
# Entry point of script #
#-----------------------#
main
sleep 5  # Wait 5 seconds before exiting and calling the cleanup function