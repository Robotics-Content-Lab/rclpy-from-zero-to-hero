#!/bin/bash

# This script is used to evaluate the performance of the EKF localization with unknown correspondences.

BASE_PATH="/home/ubuntu/Book/src/projects/ekf_localization/"

declare -a methods=("NN" "MLDA" "BSC" "JCBB")

for method in "${methods[@]}"; do
    cd $BASE_PATH

    # ros2 run ekf_localization ekf_localization_wukc --ros-args -p "data_association_method:=$method" &
    python3 ekf_localization/ekf_localization_wukc.py --ros-args -p "data_association_method:=$method" &
    PYTHON3PID=$!

    # Run bagfile
    ros2 bag play data/bagfile.db3

    kill $PYTHON3PID
    kill $(pgrep python3)
    kill $(pgrep ekf_localization_wukc)
    kill $(pgrep matplitlib)
done