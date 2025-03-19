"""This script spawns landmarks in the gazebo world. Landmarks are loaded from a yaml file and spawned in gazebo using the model.sdf file. """

import os

import rclpy
from ament_index_python.packages import get_package_share_directory

from ekf_localization.utils.gazebo import load_landmarks, spawn_landmarks

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('utils')

    # Load landmarks from file
    package_path_ekf_localization = get_package_share_directory('ekf_localization') 
    print(f"{package_path_ekf_localization}")
    landmarks = load_landmarks(os.path.join(package_path_ekf_localization, 'config', 'landmarks.yaml'))
    print(f"{len(landmarks)}")
    # Load SDF model file
    model_xml = open(os.path.join(package_path_ekf_localization, 'models', 'Landmark', 'model.sdf'), 'r').read()

    # Spawn landmarks in Gazebo
    spawn_landmarks(node, landmarks, model_xml)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
