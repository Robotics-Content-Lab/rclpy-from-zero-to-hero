from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='office@roboticscontentlab.com',
    description='Fifth ROS 2 ament-python package of the RCLPY: From Zero to Hero Book',
    license="Apache License 2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_service_server = ch5.holonomic_closed_loop_square_service_server:main',
            'square_service_client = ch5.holonomic_closed_loop_square_service_client:main',
            'square_action_server = ch5.holonomic_closed_loop_square_action_server:main',
            'square_action_client = ch5.holonomic_closed_loop_square_action_client:main',
            'drone_mission_node_server = ch5.drone_mission_node_server:main',
            'drone_mission_node = ch5.drone_mission_node:main'
        ],
    },
)
