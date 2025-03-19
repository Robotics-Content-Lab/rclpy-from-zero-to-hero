from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch3'

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
    description='Third ROS 2 ament-python package of the RCLPY: From Zero to Hero Book',
    license="Apache License 2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_odometry = ch3.diff_drive_odometry:main',
            'open_loop = ch3.open_loop:main',
            'closed_loop_p = ch3.closed_loop_p:main',
            'closed_loop_pid = ch3.closed_loop_pid:main',
            'open_loop_square = ch3.open_loop_square:main',
            'closed_loop_p_square = ch3.closed_loop_p_square:main'
        ],
    },
)
