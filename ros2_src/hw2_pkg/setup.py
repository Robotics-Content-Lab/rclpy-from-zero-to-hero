from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hw2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='office@roboticscontentlab.com',
    description='Second ROS 2 homework assignment of the RCLPY: From Zero to Hero Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_proc = hw2_pkg.sensor_proc:main',
        ],
    },
)
