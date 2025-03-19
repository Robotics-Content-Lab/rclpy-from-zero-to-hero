from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch6'

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
    description='Sixt ROS 2 ament-python package of the RCLPY: From Zero to Hero Book',
    license="Apache License 2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_lifecycle = ch6.drone_lifecycle:main',
            'drone_lifecycle2 = ch6.drone_lifecycle2:main'
        ],
    },
)
