from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hw1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='office@roboticscontentlab.com',
    description='First ROS 2 homework assignment of the RCLPY: From Zero to Hero Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = hw1_pkg.publisher_node:main',
            'subscriber_node = hw1_pkg.subscriber_node:main'
        ],
    },
)
