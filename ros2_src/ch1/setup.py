from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='office@roboticscontentlab.com',
    description='First ROS 2 ament-python package of the RCLPY: From Zero to Hero Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_node = ch1.first_node:main',
            'first_publisher = ch1.first_publisher:main',
            'first_subscriber = ch1.first_subscriber:main',
            'simple_publisher = ch1.simple_publisher:main',
            'simple_subscriber = ch1.simple_subscriber:main'
        ],
    },
)
