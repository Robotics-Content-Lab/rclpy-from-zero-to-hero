from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch2'

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
    description='Second ROS 2 ament-python package of the RCLPY: From Zero to Hero Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blocking = ch2.blocking_code:main',
            'qos_publisher1 = ch2.qos_publisher1:main',
            'qos_subscriber1 = ch2.qos_subscriber1:main',
            'qos_subscriber2 = ch2.qos_subscriber2:main',
            'mutually_exclusive = ch2.mutually_exclusive:main',
            'concurrent = ch2.concurrent:main',
            'long_running_operation = ch2.long_running_operation:main',
            'manual_spin = ch2.manual_spin:main',
            'async_spin = ch2.async_spin:main',
            'qos_publisher = ch2.qos_publisher:main',
            'qos_subscriber = ch2.qos_subscriber:main'
        ],
    },
)
