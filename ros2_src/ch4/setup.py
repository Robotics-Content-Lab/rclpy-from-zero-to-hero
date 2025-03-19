from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch4'


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

data_files=[
    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]
extra_files = package_files('bag/')
data_files.extend(('share/' + package_name + '/' + os.path.dirname(file), [file]) for file in extra_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='office@roboticscontentlab.com',
    description='Kalman Filter ROS 2 ament-python package of the RCLPY: From Zero to Hero Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_comparison = ch4.odometry_comparison:main',
            'kf = ch4.kf:main',
            'ekf = ch4.ekf:main',
            'kf2 = ch4.kf2:main',
            'ekf2 = ch4.ekf2:main'
        ],
    },
)
