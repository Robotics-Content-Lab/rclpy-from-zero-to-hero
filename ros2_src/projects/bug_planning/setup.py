import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'bug_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "config"), glob('config/*.[rviz]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='georg.novtony@aon.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bug2_planner = bug_planning.bug2_planner:main",
        ],
    },
)
