from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'project'


def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

extra_files = package_files('models/')
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name + '/hook', ['hook/hook_gazebo.sh']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*/*.world')),
    ('share/' + package_name + '/urdf', glob('urdf/*')),
    ('share/' + package_name + '/map', glob('map/*')),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name, ['package.xml']),
]
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robot = project.spawn_robot:main',
            'parameter_node = project.parameter_node:main',
            'detection = project.detection:main',
            'controller = project.controller:main',
        ],
    },
)
