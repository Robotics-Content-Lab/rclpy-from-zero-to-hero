from setuptools import setup

package_name = 'utils'
modules = f'{package_name}/utils'


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, modules],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georg Novotny',
    maintainer_email='office@roboticscontentlab.com',
    description='Utility functions for the RCLPY: From Zero to Hero Book',
    license="Apache License 2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
