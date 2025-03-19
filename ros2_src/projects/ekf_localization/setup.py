from setuptools import setup
import os
import glob

package_name = 'ekf_localization'
modules = f'{package_name}/utils'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

extra_files = package_files('models/')
extra_files2 = package_files('bags/')
data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        (('share/' + package_name + '/launch'), glob.glob('launch/*launch.[pxy][yma]*'))
    ]
data_files.extend(('share/' + package_name + '/' + os.path.dirname(file), [file]) for file in extra_files)
data_files.extend(('share/' + package_name + '/' + os.path.dirname(file), [file]) for file in extra_files2)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, modules],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='georg.novtony@aon.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmark_spawner = ekf_localization.landmark_spawner:main',
            'ekf_localization_wukc = ekf_localization.ekf_localization_wukc:main',
            'ekf_localization_with_known_correspondences = ekf_localization.ekf_localization_with_known_correspondences:main',
            'ekf_localization_with_unknown_correspondences = ekf_localization.ekf_localization_with_unknown_correspondences:main',
            'ekf_localization_with_unknown_correspondences_mht = ekf_localization.ekf_localization_with_unknown_correspondences_mht:main'
        ],
    },
)
