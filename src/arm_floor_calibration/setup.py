"""Setuptools configuration for arm_floor_calibration."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'arm_floor_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='Teach floor homographies and pick/place Z values',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'floor_calibrator = '
            'arm_floor_calibration.floor_calibrator:main',
        ],
    },
)

