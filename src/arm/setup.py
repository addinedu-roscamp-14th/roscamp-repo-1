"""Setuptools configuration for arm_pick_place."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'arm_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml', 'README.md', 'FLOOR_CALIBRATION.md'],
        ),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='Homography-corrected direct JetCobot pick and place',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gated_pick_place_aruco = '
            'arm_pick_place.gated_aruco:main',
            'container_pick_place = '
            'arm_pick_place.coordinator:main',
            'calibration_aruco_pose_publisher = '
            'arm_pick_place.calibration_aruco_pose_publisher:main',
            'floor_calibrator = '
            'arm_floor_calibration.floor_calibrator:main',
            'manual_jog = arm_pick_place.manual_jog:main',
        ],
    },
)
