"""Setuptools configuration."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'arm_simple_pick_place'

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
    description='Simple dual-backend JetCobot pick and place',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gated_dual_aruco_pose_publisher = '
            'arm_simple_pick_place.gated_aruco_pose_publisher:main',
            'simple_pick_place = '
            'arm_simple_pick_place.simple_pick_place:main',
        ],
    },
)
