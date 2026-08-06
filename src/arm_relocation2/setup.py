"""Setuptools configuration for arm_relocation2."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'arm_relocation2'

setup(
    name=package_name,
    version='0.2.0',
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
    description='Declared-stack ArUco relocation for JetCobot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stack_aruco_pose_publisher = '
            'arm_relocation2.stack_aruco_pose_publisher:main',
            'stack_pick_place_coordinator = '
            'arm_relocation2.stack_pick_place_coordinator:main',
        ],
    },
)
