from glob import glob

from setuptools import find_packages, setup

package_name = 'arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='ArUco and MoveIt based container picking for JetCobot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardware_joint_state_publisher = '
            'arm.hardware_joint_state_publisher:main',
            'manual_jog = arm.manual_jog:main',
            'aruco_pose_publisher = arm.aruco_pose_publisher:main',
            'charuco_pose_publisher = '
            'arm.charuco_pose_publisher:main',
            'generate_charuco_board = '
            'arm.charuco_board_generator:main',
            'container_pick_coordinator = '
            'arm.container_pick_coordinator:main',
            'jetcobot_trajectory_bridge = '
            'arm.jetcobot_trajectory_bridge:main',
        ],
    },
)
