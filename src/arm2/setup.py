from glob import glob

from setuptools import find_packages, setup

package_name = 'arm2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='Second JetCobot ArUco and MoveIt container picking package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm2_hardware_joint_state_publisher = '
            'arm2.arm2_hardware_joint_state_publisher:main',
            'arm2_manual_jog = arm2.arm2_manual_jog:main',
            'arm2_aruco_pose_publisher = '
            'arm2.arm2_aruco_pose_publisher:main',
            'arm2_charuco_pose_publisher = '
            'arm2.arm2_charuco_pose_publisher:main',
            'arm2_generate_charuco_board = '
            'arm2.arm2_charuco_board_generator:main',
            'arm2_container_pick_coordinator = '
            'arm2.arm2_container_pick_coordinator:main',
            'arm2_grasp_offset_calibrator = '
            'arm2.arm2_grasp_offset_calibrator:main',
            'arm2_jetcobot_trajectory_bridge = '
            'arm2.arm2_jetcobot_trajectory_bridge:main',
            'arm2_auto_handeye_sampler = '
            'arm2.arm2_auto_handeye_sampler:main',
            'arm2_camera_repeatability_monitor = '
            'arm2.arm2_camera_repeatability_monitor:main',
        ],
    },
)
