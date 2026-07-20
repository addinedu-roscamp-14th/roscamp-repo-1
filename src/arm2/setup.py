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
            'arm2_hardware_joint_state_publisher = '
            'arm2.hardware_joint_state_publisher:main',
            'arm2_manual_jog = arm2.manual_jog:main',
            'arm2_aruco_pose_publisher = arm2.aruco_pose_publisher:main',
            'arm2_charuco_pose_publisher = '
            'arm2.arm2_charuco_pose_publisher:main',
            'arm2_generate_charuco_board = '
            'arm2.arm2_charuco_board_generator:main',
            'arm2_container_pick_coordinator = '
            'arm2.container_pick_coordinator:main',
            'arm2_jetcobot_trajectory_bridge = '
            'arm2.jetcobot_trajectory_bridge:main',
            'arm2_camera_device_supervisor = '
            'arm2.camera_device_supervisor:main',
            'arm2_teach_container_grasp = '
            'arm2.teach_container_grasp:main',
            'arm2_grasp_teach_jog = arm2.grasp_teach_jog:main',
            'arm2_teach_startup_pose = arm2.teach_startup_pose:main',
        ],
    },
)
