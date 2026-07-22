from setuptools import find_packages, setup

package_name = 'arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='Camera-click pick and place control for JetCobot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
<<<<<<< Updated upstream
            'click_pick_place = arm.main:main',
=======
<<<<<<< Updated upstream
            'hardware_joint_state_publisher = '
            'arm.hardware_joint_state_publisher:main',
            'manual_jog = arm.manual_jog:main',
=======
<<<<<<< Updated upstream
            'click_pick_place = arm.main:main',
=======
            'hardware_joint_state_publisher = '
            'arm.hardware_joint_state_publisher:main',
            'manual_jog = arm.manual_jog:main',
            'charuco_handeye_test = arm.charuco_handeye_test:main',
>>>>>>> Stashed changes
            'aruco_pose_publisher = arm.aruco_pose_publisher:main',
            'charuco_pose_publisher = '
            'arm.charuco_pose_publisher:main',
            'generate_charuco_board = '
            'arm.charuco_board_generator:main',
            'container_pick_coordinator = '
            'arm.container_pick_coordinator:main',
            'jetcobot_trajectory_bridge = '
            'arm.jetcobot_trajectory_bridge:main',
<<<<<<< Updated upstream
=======
>>>>>>> Stashed changes
>>>>>>> Stashed changes
>>>>>>> Stashed changes
        ],
    },
)
