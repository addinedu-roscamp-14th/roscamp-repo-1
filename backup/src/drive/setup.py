from glob import glob
from setuptools import find_packages, setup

package_name = 'drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/pinky/launch', glob('pinky/launch/*.xml')),
        ('share/' + package_name + '/pinky/params', glob('pinky/params/*.yaml')),
        ('share/' + package_name + '/pinky/behavior_trees', glob('pinky/behavior_trees/*.xml')),
        ('share/' + package_name + '/pinky/rviz', glob('pinky/rviz/*.rviz')),
        ('share/' + package_name + '/pinky/map', glob('pinky/map/*')),
        ('share/' + package_name + '/pinky/scripts', glob('pinky/scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='Drive and Nav2 bridge package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_map_pose_to_nav_goal = drive.target_map_pose_to_nav_goal:main',
            'send_nav_goal = drive.send_nav_goal:main',
            'pinky_nav2_web_server = drive.pinky_nav2_web_server:main',
        ],
    },
)
