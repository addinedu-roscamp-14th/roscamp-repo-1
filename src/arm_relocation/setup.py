from glob import glob

from setuptools import find_packages, setup


package_name = 'arm_relocation'

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
    description='Multi-layer ArUco container relocation for JetCobot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'relocation_aruco_pose_publisher = '
            'arm_relocation.relocation_aruco_pose_publisher:main',
            'container_pick_place_relocation = '
            'arm_relocation.container_pick_place_relocation:main',
        ],
    },
)
