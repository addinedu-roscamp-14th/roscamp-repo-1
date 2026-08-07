"""Setuptools configuration for arm_homography_pick_place."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'arm_homography_pick_place'

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
        ('share/' + package_name + '/config', glob('config/*')),
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
            'arm_homography_pick_place.gated_aruco:main',
            'homography_pick_place = '
            'arm_homography_pick_place.coordinator:main',
        ],
    },
)
