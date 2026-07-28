from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'central'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='Central control package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_to_map_bridge = central.camera_to_map_bridge:main',
            'control_gateway = central.control_gateway:main',
            'fleet_dispatcher = central.fleet_dispatcher:main',
            'rqt_click_to_target = central.rqt_click_to_target:main',
        ],
    },
)
