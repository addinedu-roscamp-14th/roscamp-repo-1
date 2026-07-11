from glob import glob
from setuptools import find_packages, setup


package_name = 'slam'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description='SLAM Toolbox mapping package for Pinky',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
