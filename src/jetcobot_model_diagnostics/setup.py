from glob import glob

from setuptools import find_packages, setup


package_name = 'jetcobot_model_diagnostics'


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
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jio',
    maintainer_email='jio@todo.todo',
    description=(
        'Compare JetCobot get_coords with an external official URDF'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'official_urdf_coords_test = '
            'jetcobot_model_diagnostics.official_urdf_coords_test:main',
        ],
    },
)
