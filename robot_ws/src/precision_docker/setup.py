import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'precision_docker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='jjj@todo.todo',
    description='High-precision Autonomous Marker & Depth Docking Package for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'precision_approacher = precision_docker.precision_approacher_node:main',
            'docking_gui = precision_docker.docking_gui_node:main',
            'bumper_sensor = precision_docker.bumper_sensor_node:main',
        ],
    },
)
