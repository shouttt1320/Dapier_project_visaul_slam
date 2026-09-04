import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_visual_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle_pi_os30a'), glob('models/turtlebot3_waffle_pi_os30a/*')),
        (os.path.join('share', package_name, 'materials', 'textures'), glob('materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='jjj@todo.todo',
    description='TurtleBot3 Visual-Inertial SLAM Package with Orbbec Astra S and OpenCR IMU',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'compressed_bridge_node = turtlebot3_visual_slam.compressed_bridge_node:main',
        ],
    },
)
