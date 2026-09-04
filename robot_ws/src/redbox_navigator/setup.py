import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'redbox_navigator'

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
    description='TurtleBot3 Red Box Recognition and Autonomous Target Stopping Navigation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'redbox_detector = redbox_navigator.redbox_detector_node:main',
            'target_coordinate = redbox_navigator.target_coordinate_node:main',
            'target_depth_coordinate = redbox_navigator.target_depth_coordinate_node:main',
            'mission_controller = redbox_navigator.redbox_mission_controller:main',
        ],
    },
)
