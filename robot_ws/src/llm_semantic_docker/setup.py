import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'llm_semantic_docker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='shouttt1320@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_agent_node = llm_semantic_docker.llm_agent_node:main',
            'task_manager_node = llm_semantic_docker.task_manager:main',
            'object_detector_3d_node = llm_semantic_docker.object_detector_3d:main',
            'qwen_semantic_gui_node = llm_semantic_docker.qwen_semantic_gui_node:main',
            'precision_docking_controller_node = llm_semantic_docker.precision_docking_controller_node:main',
            'interactive_docking_teacher_node = llm_semantic_docker.interactive_docking_teacher:main',
            'standalone_map_server = llm_semantic_docker.standalone_map_server:main',
        ],
    },
)
