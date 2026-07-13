from glob import glob
import os

from setuptools import setup


package_name = 'vi_grab'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='YOLOv8 visual grasp demo for RM65 on ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_detector = vi_grab.object_detector_node:main',
            'grasp_coordinator = vi_grab.grasp_coordinator_node:main',
        ],
    },
)
