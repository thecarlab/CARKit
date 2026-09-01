# CARKit learning annotation: implements the behavior described by this file's package and module.
from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'osracer_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'camera_info'),
         glob('config/camera_info/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARKit maintainers',
    maintainer_email='ada@todo.todo',
    description='CARKit-compatible OSRacer chassis, sensor, and joystick bringup.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joystick_teleop = osracer_bringup.joystick_teleop:main',
            'command_relay = osracer_bringup.command_relay:main',
        ],
    },
)
