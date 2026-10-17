# CARKit learning annotation: implements the behavior described by this file's package and module.
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'carkit_human_control'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='CARKit maintainers',
    maintainer_email='ada@todo.todo',
    description='CARKit joystick control launch',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'joystick_teleop = carkit_human_control.joystick_teleop:main',
        'command_relay = carkit_human_control.command_relay:main',
    ]},
)
