# CARKit learning annotation: implements the behavior described by this file's package and module.
from glob import glob
import os

from setuptools import setup

package_name = 'carkit_human_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARKit maintainers',
    maintainer_email='ada@todo.todo',
    description='CARKit joystick control launch',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
