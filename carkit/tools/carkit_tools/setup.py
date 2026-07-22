from glob import glob
import os

from setuptools import setup, find_packages
package_name = 'carkit_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ada',
    maintainer_email='ada@todo.todo',
    description='CARKit educational utility and demo nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_waypoints = carkit_tools.interactive_waypoints:main',
            'demo1 = carkit_tools.demo1:main',
            'demo2 = carkit_tools.demo2:main',
            'distance_metrics = carkit_tools.distance_metrics:main',
            'object_angle = carkit_tools.object_angle:main',
            'object_position = carkit_tools.object_position:main',
            'path_tracker = carkit_tools.path_tracker:main',
            'cmd_vel_to_ackermann = carkit_tools.cmd_vel_to_ackermann:main',
        ],
    },
) 
