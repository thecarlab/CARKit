from glob import glob
import os

from setuptools import setup

package_name = 'carkit_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'map'), glob('../map/*')),
        (os.path.join('share', package_name, 'waypoints'), glob('../waypoints/*')),
        (os.path.join('share', package_name, 'rviz'), glob('../rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARKit maintainers',
    maintainer_email='ada@todo.todo',
    description='CARKit launch, maps, waypoints, and RViz configuration',
    license='Apache-2.0',
)
