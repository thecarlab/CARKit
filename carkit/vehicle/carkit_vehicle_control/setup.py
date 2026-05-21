from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'carkit_vehicle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    description='CARKit vehicle command launch and utility nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_ackermann = carkit_vehicle_control.keyboard_ackermann:main',
        ],
    },
)
