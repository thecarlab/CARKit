# CARKit learning annotation: implements the behavior described by this file's package and module.
from glob import glob
from setuptools import find_packages, setup

package_name = 'osracer_base'

setup(
    name=package_name,
    version='0.3.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md', 'README_zh.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/docs', glob('docs/*.md')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/scripts', glob('scripts/*')),
        (f'share/{package_name}/udev', glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osrbot',
    maintainer_email='winter@osrbot.com',
    description='ROS 2 chassis driver for modern and legacy OSRacer controllers.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'chassis_driver = osracer_base.chassis_driver:main',
            'check_device = osracer_base.tools.check_device:main',
            'install_udev_rules = osracer_base.tools.install_udev_rules:main',
        ],
    },
)
