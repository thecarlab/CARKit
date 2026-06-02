from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'carkit_amcl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARKit maintainers',
    maintainer_email='ada@todo.todo',
    description='CARKit Nav2 AMCL localization and navigation workflow',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_ackermann = carkit_amcl.twist_to_ackermann:main',
            'odom_tf_broadcaster = carkit_amcl.odom_tf_broadcaster:main',
        ],
    },
)
