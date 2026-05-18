from setuptools import find_packages, setup

package_name = 'carkit_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARKit maintainers',
    maintainer_email='ada@todo.todo',
    description='CARKit behavior nodes for educational driving demos',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_sign_behavior_node = carkit_behaviors.stop_sign_behavior_node:main',
        ],
    },
)
