from glob import glob
import os

from setuptools import find_packages, setup

package_name = "carkit_behavior"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CARKit maintainers",
    maintainer_email="ada@todo.todo",
    description="CARKit behavior layer",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "behavior_center_node = carkit_behavior.behavior_center_node:main",
        ],
    },
)
