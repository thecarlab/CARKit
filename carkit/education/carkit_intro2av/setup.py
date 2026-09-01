# CARKit learning annotation: implements the behavior described by this file's package and module.
from glob import glob
import os

from setuptools import find_packages, setup


package_name = "carkit_intro2av"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CAR Lab",
    maintainer_email="ada@udel.edu",
    description="Intro2AV ROS 2 algorithm boilerplates",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "planning_node = carkit_intro2av.planning:main",
        "control_node = carkit_intro2av.control:main",
        "perception_node = carkit_intro2av.perception:main",
    ]},
)
