from glob import glob
import os

from setuptools import find_packages, setup


package_name = "carkit_behavior"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CARKit maintainers",
    maintainer_email="ada@todo.todo",
    description="Traffic-light and stop-sign behavior overrides for CARKit.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "road_rule_behavior = "
            "carkit_behavior.road_rule_behavior_node:main",
        ],
    },
)
