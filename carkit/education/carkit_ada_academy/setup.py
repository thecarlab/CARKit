# CARKit learning annotation: implements the behavior described by this file's package and module.
from setuptools import find_packages, setup


package_name = "carkit_ada_academy"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CAR Lab",
    maintainer_email="ada@udel.edu",
    description="Guided ADA Academy CARKit algorithms",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "planning_node = carkit_ada_academy.planning:main",
        "control_node = carkit_ada_academy.control:main",
        "perception_node = carkit_ada_academy.perception:main",
    ]},
)
