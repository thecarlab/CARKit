# CARKit learning annotation: implements the behavior described by this file's package and module.
from setuptools import find_packages, setup


package_name = "carkit_student_algorithms"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CAR Lab",
    maintainer_email="ada@udel.edu",
    description="Student-owned CARKit algorithm implementations",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "guided_planning = carkit_student_algorithms.planning:guided_main",
        "boilerplate_planning = carkit_student_algorithms.planning:boilerplate_main",
        "guided_control = carkit_student_algorithms.control:guided_main",
        "boilerplate_control = carkit_student_algorithms.control:boilerplate_main",
        "guided_perception = carkit_student_algorithms.perception:guided_main",
        "boilerplate_perception = carkit_student_algorithms.perception:boilerplate_main",
    ]},
)
