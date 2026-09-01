# CARKit learning annotation: implements the behavior described by this file's package and module.
from glob import glob
import os

from setuptools import find_packages, setup


package_name = "carkit_webui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "static"), glob("static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CAR Lab",
    maintainer_email="ada@udel.edu",
    description="CARKit browser UI",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "carkit-webui = carkit_webui.server:main",
    ]},
)
