#!/usr/bin/env python
# Project AirSim ROS node bridge package

from setuptools import find_packages, setup


setup(
    name="projectairsim-rosbridge",
    version="{# include "client_version.txt" #}",
    description="Project AirSim ROS bridge core package",
    long_description="To be populated from a README.md",  # TODO Populate from a README.md
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/projectairsim_rosbridge"]),
        ("share/projectairsim_rosbridge", ["package.xml"]),
    ],
    include_package_data=True,
    package_data={"": ["schema/*.jsonc"]},
    python_requires=">=3.7, <4",
    install_requires=[
        "projectairsim",
        "numpy",
    ],
)
