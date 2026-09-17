#!/usr/bin/env python
# Project AirSim ROS node bridge package

from setuptools import find_packages, setup


setup(
    name="projectairsim-rosbridge",
    version="0.1.1",
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
        "pyyaml",
    ],
    # colcon only selects its pytest test step for packages that declare a
    # test dependency on pytest; without this it falls back to the
    # unittest-based "setup.py test" and collects nothing.  The "test" extra
    # is used rather than the deprecated tests_require, which current
    # setuptools drops from the metadata colcon reads.
    extras_require={"test": ["pytest"]},
)
