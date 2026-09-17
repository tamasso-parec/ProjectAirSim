#!/usr/bin/env python
# Project AirSim ROS 2 node bridge support package

import os
from glob import glob

from setuptools import find_packages, setup


setup(
    name="projectairsim-ros2",
    version="0.1.1",
    description="Project AirSim ROS 2 support package",
    long_description="Native ROS 2 bridge for Project AirSim",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/projectairsim_ros2"]),
        ("share/projectairsim_ros2", ["package.xml"]),
        (os.path.join("share", "projectairsim_ros2", "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", "projectairsim_ros2", "config"), glob("config/*.yaml")),
        (
            os.path.join("share", "projectairsim_ros2", "config", "px4_airframes"),
            glob("config/px4_airframes/*"),
        ),
    ],
    include_package_data=True,
    package_data={"": ["schema/*.jsonc"]},
    python_requires=">=3.7, <4",
    install_requires=["projectairsim-rosbridge"],
    # See the note in projectairsim-rosbridge/setup.py: colcon needs this to
    # run the pytest suite instead of silently running nothing.
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "projectairsim_bridge_ros2 = projectairsim_ros2.main:main",
        ],
    },
)
