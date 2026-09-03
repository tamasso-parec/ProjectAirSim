#!/usr/bin/env python3
import os
from glob import glob
from setuptools import find_packages, setup

setup(
    name="projectairsim-navigation",
    version="0.1.0",
    description="Minimal ground-truth navigation stack for Project AirSim",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/projectairsim_navigation"]),
        ("share/projectairsim_navigation", ["package.xml"]),
        (os.path.join("share", "projectairsim_navigation", "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", "projectairsim_navigation", "config"), glob("config/*.yaml")),
    ],
    entry_points={
        "console_scripts": [
            "ground_truth_adapter = projectairsim_navigation.nodes:ground_truth_main",
            "simple_navigator = projectairsim_navigation.nodes:navigator_main",
            "scene_loader = projectairsim_navigation.scene_loader:main",
            "waypoint = projectairsim_navigation.waypoint:main",
        ]
    },
)
