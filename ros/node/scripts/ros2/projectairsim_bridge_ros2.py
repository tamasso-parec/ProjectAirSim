#!/usr/bin/env python
"""
Copyright (C) Microsoft Corporation. 
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

ROS bridge for Project AirSim.  This example illustrates the basic setup of
the Project AirSim ROS Bridge node for ROS2.  Here no client script is run at
all so all interaction with the simulation must be done through the ROS
topics.  For an example that runs a mission client script in addition to the
bridge node, see "hello_ros2.py".

From a Project AirSim virtual Python environment, run this script and specify:
1. The IP address of the Project AirSim server with the "--ipaddress" flag if
   the server is not running on the local machine, and
2. The path to the simulation configuration files with the "--simconfigpath" flag
   (e.g., "../../../client/python/example_user_scripts/sim_config".)
"""

from projectairsim_ros2.main import main


if __name__ == "__main__":
    main()
