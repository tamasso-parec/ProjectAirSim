"""Installed ROS 2 entry point for the Project AirSim bridge."""

from typing import Optional, Sequence

import rclpy

from projectairsim.utils import projectairsim_log
from projectairsim_rosbridge import ProjectAirSimROSBridge

from .ros2_node import ROS2Node


def _declare_parameters(ros_node: ROS2Node):
    node = ros_node.native_node
    node.declare_parameter("address", "127.0.0.1")
    node.declare_parameter("topics_port", 8989)
    node.declare_parameter("services_port", 8990)
    node.declare_parameter("sim_config_path", "sim_config/")
    node.declare_parameter("cmd_vel_timeout_sec", 1.0)
    node.declare_parameter("takeoff_timeout_sec", 20.0)
    node.declare_parameter("land_timeout_sec", 60.0)


def _parameter(ros_node: ROS2Node, name: str):
    return ros_node.native_node.get_parameter(name).value


def main(args: Optional[Sequence[str]] = None):
    """Initialize ROS, connect the bridge, and spin until shutdown."""
    rclpy.init(args=args)
    ros_node = ROS2Node(name="projectairsim")
    bridge = None

    try:
        _declare_parameters(ros_node)
        bridge = ProjectAirSimROSBridge(
            ros_node=ros_node,
            address=str(_parameter(ros_node, "address")),
            port_topics=int(_parameter(ros_node, "topics_port")),
            port_services=int(_parameter(ros_node, "services_port")),
            sim_config_path=str(_parameter(ros_node, "sim_config_path")),
            cmd_vel_timeout_sec=float(
                _parameter(ros_node, "cmd_vel_timeout_sec")
            ),
            takeoff_timeout_sec=float(
                _parameter(ros_node, "takeoff_timeout_sec")
            ),
            land_timeout_sec=float(_parameter(ros_node, "land_timeout_sec")),
        )
        projectairsim_log().info("Project AirSim ROS 2 bridge ready")
        ros_node.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        projectairsim_log().exception("Project AirSim ROS 2 bridge failed")
        raise
    finally:
        if bridge is not None:
            bridge.clear()
        ros_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
