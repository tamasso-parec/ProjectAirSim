# Project AirSim ROS bridge

Project AirSim includes a shared Python bridge with adapters for ROS 1 and ROS 2. The bridge connects through the Project AirSim Client API, discovers robots and sensors from the active scene, and exposes matching ROS topics and transforms. Unreal Engine continues to render the environment; ROS receives the rendered camera and simulated sensor data.

ROS 2 Humble on Ubuntu 22.04 is the primary ROS 2 target. The existing ROS 1 interface remains available for Noetic users.

## ROS 2 Humble setup

Install ROS 2 Humble and development tools, then source the installation:

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep ros-humble-radar-msgs
```

The installed bridge executable uses the same Python interpreter that runs
`colcon`. With the Ubuntu apt installation of colcon this is normally
`/usr/bin/python3`, even when an unrelated virtual environment is active.
Install the Project AirSim client for that interpreter:

```bash
sudo apt install python3-pip
/usr/bin/python3 -m pip install --user -e client/python/projectairsim
/usr/bin/python3 -c "import projectairsim; print(projectairsim.__file__)"
```

The final command must succeed before launching the bridge. If you deliberately
build the ROS packages with another Python interpreter, install the client and
all of its dependencies into that same interpreter instead. Merely activating a
virtual environment before invoking an apt-installed `colcon` does not change
the interpreter embedded in the generated console script; check it with
`head -1 install/projectairsim_ros2/lib/projectairsim_ros2/projectairsim_bridge_ros2`.

Install ROS dependencies and build both ROS 2 packages from the repository root:

```bash
rosdep update
rosdep install \
  --from-paths ros/node/projectairsim-rosbridge ros/node/projectairsim-ros2 \
  --ignore-src -r -y
colcon build \
  --base-paths ros/node/projectairsim-rosbridge ros/node/projectairsim-ros2 \
  --symlink-install
source install/setup.bash
```

The ROS libraries (`rclpy`, message packages, TF, and radar messages) are ROS/apt dependencies, not pip dependencies.

## Launching ROS 2

Start a Project AirSim Unreal environment first. Then launch the bridge:

```bash
ros2 launch projectairsim_ros2 projectairsim_bridge_ros2.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config"
```

Connection and safety settings are launch arguments and ROS parameters:

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `address` | string | `127.0.0.1` | Project AirSim server address. |
| `topics_port` | integer | `8989` | Project AirSim pub/sub port. |
| `services_port` | integer | `8990` | Project AirSim service port. |
| `sim_config_path` | string | `sim_config/` | Directory containing scene configuration files. |
| `cmd_vel_timeout_sec` | double | `1.0` | Duration of each velocity request and motion failsafe. Publish faster than this interval. |
| `takeoff_timeout_sec` | double | `20.0` | Takeoff service timeout. |
| `land_timeout_sec` | double | `60.0` | Landing service timeout. |

The node name defaults to `projectairsim` and can be changed with `node_name:=...`. Stop the bridge with Ctrl-C; it removes ROS entities, cancels pending flight operations, and disconnects its Project AirSim client.

## Moving a drone from ROS 2

Interfaces are generated under each discovered `<robot_path>`. Obtain the exact path for a scene with:

```bash
ros2 service list | grep enable_api_control
ros2 topic list | grep cmd_vel
```

Enable API control, arm, and take off before sending velocity commands. Replace `<robot_path>` with the discovered path:

```bash
ros2 service call <robot_path>/enable_api_control std_srvs/srv/SetBool '{data: true}'
ros2 service call <robot_path>/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call <robot_path>/takeoff std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 <robot_path>/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
```

Stop the publisher with Ctrl-C and land explicitly:

```bash
ros2 service call <robot_path>/land std_srvs/srv/Trigger '{}'
ros2 service call <robot_path>/arm std_srvs/srv/SetBool '{data: false}'
```

The lifecycle services return `success: false` and the Project AirSim error when a robot/controller rejects a request. Takeoff and landing services wait for completion or their configured timeout.

## Topics and services

`<robot_path>` is the full scene-specific Project AirSim robot path. `<sensor_path>` is the corresponding path through the robot's configured sensor ID.

### Subscribed by the bridge

| ROS interface | Type | Behavior |
| --- | --- | --- |
| `/ProjectAirSim/node/<node_name>/load_scene` | `std_msgs/String` | Loads a configuration from `sim_config_path`, then replaces scene-specific topics, services, and TF frames. |
| `<robot_path>/cmd_vel` | `geometry_msgs/Twist` | Requests linear velocity and yaw rate. Publish continuously before `cmd_vel_timeout_sec` expires. |
| `<robot_path>/desired_pose` | `geometry_msgs/PoseStamped` | Immediately sets the robot pose when the controller exposes a desired-pose topic. |
| `<camera_path>/desired_pose` | `geometry_msgs/PoseStamped` | Immediately changes the camera pose for all captures from that camera. |

### Lifecycle services provided by ROS 2

| Service | Type |
| --- | --- |
| `<robot_path>/enable_api_control` | `std_srvs/SetBool` |
| `<robot_path>/arm` | `std_srvs/SetBool` |
| `<robot_path>/takeoff` | `std_srvs/Trigger` |
| `<robot_path>/land` | `std_srvs/Trigger` |

### Published by the bridge

| Topic | Type |
| --- | --- |
| `<robot_path>/actual_pose` | `geometry_msgs/PoseStamped` |
| `<sensor_path>/gps` | `sensor_msgs/NavSatFix` |
| `<sensor_path>/barometer` | `sensor_msgs/FluidPressure` |
| `<sensor_path>/imu_kinematics` | `sensor_msgs/Imu` |
| `<sensor_path>/magnetometer` | `sensor_msgs/MagneticField` |
| `<sensor_path>/lidar` | `sensor_msgs/PointCloud2` |
| `<sensor_path>/radar_detections` | `radar_msgs/RadarScan` |
| `<sensor_path>/radar_tracks` | `radar_msgs/RadarTracks` |
| `<camera_path>/<image_type>/image` | `sensor_msgs/Image` |
| `<camera_path>/<image_type>/camera_info` | `sensor_msgs/CameraInfo` |

Supported camera image types are `scene_camera`, `depth_planar_camera`, `depth_camera`, `depth_vis_camera`, `segmentation_camera`, `disparity_normalized_camera`, and `surface_normals_camera`. A scene camera contains the photorealistic Unreal rendering. Camera topics are created only for captures enabled in the robot configuration.

High-rate sensor publishers use the ROS 2 sensor-data QoS profile. Camera information and other latched state use reliable, transient-local QoS. Commands and services use reliable, volatile communication.

## TF and coordinates

The bridge broadcasts each robot from the `map` parent frame and sensor frames relative to their configured parents.

Project AirSim values use NED-style axes: X forward/north, Y right/east, and Z down. The legacy ROS bridge conversion is preserved: X remains forward, while Y and Z are negated to produce ROS right-handed X-forward, Y-left, Z-up values. For a world-aligned map this is effectively a North-West-Up representation, not standard ENU. The same inverse conversion is applied to `cmd_vel` and `desired_pose` requests.

## Changing scenes

Publish a scene configuration filename relative to `sim_config_path`:

```bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_drone_sensors.jsonc}'
```

Pending takeoff or landing operations are cancelled. Interfaces and transforms belonging to the previous scene are removed before the new scene is loaded and discovered.

## Viewing rendered images

Install and run `rqt_image_view`, then select a discovered `scene_camera/image` topic:

```bash
sudo apt install ros-humble-rqt-image-view
rqt_image_view
```

Camera images are published on demand: Project AirSim streaming begins after a ROS subscriber is detected and stops when the final subscriber disconnects.

## ROS 1 compatibility

The ROS 1 adapter remains in `ros/node/projectairsim-ros1`. On a ROS Noetic system, install the shared bridge and ROS 1 Python packages, or place that directory in a Catkin workspace. ROS 1 continues to expose the established topics and conversion behavior; the lifecycle services above are ROS 2 additions.

The bridge and mission code cannot connect as separate Project AirSim clients. To combine a mission with ROS, use `ros/node/scripts/ros1/hello_ros1.py` or `ros/node/scripts/ros2/hello_ros2.py`, which share one `ProjectAirSimClient` instance.

## Validation

With ROS 2 Humble sourced, run:

```bash
colcon test --packages-select projectairsim_rosbridge projectairsim_ros2
colcon test-result --verbose
```

An end-to-end check requires a running Blocks environment. Verify lifecycle services, pose movement from `cmd_vel`, camera images and `CameraInfo`, configured IMU/lidar/radar topics, scene replacement, landing, and clean shutdown.

## Minimal waypoint navigation

The optional `projectairsim_navigation` package provides a small ROS 2 navigation
example using simulator ground truth, a smooth straight-line trajectory, and a
velocity feedback controller. It intentionally has no mapping or collision
avoidance. See [`ros/node/projectairsim-navigation/README.md`](../../ros/node/projectairsim-navigation/README.md)
for its interfaces, build command, launch instructions, and safety limitations.
The bridge must have an active scene before robot pose topics and lifecycle
services exist; use its `load_scene` topic or the navigation package's combined
launch file to load one.

---

Copyright (C) Microsoft Corporation.  
Copyright (C) 2025 IAMAI CONSULTING CORP

MIT License. All rights reserved.
