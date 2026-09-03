# Minimal Project AirSim navigation

This ROS 2 Humble package is a deliberately small navigation stack. It uses the
simulator pose as ground truth, converts the bridge's NWU map convention to ENU,
creates a smooth straight-line trajectory for each waypoint, and tracks it with
velocity feed-forward plus position/yaw feedback. It has no estimator, map, or
obstacle avoidance; waypoints must be placed in known free space.

Interfaces:

- Input ground truth: `<robot_path>/actual_pose` (`geometry_msgs/PoseStamped`)
- Input waypoint: `/navigation/goal` (`geometry_msgs/PoseStamped`, ENU)
- Output state: `/navigation/odom` (`nav_msgs/Odometry`, ENU)
- Output visualization: `/navigation/trajectory` (`nav_msgs/Path`, ENU)
- Output control: `<robot_path>/cmd_vel` (`geometry_msgs/Twist`, converted back to NWU)

The ground-truth subscription uses ROS 2's sensor-data QoS (`BEST_EFFORT`) to
match the Project AirSim bridge. A default reliable subscription is not
compatible with that publisher even though the topic still appears in
`ros2 topic list`.

## Reproducible quick start

### 1. Build once

From the repository root:

```bash
cd /home/tom/Documents/ProjectAirSim
source /opt/ros/humble/setup.bash
colcon build --base-paths \
  ros/node/projectairsim-rosbridge \
  ros/node/projectairsim-ros2 \
  ros/node/projectairsim-navigation --symlink-install
source install/setup.bash
```

Rebuild `projectairsim_rosbridge` and `projectairsim_navigation` after changing
their source. Source `install/setup.bash` in every new ROS terminal.

### 2. Terminal 1: start Blocks

For a visible Unreal window:

```bash
cd /home/tom/Documents/ProjectAirSim
./packages/Blocks/Development/Linux/Blocks.sh -log
```

Use `-RenderOffScreen` only when an Unreal viewport is not required. Camera
sensors remain available in that mode.

### 3. Terminal 2: start the bridge, scene, and navigation

```bash
cd /home/tom/Documents/ProjectAirSim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch projectairsim_navigation simulation_navigation.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  scene_config:=scene_basic_drone.jsonc \
  robot_path:=/Sim/SceneBasicDrone/robots/Drone1 \
  takeoff_timeout_sec:=30.0
```

This combined launch starts the bridge, asks it to load the scene, and starts
the ground-truth adapter and navigator. Wait for all of these messages:

```text
Successfully load scene config file: scene_basic_drone.jsonc
Ground-truth pose received; publishing ENU odometry
ENU odometry received; navigator is ready for waypoints
```

### 4. Terminal 3: prepare and fly the drone

Source ROS and define the robot path:

```bash
cd /home/tom/Documents/ProjectAirSim
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROBOT_PATH=/Sim/SceneBasicDrone/robots/Drone1
```

Enable API control, arm, and take off in this order. Do not send a waypoint
until takeoff returns; every response must contain `success: true`.

```bash
ros2 service call "$ROBOT_PATH/enable_api_control" \
  std_srvs/srv/SetBool '{data: true}'

ros2 service call "$ROBOT_PATH/arm" \
  std_srvs/srv/SetBool '{data: true}'

ros2 service call "$ROBOT_PATH/takeoff" \
  std_srvs/srv/Trigger '{}'
```

Send an absolute ENU waypoint in metres, using only a known collision-free
straight line. Yaw is expressed in degrees:

```bash
ros2 run projectairsim_navigation waypoint 3.0 0.0 2.0 --yaw 0
```

Additional waypoint commands replace the currently tracked trajectory.

### 5. Land and shut down

```bash
ros2 service call "$ROBOT_PATH/land" std_srvs/srv/Trigger '{}'
ros2 service call "$ROBOT_PATH/arm" std_srvs/srv/SetBool '{data: false}'
ros2 service call "$ROBOT_PATH/enable_api_control" \
  std_srvs/srv/SetBool '{data: false}'
```

Stop the launch processes with Ctrl-C after landing.

## Separate launch workflow

If the bridge is launched separately, it does not load a scene automatically.
Load one before starting navigation:

```bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene \
  std_msgs/msg/String '{data: scene_basic_drone.jsonc}'

ros2 topic list | grep /actual_pose

ros2 launch projectairsim_navigation navigation.launch.py \
  robot_path:=/Sim/SceneBasicDrone/robots/Drone1
```

The controller publishes at 50 Hz, limits speed to 4 m/s and yaw rate to 60
degrees/s, and commands zero velocity if ground truth becomes older than 0.2
seconds during an active waypoint. It does not publish velocity commands before
a waypoint or after its final stop, so it cannot override takeoff or landing.

## Visibility and diagnostics

To retain off-screen rendering and view a ROS camera stream, run
`rqt_image_view` and select a topic ending in `/scene_camera/image`. Camera
streaming starts only when a subscriber is present.

If odometry is silent or lifecycle services are absent, check the pipeline in
order:

```bash
ros2 topic info -v /ProjectAirSim/node/projectairsim/load_scene
ros2 topic list | grep /actual_pose
ros2 topic echo <robot_path>/actual_pose --once --qos-reliability best_effort
ros2 topic echo --once /navigation/odom
ros2 service list | grep -E 'enable_api_control|takeoff|land'
```

When inspecting the bridge pose directly, request the matching QoS explicitly
if your ROS 2 CLI does not negotiate it automatically:

```bash
ros2 topic echo <robot_path>/actual_pose --once --qos-reliability best_effort
```

`/navigation/trajectory` is event-driven and remains silent until a waypoint is
accepted. `/navigation/goal` is an input topic, so it also has no messages until
a waypoint is published.
