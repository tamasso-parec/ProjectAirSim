# Project AirSim Quick Start

All commands run from `/home/tom/Documents/ProjectAirSim`.

## 1. Launch Blocks

```bash
./packages/Blocks/Development/Linux/Blocks.sh -log
```

Wait for the window to open. The scene starts empty — that is expected; nothing spawns until a scene is loaded.

Headless variants: `-RenderOffScreen` keeps cameras working, `-nullrhi` disables rendering entirely (no camera images). Use the Development package for testing, Shipping for performance.

## 2. Smoke test with the Python client

```bash
source airsim-venv/bin/activate
cd client/python/example_user_scripts
python hello_drone.py
```

`Drone1` should spawn, show its camera feeds, take off, move and land. Other examples live alongside it (`hello_rover.py`, and more).

## 3. ROS 2 bridge

Build once, then launch against the running Blocks instance:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros/node/projectairsim-rosbridge ros/node/projectairsim-ros2 --symlink-install
source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_bridge_ros2.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config"
```

In another terminal, spawn the drone by loading a scene:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_basic_drone.jsonc}'
```

Then fly it:

```bash
export ROBOT_PATH=/Sim/SceneBasicDrone/robots/Drone1
ros2 service call $ROBOT_PATH/enable_api_control std_srvs/srv/SetBool '{data: true}'
ros2 service call $ROBOT_PATH/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call $ROBOT_PATH/takeoff std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 $ROBOT_PATH/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
```

## 4. Replacing another simulator

To have the bridge present the topic names, transform frames, ENU convention, simulated time, metric depth and point clouds that an existing ROS 2 stack already expects, pass an interface profile:

```bash
ros2 launch projectairsim_ros2 projectairsim_bridge_ros2.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  interface_profile:="$PWD/ros/node/projectairsim-ros2/config/interface_profile_example.yaml"
```

See [Standing in for another simulator](docs/ros/ros.md#standing-in-for-another-simulator).

## 5. PX4 offboard control

Two transports: **uXRCE-DDS** (`px4_msgs`, `/fmu/*`) and **MAVROS2** (`/mavros/*`). Both need a `px4-api` robot config.

The order always matters, because PX4 waits on TCP 4560 and Project AirSim only connects once a scene with a `px4-api` robot is loaded: **Blocks → PX4 SITL → bridge → load scene → wait for `home_set` → arm.**

For uXRCE-DDS, one launch file handles all of it (with Blocks already running):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_px4_sitl.launch.py \
  px4_dir:=/path/to/PX4-Autopilot \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  scene:=scene_x500_realsense_ground.jsonc \
  interface_profile:="$PWD/ros/node/projectairsim-ros2/config/interface_profile_example.yaml"
```

**Do not arm until `home_set` appears** in the PX4 output — that is the most common sticking point.

One-time setup (PX4, the Micro XRCE-DDS Agent, `px4_msgs`, MAVROS), the X500 + RealSense vehicle that mirrors PX4's Gazebo `x500_realsense` model, the flight sequence for each transport, shutdown order and a troubleshooting table are all in [PX4 offboard control](docs/ros/ros.md#px4-offboard-control).

## Running tests

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select projectairsim_rosbridge projectairsim_ros2
colcon test-result --verbose
```

> If a simulation is already running on this machine, isolate the test run first — the suite publishes real `/clock` messages, which would disturb any node using `use_sim_time` on the same DDS domain:
>
> ```bash
> export ROS_DOMAIN_ID=91     # anything but the default 0
> ```

## Where to look next

| Topic | Document |
| --- | --- |
| Full bridge reference: topics, services, transforms, profile options | [`docs/ros/ros.md`](docs/ros/ros.md) |
| PX4 controller configuration | [`docs/controllers/px4/`](docs/controllers/px4/) |
| Robot and scene configuration | [`docs/config_robot.md`](docs/config_robot.md) |
