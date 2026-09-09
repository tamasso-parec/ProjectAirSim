# Project AirSim Quick Start

## Launch Blocks

In the first terminal:

```bash
cd /home/tom/Documents/ProjectAirSim
./packages/Blocks/Development/Linux/Blocks.sh -log
```

Wait for the Blocks environment to open.

## Run the drone smoke test

In a second terminal:

```bash
cd /home/tom/Documents/ProjectAirSim
source airsim-venv/bin/activate
cd client/python/example_user_scripts
python hello_drone.py
```

The example should spawn `Drone1`, display its camera feeds, take off, move up
and down, and land. Keep Blocks running while using client scripts.

Other examples are available in `client/python/example_user_scripts`, including:

```bash
python hello_rover.py
```

## Headless launch options

Run with off-screen rendering (camera rendering remains available):

```bash
./packages/Blocks/Development/Linux/Blocks.sh -RenderOffScreen
```

Run without rendering (camera images are unavailable):

```bash
./packages/Blocks/Development/Linux/Blocks.sh -nullrhi
```

Use the Development package for normal testing and the Shipping package for
performance testing.

## ROS 2 bridge (velocity control)

In a second terminal, build once, then launch the bridge against a running Blocks instance:

```bash
cd /home/tom/Documents/ProjectAirSim
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths ros/node/projectairsim-rosbridge ros/node/projectairsim-ros2 \
  --symlink-install
source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_bridge_ros2.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config"
```

Blocks starts with an empty scene, so spawn `Drone1` by loading a scene through the bridge (in another terminal):

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_basic_drone.jsonc}'
```

This spawns robot/sensor topics and services under `/Sim/SceneBasicDrone/robots/Drone1` (scene id + robot name from the scene JSON). For example:

```bash
ros2 service call /Sim/SceneBasicDrone/robots/Drone1/enable_api_control std_srvs/srv/SetBool '{data: true}'
ros2 service call /Sim/SceneBasicDrone/robots/Drone1/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /Sim/SceneBasicDrone/robots/Drone1/takeoff std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 /Sim/SceneBasicDrone/robots/Drone1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
```

See [`docs/ros/ros.md`](docs/ros/ros.md) for full setup, topics/services, and `cmd_vel` usage.

## PX4 offboard control over ROS 2

This is the fiddly one: four processes (Unreal, PX4 SITL, the ROS 2 bridge, MAVROS2) that must start **in this order** and hit specific checkpoints. See [`docs/ros/ros.md#px4-offboard-control-mavros2`](docs/ros/ros.md#px4-offboard-control-mavros2) for the full explanation, every checkpoint's expected output, and a troubleshooting table — this section is just the command sequence. One-time setup (PX4 SITL build, MAVROS2 install) is also there.

Requires a robot config with `"type": "px4-api"`, e.g. `robot_quadrotor_px4_sitl.jsonc` / `scene_px4_sitl.jsonc`.

Terminal 1 — Blocks:

```bash
cd /home/tom/Documents/ProjectAirSim
./packages/Blocks/Development/Linux/Blocks.sh -log
```

Terminal 2 — PX4 SITL. Wait for `Waiting for simulator to connect on TCP port 4560` before moving on:

```bash
cd PX4/PX4-Autopilot
make px4_sitl none_iris
```

Terminal 3 — ROS 2 bridge + MAVROS2:

```bash
cd /home/tom/Documents/ProjectAirSim
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras   # first time only
ros2 run mavros install_geographiclib_datasets.sh              # first time only
source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_px4_bridge.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  fcu_url:="udp://:14550@127.0.0.1:14550"
```

Terminal 4 — load the scene (this is what makes Project AirSim connect to PX4; also `/Sim/SceneBasicDrone/robots/Drone1`, same scene id/robot name as the default scene):

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_px4_sitl.jsonc}'
```

Back in Terminal 2, wait for `home_set` in the PX4 console log before arming — arming or switching to OFFBOARD earlier fails with `Takeoff denied, disarm and re-try`. Then, in Terminal 4, confirm `ros2 topic echo /mavros/state --once` shows `connected: true`, and arm, switch to OFFBOARD, and fly to a setpoint:

```bash
ros2 topic pub --rate 20 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}}}' &
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "OFFBOARD"}'
```

When done, shut down in order (stop setpoint publisher → land/disarm → Ctrl-C Terminal 3 → `shutdown` in Terminal 2's PX4 console → close Blocks) — a leftover PX4 process is the most common reason the *next* session fails. Full details in [`docs/ros/ros.md`](docs/ros/ros.md#px4-offboard-control-mavros2).
