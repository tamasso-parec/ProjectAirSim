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

## Loading a scene (spawning the robot)

An Unreal instance starts with an empty `DefaultScene`; nothing under `<robot_path>` exists until a scene is loaded. The bridge does not load one automatically. Publish a scene configuration filename relative to `sim_config_path` to spawn its actors:

```bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_basic_drone.jsonc}'
```

This calls the same `projectairsim.World(...)` load that a Python client script (e.g. `hello_drone.py`) performs; use either one, but not both against the same Project AirSim server at once (see [ROS 1 compatibility](#ros-1-compatibility)).

Once loaded, robot and sensor topics/services appear under `/Sim/<scene id>/robots/<robot name>`, where `<scene id>` is the scene JSON's top-level `"id"` and `<robot name>` is the robot actor's `"name"` in its `actors` list. For `scene_basic_drone.jsonc` (`"id": "SceneBasicDrone"`, robot `"name": "Drone1"`) this is `/Sim/SceneBasicDrone/robots/Drone1`, giving e.g. `/Sim/SceneBasicDrone/robots/Drone1/cmd_vel` and `/Sim/SceneBasicDrone/robots/Drone1/arm`. `scene_px4_sitl.jsonc` uses the same scene id and robot name, so its `<robot_path>` is identical.

```bash
export ROBOT_PATH=/Sim/SceneBasicDrone/robots/Drone1
```

## Moving a drone from ROS 2

Interfaces are generated under each discovered `<robot_path>` (see above). To confirm the exact path for a given scene rather than deriving it by hand:

```bash
ros2 service list | grep enable_api_control
ros2 topic list | grep cmd_vel
```

Enable API control, arm, and take off before sending velocity commands. Replace `<robot_path>` with the discovered path:

```bash
ros2 service call $ROBOT_PATH/enable_api_control std_srvs/srv/SetBool '{data: true}'
ros2 service call $ROBOT_PATH/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call $ROBOT_PATH/takeoff std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 $ROBOT_PATH/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0, y: 0.0, z: 1.0}, angular: {z: 0.0}}'
```

Stop the publisher with Ctrl-C and land explicitly:

```bash
ros2 service call $ROBOT_PATH/land std_srvs/srv/Trigger '{}'
ros2 service call $ROBOT_PATH/arm std_srvs/srv/SetBool '{data: false}'
```

The lifecycle services return `success: false` and the Project AirSim error when a robot/controller rejects a request. Takeoff and landing services wait for completion or their configured timeout.

## PX4 offboard control (MAVROS2)

`cmd_vel` above talks to whatever controller the robot's configuration selects (`simple-flight-api`, `manual-controller-api`, or `px4-api`) through the same generic Project AirSim RPC methods. For a `px4-api` robot, ROS 2 can additionally command the vehicle through the real PX4 autopilot's offboard mode using standard MAVROS2, independent of this bridge.

This involves four independent processes (Unreal, PX4 SITL, the ROS 2 bridge, MAVROS2) that all have to come up **in a specific order** and find each other on specific ports, which is what makes it fiddly. The steps below spell out that order, with a checkpoint to confirm after each one before moving to the next — if something isn't working, it's almost always because a checkpoint earlier in the list was skipped.

### Why the ordering matters

- PX4 SITL starts and waits for something to connect to it on TCP port `4560`; it does nothing else until then.
- Project AirSim only opens that connection once a scene containing a `px4-api` robot is actually loaded — an empty Unreal instance never talks to PX4.
- PX4 only starts its EKF/GPS fusion and sets its home position *after* Project AirSim connects, and it refuses to arm or accept OFFBOARD until the home position is set.
- MAVROS is a second, independent MAVLink client. It can technically be started at any time, but it has nothing useful to report until PX4 has a GPS fix, and there's no point troubleshooting MAVROS/OFFBOARD problems before that point.

So the only safe order is: **Unreal → PX4 SITL → load the scene → wait for GPS fusion/home position in the PX4 console → MAVROS → arm/OFFBOARD.**

### One-time setup

1. Build PX4 SITL for the `iris` airframe (Project AirSim's PX4 quadrotor configs model an iris-class frame) per [`docs/controllers/px4/px4_sitl.md`](../controllers/px4/px4_sitl.md):

   ```bash
   mkdir -p PX4 && cd PX4
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
   cd PX4-Autopilot
   git checkout v1.12.3   # or the latest stable release
   make px4_sitl none_iris
   ```

   This also starts PX4 once the build finishes; stop it with `shutdown` in its console for now — you'll start it again in its own terminal below.

2. Install MAVROS2 for ROS 2 Humble (one time):

   ```bash
   sudo apt install ros-humble-mavros ros-humble-mavros-extras
   ros2 run mavros install_geographiclib_datasets.sh
   ```

3. Make sure the robot/scene config selects PX4: `robot_quadrotor_px4_sitl.jsonc` (`"type": "px4-api"`) and `scene_px4_sitl.jsonc` in `client/python/example_user_scripts/sim_config/` already do this and are used in the commands below. `control-port` (default `14540`) is Project AirSim's own link to PX4 — never point MAVROS at it. PX4's separate GCS-facing endpoint (default UDP `14550`) is what MAVROS should use.

### Every-session launch order

**Terminal 1 — Unreal:**

```bash
./packages/Blocks/Development/Linux/Blocks.sh -log
```

Wait for the window (or, headless, the log) to settle. Nothing is loaded yet — that's expected.

**Terminal 2 — PX4 SITL:**

```bash
cd PX4/PX4-Autopilot
make px4_sitl none_iris
```

Checkpoint — you should see it waiting, and nothing further, until Terminal 3:

```
INFO  [simulator] Waiting for simulator to connect on TCP port 4560
```

**Terminal 3 — ROS 2 bridge + MAVROS2**, then load the scene:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_px4_bridge.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  fcu_url:="udp://:14550@127.0.0.1:14550"
```

In a fourth terminal, trigger the scene load (this is what makes Project AirSim connect to PX4):

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_px4_sitl.jsonc}'
```

Checkpoint — back in Terminal 2 (PX4 console), within a few seconds you should see, in order:

```
INFO  [simulator] Simulator connected on UDP port 14560
INFO  [mavlink] partner IP: 127.0.0.1
INFO  [ecl/EKF] EKF GPS checks passed (WGS-84 origin set)
INFO  [ecl/EKF] EKF commencing GPS fusion
...
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
INFO  [tone_alarm] home_set
```

**Do not proceed until `home_set` appears.** Arming or switching to OFFBOARD before this point fails with `WARN [commander] Takeoff denied, disarm and re-try`, and it's the single most common point people get stuck.

Checkpoint — confirm MAVROS sees PX4 (same terminal as the scene-load command, or a new one):

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/state --once
```

Expect `connected: true`. If it's `false`, MAVROS never reached PX4 — see [Troubleshooting](#px4mavros-troubleshooting) below before continuing.

### Flight sequence

PX4 requires setpoints streamed at >2 Hz *before and during* the switch to OFFBOARD, or it rejects the mode change. Start the publisher first, in the background, then arm and switch modes:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --rate 20 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}}}' &
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "OFFBOARD"}'
```

Checkpoint — `ros2 topic echo /mavros/state --once` should now show `armed: true`, `mode: OFFBOARD`, and the drone should visibly climb to 2 m in the Unreal viewport.

Velocity and attitude/rate setpoints work the same way through `/mavros/setpoint_velocity/cmd_vel` (`geometry_msgs/TwistStamped`) and `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`).

Keep the existing bridge (`projectairsim_bridge_ros2`) for sensor/camera streaming; don't drive the same vehicle from both `$ROBOT_PATH/cmd_vel` and MAVROS setpoints at the same time.

### Shutting down (in order)

PX4 SITL will not reconnect to a second simulation instance once it's connected to one — a stale PX4 process is the most common reason a *second* attempt fails even though the first one worked. Shut down in this order every time:

1. Stop the setpoint publisher (`fg` then Ctrl-C, or `kill %1`).
2. Land: switch PX4 out of OFFBOARD to `AUTO.LAND` (`ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "AUTO.LAND"}'`) or use the bridge's `$ROBOT_PATH/land` service, then disarm.
3. Ctrl-C the bridge+MAVROS launch (Terminal 3).
4. In the PX4 console (Terminal 2), run `shutdown` (or Ctrl-C if it's unresponsive).
5. Close Unreal (Terminal 1).

### PX4/MAVROS troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| PX4 console stuck on "Waiting for simulator to connect on TCP port 4560" | Scene not loaded yet, or wrong robot config | Publish `load_scene` with `scene_px4_sitl.jsonc`; confirm the robot config's `controller.type` is `px4-api`. |
| `WARN [commander] Takeoff denied, disarm and re-try` | Home position not set yet (no `home_set` log line) | Wait longer after the scene loads; if it never appears, check `LPE_LAT`/`LPE_LON` in the robot config match the scene's `home-geo-point`. |
| `/mavros/state` shows `connected: false` | MAVROS pointed at the wrong port, or PX4 not running | Use PX4's GCS port (`udp://:14550@127.0.0.1:14550` for SITL), not `14540` (that's Project AirSim's own link); confirm PX4 SITL is actually running. |
| `set_mode` to `OFFBOARD` succeeds but PX4 immediately drops back out of OFFBOARD | Setpoint publisher stopped, too slow (<2 Hz), or started after the mode switch instead of before | Start the `ros2 topic pub --rate 20 ...` setpoint stream *before* calling `set_mode`, and keep it running continuously. |
| A second SITL session behaves like the first never happened / ports appear busy | Previous PX4 SITL process still running and already bound to a simulation | Fully stop PX4 (`shutdown` in its console, or `pkill -x px4`) and Unreal before starting a new session — see [Shutting down](#shutting-down-in-order). |
| MAVROS and *QGroundControl* both fighting over the same port | Both configured for UDP `14550` | Only run one GCS-like client on that port at a time, or give QGC a distinct port via the robot config's `qgc-host-ip`/`qgc-port` (HITL only; see [`docs/config_robot.md`](../config_robot.md#px4-communication-port-settings)). |

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
