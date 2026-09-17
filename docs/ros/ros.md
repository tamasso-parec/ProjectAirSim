# Project AirSim ROS bridge

A shared Python bridge with ROS 1 and ROS 2 adapters. It connects through the Project AirSim Client API, discovers robots and sensors from the active scene, and exposes matching ROS topics, services and transforms. Unreal keeps rendering; ROS receives the rendered camera and simulated sensor data.

ROS 2 Humble on Ubuntu 22.04 is the primary target. The ROS 1 adapter remains available for Noetic.

**Contents**

- [Setup](#setup) · [Quick start](#quick-start)
- [Standing in for another simulator](#standing-in-for-another-simulator) — interface profiles, simulated time, depth and point clouds, ground truth, collisions
- [PX4 offboard control](#px4-offboard-control) — uXRCE-DDS and MAVROS
- [Reference](#reference) — topics, transforms, profile options
- [Validation](#validation) · [Other tasks](#other-tasks)

---

## Setup

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep ros-humble-radar-msgs
```

The installed bridge executable uses the interpreter that runs `colcon` — normally `/usr/bin/python3`, even with a virtualenv active. Install the client for *that* interpreter:

```bash
sudo apt install python3-pip
/usr/bin/python3 -m pip install --user -e client/python/projectairsim
/usr/bin/python3 -c "import projectairsim; print(projectairsim.__file__)"   # must succeed
```

Then build, from the repository root:

```bash
rosdep update
rosdep install --from-paths ros/node/projectairsim-rosbridge ros/node/projectairsim-ros2 --ignore-src -r -y
colcon build --base-paths ros/node/projectairsim-rosbridge ros/node/projectairsim-ros2 --symlink-install
source install/setup.bash
```

If the bridge cannot import `projectairsim`, check which interpreter it was built against:

```bash
head -1 install/projectairsim_ros2/lib/projectairsim_ros2/projectairsim_bridge_ros2
```

ROS libraries (`rclpy`, message packages, TF) are apt dependencies, not pip ones. `ros_gz_interfaces` is optional and needed only for [collision reporting](#collisions).

---

## Quick start

Start an Unreal environment, then the bridge, then load a scene. **Nothing exists under `<robot_path>` until a scene is loaded** — an Unreal instance starts empty and the bridge does not load one for you.

```bash
# Terminal 1
./packages/Blocks/Development/Linux/Blocks.sh -log

# Terminal 2
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_bridge_ros2.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config"

# Terminal 3
source /opt/ros/humble/setup.bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_basic_drone.jsonc}'
```

Topics and services then appear under `/Sim/<scene id>/robots/<robot name>`, from the scene JSON's `id` and the actor's `name`. For `scene_basic_drone.jsonc` that is:

```bash
export ROBOT_PATH=/Sim/SceneBasicDrone/robots/Drone1
ros2 service call $ROBOT_PATH/enable_api_control std_srvs/srv/SetBool '{data: true}'
ros2 service call $ROBOT_PATH/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call $ROBOT_PATH/takeoff std_srvs/srv/Trigger '{}'
ros2 topic pub --rate 10 $ROBOT_PATH/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0, y: 0.0, z: 1.0}, angular: {z: 0.0}}'
```

Ctrl-C the publisher, then land and disarm:

```bash
ros2 service call $ROBOT_PATH/land std_srvs/srv/Trigger '{}'
ros2 service call $ROBOT_PATH/arm std_srvs/srv/SetBool '{data: false}'
```

`cmd_vel` drives whichever controller the robot config selects (`simple-flight-api`, `manual-controller-api` or `px4-api`). Lifecycle services return `success: false` with the simulator's error when rejected.

### Bridge parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `address` | string | `127.0.0.1` | Project AirSim server address. |
| `topics_port` | integer | `8989` | Pub/sub port. |
| `services_port` | integer | `8990` | Service port. |
| `sim_config_path` | string | `sim_config/` | Directory holding scene configuration files. |
| `interface_profile` | string | `""` | Path to an [interface profile](#interface-profiles). Empty keeps the bridge's default naming. |
| `use_sim_time` | bool | `false` | Publish `/clock` from the simulation clock and stamp messages with it. See [Simulated time](#simulated-time). |
| `cmd_vel_timeout_sec` | double | `1.0` | Duration of each velocity request and motion failsafe. Publish faster than this. |
| `takeoff_timeout_sec` | double | `20.0` | Takeoff service timeout. |
| `land_timeout_sec` | double | `60.0` | Landing service timeout. |

`node_name:=` changes the node name (default `projectairsim`). Ctrl-C removes ROS entities, cancels pending flight operations and disconnects the client.

---

## Standing in for another simulator

Project AirSim names its topics after the loaded scene and robot:

```
/Sim/SceneBasicDrone/robots/Drone1/sensors/RGBD/scene_camera/image
```

An existing ROS stack expects its own fixed names, standard ENU, simulated time and metric depth. An **interface profile** declares that mapping so Project AirSim can replace another simulator without editing the stack.

```bash
ros2 launch projectairsim_ros2 projectairsim_bridge_ros2.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  interface_profile:="$PWD/ros/node/projectairsim-ros2/config/interface_profile_example.yaml"
```

The bridge logs the effective profile at start-up, before it connects — so the settings are on record even if the connection then fails:

```
[INFO] Interface profile: /path/to/profile.yaml; frame convention enu; world frame map; depth 32FC1 clamped to 10.0 m
[INFO] Simulated time enabled: publishing /clock from Project AirSim message timestamps.
[INFO] Robot transform broadcast disabled by profile
```

### Interface profiles

Every section is optional; omitting the file reproduces the bridge's default behaviour. A commented reference covering every option ships at [`config/interface_profile_example.yaml`](../../ros/node/projectairsim-ros2/config/interface_profile_example.yaml).

```yaml
frame_convention: enu           # nwu (default) | enu
world_frame: map

sim_time:
  enabled: true

depth:
  encoding: 32FC1               # 32FC1 (default) | 16UC1 | mono8
  max_range_m: 10.0             # 0 disables the clamp

tf:
  publish_robot_tf: false
  publish_sensor_tf: false
  ground_truth_topic: /ground_truth/tf

collision:
  message: gz_contacts          # none (default) | gz_contacts

frames:
  "*/robots/Drone1": x500_realsense
  "*/robots/Drone1/sensors/RGBD": x500_realsense/realsense_d435/color_optical_frame

topics:
  "*/robots/Drone1/actual_pose": /ground_truth/pose
  "*/robots/Drone1/collision_info": /evaluation/contacts
  "*/robots/Drone1/sensors/RGBD/scene_camera": /rgbd_camera
  "*/robots/Drone1/sensors/RGBD/depth_planar_camera":
    image: /rgbd_camera/depth_image
    camera_info: /rgbd_camera/depth_camera_info
    points: /rgbd_camera/points
```

Rules:

- Patterns are shell-style globs matched against the full Project AirSim path, and `*` also matches `/`, so `*/robots/Drone1` matches any scene. **First match wins** — put specific patterns first.
- Match robots on their robot path, sensors on their sensor path. Every image type of one camera shares that camera's frame, because they share an optical centre.
- A camera alias is either a plain string used as a base name (`/image` and `/camera_info` appended), or a mapping naming topics explicitly. Use the mapping when the names do not share a base, or to add `points`.
- An unreadable path, unparseable YAML or unknown key is rejected at start-up with the offending key named, never silently ignored.

See [Profile options](#profile-options) for the full list of settings and defaults.

### Simulated time

Project AirSim stamps every sensor message with its simulation clock in nanoseconds. By default the bridge ignores that and uses the wall clock — fine for viewing, wrong for TF lookups, message filters, `rosbag2` replay, or any node with `use_sim_time`.

Set `use_sim_time:=true` (or `sim_time.enabled` in a profile) and the bridge stamps every message and transform from the simulation clock, and republishes it on `/clock`.

**Run every node in the stack with `use_sim_time:=true` as well.** Mixing is worse than using neither: wall-stamped messages are silently discarded by nodes on simulation time.

Two consequences:

- **`/clock` starts only once a scene with a robot is loaded.** Simulation time advances when the simulator produces data, and the bridge derives it from message timestamps rather than polling. Before then, nodes on simulation time see time zero and block — exactly as they do before a Gazebo server starts.
- **Loading a new scene restarts the clock at zero.** The bridge resets its clock source on a scene load, so the new scene's timestamps are accepted rather than rejected as stale. A large backwards jump is logged.

`/clock` is published reliably with depth one, rate-limited by `sim_time.min_step_ms` (1 ms, so at most 1 kHz) however fast the simulator steps.

### Depth images and point clouds

Depth is published as **`32FC1` metres** by default, which is what `depth_image_proc`, RTAB-Map and ORB-SLAM3 expect. Samples with no reading become `NaN`; samples beyond range become `+inf`, so consumers skip them instead of mapping phantom geometry.

Project AirSim transmits depth as 16-bit millimetres saturating at 65.535 m, so set `depth.max_range_m` to the far clip of the sensor you are standing in for.

> **Behaviour change:** earlier versions published depth as `mono8` scaled so 6 m mapped to 255 — lossy by roughly 24× and unusable for mapping or SLAM. Set `depth.encoding: mono8` to restore it, or use the `depth_vis_camera` image type, which exists for viewing.

Use **`depth_planar_camera`** (planar Z depth, image type 1) when replacing a Gazebo depth camera or a RealSense. `depth_camera` (image type 2) is perspective distance and would read as a curved wall.

Naming a `points` topic on a depth camera makes the bridge reproject it using that camera's own intrinsics. Doing this in the bridge rather than a downstream `depth_image_proc` node avoids shipping the depth image over DDS only to convert it, needs no extra package, and gives the cloud the same timestamp and frame as the image. `depth_image_proc::PointCloudXyzNode` remains a fine alternative.

The cloud is organised — one point per pixel, row major — and not dense: invalid pixels are `NaN`, exactly as a Gazebo depth camera publishes them. `points.frame_convention` selects `optical` (X right, Y down, Z forward; the REP 103 default) or `ros` (X forward, Y left, Z up); `points.decimation` keeps every Nth pixel per axis.

Reprojection is skipped entirely while the topic has no subscribers, and a `points` topic on a non-depth image type is refused with a warning rather than reading colour bytes as depth.

### Ground truth and transform frames

A ROS transform frame can have only one parent. A stack with its own state estimator — a PX4 or SLAM pipeline publishing the vehicle frame, or a robot description supplying sensor extrinsics — must stop the bridge claiming those frames, or two publishers fight over one frame:

```yaml
tf:
  publish_robot_tf: false
  publish_sensor_tf: false
  ground_truth_topic: /ground_truth/tf
```

`ground_truth_topic` publishes each robot's simulator pose as a `tf2_msgs/TFMessage` on its own topic, parented to `world_frame`. That pairing is the point: the bridge stays off the vehicle frame in the transform tree, while evaluation code still gets exact ground truth. It is deliberately **not** `/tf`. Pose topics are published either way.

### Collisions

Project AirSim reports collisions on each robot's `collision_info` topic. ROS has no canonical collision message, so the shape is explicit and defaults to not publishing:

```yaml
collision:
  message: gz_contacts
topics:
  "*/robots/Drone1/collision_info": /evaluation/contacts
```

`gz_contacts` produces `ros_gz_interfaces/msg/Contacts`, what a Gazebo-based stack subscribes to. That package is imported only when this setting selects it, so it is **not** a dependency for everyone else:

```bash
sudo apt install ros-$ROS_DISTRO-ros-gz-interfaces
```

Project AirSim reports one collision at a time, so the message carries a single contact, with the robot's own frame standing in for the side Gazebo would name.

---

## PX4 offboard control

For a `px4-api` robot, ROS 2 can command the vehicle through the real PX4 autopilot's offboard mode. Project AirSim supplies physics and rendering over PX4's MAVLink simulator link (TCP 4560); your stack talks to PX4 over one of two transports.

### Choosing a transport

| | uXRCE-DDS (`px4_msgs`) | MAVROS2 |
| --- | --- | --- |
| Topics | `/fmu/in/*`, `/fmu/out/*` | `/mavros/*` |
| Transport | PX4 uORB over DDS, via the Micro XRCE-DDS Agent | MAVLink, via PX4's GCS endpoint (UDP 14550) |
| PX4 version | v1.14 or newer | any |
| Launch file | `projectairsim_px4_sitl.launch.py` | `projectairsim_px4_bridge.launch.py` |

Pick uXRCE-DDS if your stack already uses `px4_msgs` — it is PX4-native and the launch file below handles the whole start-up order. Neither transport affects Project AirSim's own MAVLink link to PX4.

> **PX4 version.** Project AirSim's docs state it "supports PX4 v1.12.3. Other versions may work but are unsupported" ([`px4.md`](../controllers/px4/px4.md#supported-versions-of-px4)). That is over-conservative. uXRCE-DDS did not exist in v1.12 — it became the default ROS 2 bridge in v1.14 — so `px4_msgs` needs a newer PX4 than that note validates. Inspecting PX4 v1.18 confirms `simulator_mavlink` still handles `HIL_SENSOR`, `HIL_GPS` and `HIL_STATE_QUATERNION` and emits `HIL_ACTUATOR_CONTROLS` with lock-step, that `10016_none_iris` still routes through `px4-rc.mavlinksim`, and that `uxrce_dds_client` starts unconditionally in `rcS`. So recent PX4 should work; this has been verified by reading PX4's source, not by flying it. Prefer the oldest uXRCE-DDS-capable release that suits you over `main`.

### Why the start-up order matters

- PX4 SITL starts and waits on TCP 4560. It does nothing else until something connects.
- **Project AirSim only opens that connection once a scene containing a `px4-api` robot is loaded.** An empty Unreal instance never talks to PX4.
- PX4 starts its estimator and sets its home position only *after* Project AirSim connects, and refuses to arm or accept OFFBOARD until home is set.

So the order is always: **Unreal → PX4 SITL → bridge → load scene → wait for `home_set` → arm/OFFBOARD.**

### One-time setup

**1. PX4 SITL.** Build for whichever airframe you need — `none_iris` for the stock `robot_quadrotor_px4_sitl.jsonc`, or install the X500 airframe below:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
cd PX4-Autopilot && make px4_sitl_default
export PX4_AUTOPILOT_DIR=$PWD          # the launch file below reads this
```

**2. For uXRCE-DDS — the Micro XRCE-DDS Agent and `px4_msgs`:**

```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig /usr/local/lib/
```

Build `px4_msgs` from **the branch matching your PX4 firmware exactly** — uORB layouts change between versions, and a mismatch shows up as topics that exist but carry empty or garbage fields:

```bash
cd <your_ros2_ws>/src && git clone -b release/1.14 https://github.com/PX4/px4_msgs.git
cd <your_ros2_ws> && colcon build --packages-select px4_msgs && source install/setup.bash
```

**3. For MAVROS — install it:**

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
ros2 run mavros install_geographiclib_datasets.sh
```

`control-port` (default 14540) is Project AirSim's own link to PX4 — **never point MAVROS at it**. Use PX4's GCS endpoint, UDP 14550.

### The x500 + RealSense vehicle

Project AirSim ships a vehicle, two scenes and a PX4 airframe reproducing PX4's Gazebo `x500_realsense` model, so a stack developed against that model can be pointed here without retuning:

| File | Purpose |
| --- | --- |
| [`robot_x500_realsense_px4_sitl.jsonc`](../../client/python/example_user_scripts/sim_config/robot_x500_realsense_px4_sitl.jsonc) | X500 with a forward-facing D435, `px4-api` in lock-step |
| [`scene_x500_realsense_ground.jsonc`](../../client/python/example_user_scripts/sim_config/scene_x500_realsense_ground.jsonc) | Open ground, for bring-up |
| [`scene_x500_realsense_wall.jsonc`](../../client/python/example_user_scripts/sim_config/scene_x500_realsense_wall.jsonc) | One wall between the vehicle and a goal ~12 m ahead |
| [`config/px4_airframes/10020_none_x500`](../../ros/node/projectairsim-ros2/config/px4_airframes/10020_none_x500) | PX4 airframe for the MAVLink simulator link |

Mass (2.064 kg), body inertia, 0.174 m arms and per-rotor peak thrust (8.549 N) and torque (0.1368 N·m) are derived from PX4's `x500_base`/`x500` SDF and reproduce it to within one part in a billion. Camera resolution, rate, mount offset and focal length match the Gazebo RealSense: 640×480 at 30 Hz, 60° horizontal FOV, `fx = fy = 554.256`.

Two differences are deliberate:

- **Principal point.** Project AirSim uses `width/2`, Gazebo `(width + 1)/2` — half a pixel apart.
- **Thrust curve.** Project AirSim's rotor model is *linear* in the normalized actuator command; Gazebo maps that command to a rotor speed and squares it. Parameter matching fixes the envelope — peak thrust, thrust-to-weight 1.69, hover at 59% throttle — but not the curve in between. PX4 is told the matching hover point (`MPC_THR_HOVER 0.6`).

**Installing the airframe.** PX4 airframes are compiled into the firmware, so the file must be copied into a PX4 checkout and PX4 rebuilt:

```bash
ros/node/projectairsim-ros2/config/px4_airframes/install_px4_airframes.sh /path/to/PX4-Autopilot
cd /path/to/PX4-Autopilot && make px4_sitl_default
```

The script refuses to run while a `px4` process is alive, since rebuilding underneath a running SITL instance disturbs it.

PX4's own `4001_gz_x500` cannot be used here: it sets `SIM_GZ_EN` and `PX4_SIMULATOR=gz`, routing PX4 to its Gazebo bridge instead of the MAVLink link. `10020_none_x500` carries identical control allocation but leaves both unset.

**The wall asset.** `scene_x500_realsense_wall.jsonc` scales Unreal's rounded template cube, `TemplateCube_Rounded`. Confirm that name on first run and change the single `asset-path` line if it differs:

```python
world.list_assets(".*[Cc]ube.*")
```

### Running it — uXRCE-DDS

With Unreal already running, one launch file starts PX4 SITL, the Agent, the bridge, the vehicle's fixed sensor extrinsics, and then publishes `load_scene` on a delay:

```bash
# Terminal 1
./packages/Blocks/Development/Linux/Blocks.sh -log

# Terminal 2
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_px4_sitl.launch.py \
  px4_dir:=/path/to/PX4-Autopilot \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  scene:=scene_x500_realsense_ground.jsonc \
  interface_profile:="$PWD/ros/node/projectairsim-ros2/config/interface_profile_example.yaml"
```

The scene load is delayed rather than immediate because loading it is what makes Project AirSim dial PX4, and the bridge must have advertised `load_scene` first.

Checkpoint — in the PX4 output, within a few seconds of the scene loading:

```
INFO  [simulator] Simulator connected on UDP port 14560
INFO  [ecl/EKF] EKF commencing GPS fusion
INFO  [commander] home: 47.6414680, -122.1401672, 119.99
INFO  [tone_alarm] home_set
```

**Do not arm until `home_set` appears.** Earlier attempts fail with `Takeoff denied, disarm and re-try` — the single most common sticking point.

Checkpoint — PX4's ROS 2 topics are live:

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_status --once
```

Useful arguments: `px4:=false` runs PX4 in its own terminal instead, which is the easier way to watch that log. A SLAM-driven stack usually adds `ekf2_gps_ctrl:=0 ekf2_ev_ctrl:=11 ekf2_hgt_ref:=3` to fuse external vision instead of GNSS. `ros2 launch ... --show-args` lists the rest.

### Running it — MAVROS

```bash
# Terminal 2 — PX4 SITL; wait for "Waiting for simulator to connect on TCP port 4560"
cd /path/to/PX4-Autopilot && make px4_sitl none_iris

# Terminal 3 — bridge + MAVROS
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch projectairsim_ros2 projectairsim_px4_bridge.launch.py \
  sim_config_path:="$PWD/client/python/example_user_scripts/sim_config" \
  fcu_url:="udp://:14550@127.0.0.1:14550"

# Terminal 4 — load the scene, then wait for home_set in Terminal 2
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_px4_sitl.jsonc}'
ros2 topic echo /mavros/state --once        # expect connected: true
```

### Flight sequence

PX4 needs setpoints streaming at >2 Hz **before and during** the switch to OFFBOARD, or it rejects the mode change and drops back out.

**MAVROS** — start the publisher first, then arm and switch:

```bash
ros2 topic pub --rate 20 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}}}' &
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "OFFBOARD"}'
```

Velocity and attitude setpoints work the same way via `/mavros/setpoint_velocity/cmd_vel` and `/mavros/setpoint_raw/attitude`.

**uXRCE-DDS** — PX4's standard offboard pattern, as in `px4_ros_com`'s `offboard_control` example:

1. Publish `OffboardControlMode` on `/fmu/in/offboard_control_mode` at ≥2 Hz, with `position: true` (or `velocity`/`acceleration`) and the rest `false`.
2. Publish `TrajectorySetpoint` on `/fmu/in/trajectory_setpoint` at the same rate — position/velocity/acceleration in NED, `yaw` in radians.
3. After a few cycles of both, publish two `VehicleCommand` messages on `/fmu/in/vehicle_command`, each with `target_system: 1`, `target_component: 1`, `from_external: true`:
   - arm: `command: 400`, `param1: 1.0`
   - OFFBOARD: `command: 176`, `param1: 1.0`, `param2: 6.0`
4. Keep publishing (1) and (2); a gap over 500 ms drops PX4 out of OFFBOARD.

> Publishers and subscribers on `/fmu/*` **must** use a best-effort, volatile QoS profile (`rclcpp::SensorDataQoS()`, or the rclpy equivalent). PX4's DDS client uses this, and a default "reliable" profile silently fails to match — which looks exactly like "the topic exists but nothing ever arrives."

Checkpoint — `/fmu/out/vehicle_status` shows `arming_state: 2` and `nav_state: 14`, or `/mavros/state` shows `armed: true`, `mode: OFFBOARD`, and the vehicle climbs in the viewport.

Keep the bridge for sensor and camera streaming, but do not drive the same vehicle from both `$ROBOT_PATH/cmd_vel` and offboard setpoints at once.

### Shutting down

PX4 SITL will not reconnect to a second simulation instance once connected to one. **A stale PX4 process is the most common reason a second attempt fails when the first worked.** Every time, in this order:

1. Stop the setpoint publishers.
2. Land and disarm — `AUTO.LAND` via MAVROS or a `VehicleCommand`, or the bridge's `$ROBOT_PATH/land`.
3. Ctrl-C the launch (this stops the Agent and bridge together).
4. `shutdown` in the PX4 console, or Ctrl-C if unresponsive.
5. Close Unreal.

### Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| PX4 stuck on "Waiting for simulator to connect on TCP port 4560" | Scene not loaded, or wrong robot config | Publish `load_scene`; confirm `controller.type` is `px4-api`. |
| `Takeoff denied, disarm and re-try` | Home position not set — no `home_set` line yet | Wait longer after the scene loads. If it never appears, check `LPE_LAT`/`LPE_LON` match the scene's `home-geo-point`. |
| A second session behaves as if the first never happened | Previous PX4 process still bound to a simulation | Fully stop PX4 and Unreal first — see [Shutting down](#shutting-down). |
| OFFBOARD is accepted then immediately dropped | Setpoints stopped, too slow (<2 Hz), or started *after* the mode switch | Start the setpoint stream before the mode switch and keep it running. |
| `/fmu/*` topics never appear | Agent not running, wrong port, or `uxrce_dds_client` not started | Confirm the Agent is on port 8888 and PX4's `uxrce_dds_client status` shows it running on the same port. |
| `/fmu/*` topics exist but `ros2 topic echo` prints nothing | QoS mismatch — default reliable instead of PX4's best-effort | Use `SensorDataQoS()` or an equivalent best-effort, volatile profile. |
| `/fmu/*` fields are all zero or nonsensical | `px4_msgs` does not match the PX4 firmware | Rebuild `px4_msgs` from the branch matching your PX4 checkout exactly. |
| `VehicleCommand` has no effect | Missing `target_system`/`target_component`/`from_external`, or setpoints not yet streaming | Set all three; start setpoints before the command. |
| `/mavros/state` shows `connected: false` | MAVROS pointed at the wrong port | Use PX4's GCS port (14550), not Project AirSim's `control-port` (14540). |
| MAVROS and QGroundControl fight over a port | Both on UDP 14550 | Run one at a time, or give QGC a distinct port via `qgc-host-ip`/`qgc-port` (HITL only). |
| Nodes on `use_sim_time` never advance | `/clock` starts only after a scene with a robot is loaded | Load the scene; see [Simulated time](#simulated-time). |

---

## Reference

`<robot_path>` is the scene-specific robot path; `<sensor_path>` and `<camera_path>` run through the configured sensor ID.

### Subscribed by the bridge

| ROS interface | Type | Behaviour |
| --- | --- | --- |
| `/ProjectAirSim/node/<node_name>/load_scene` | `std_msgs/String` | Loads a configuration from `sim_config_path`, then replaces scene-specific topics, services and frames. |
| `<robot_path>/cmd_vel` | `geometry_msgs/Twist` | Linear velocity and yaw rate. Publish continuously within `cmd_vel_timeout_sec`. |
| `<robot_path>/desired_pose` | `geometry_msgs/PoseStamped` | Sets the robot pose, when the controller exposes it. |
| `<camera_path>/desired_pose` | `geometry_msgs/PoseStamped` | Changes the camera pose for all captures from that camera. |

### Services (ROS 2 only)

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
| `/clock`, when `use_sim_time` is set | `rosgraph_msgs/Clock` |
| depth reprojection, when a profile names a `points` topic | `sensor_msgs/PointCloud2` |
| ground truth, when `tf.ground_truth_topic` is set | `tf2_msgs/TFMessage` |
| collisions, when `collision.message` is not `none` | `ros_gz_interfaces/Contacts` |

Camera image types are `scene_camera`, `depth_planar_camera`, `depth_camera`, `depth_vis_camera`, `segmentation_camera`, `disparity_normalized_camera` and `surface_normals_camera`. `scene_camera` carries the photorealistic Unreal rendering. Topics exist only for captures enabled in the robot configuration, and images stream only while a ROS subscriber is attached.

High-rate sensors use the sensor-data QoS profile; camera info and other latched state use reliable transient-local; commands and services use reliable volatile.

### TF and coordinates

The bridge broadcasts each robot from `world_frame` (default `map`), and sensor frames relative to their configured parents.

Project AirSim expresses world poses in NED (X north, Y east, Z down) and body-relative quantities in FRD (X forward, Y right, Z down). Two conversions apply:

- **Body-relative values** — IMU rates and accelerations, the magnetometer body field, sensor-frame point clouds — always convert FRD to ROS FLU by negating Y and Z, independent of the world convention.
- **World values** — robot and sensor poses, `cmd_vel` and `desired_pose` requests — convert NED into the configured world convention:

| `frame_convention` | Meaning |
| --- | --- |
| `nwu` (default) | The bridge's long-standing behaviour: Y and Z negated, giving North-West-Up. Self-consistent, but a world-aligned `map` has X pointing north, so it is **not** the ENU that [REP 103](https://ros.org/reps/rep-0103.html) and most ROS software assume. |
| `enu` | Standard ROS East-North-Up. Choose this for any stack that assumes REP 103. |

The default stays `nwu` so existing users see no change. New stacks should set `enu`.

Note that `cmd_vel` is a **world-frame** request here, not body-frame as many ROS stacks assume.

### Profile options

| Setting | Default | Meaning |
| --- | --- | --- |
| `frame_convention` | `nwu` | World convention: `nwu` or `enu`. |
| `world_frame` | `map` | Parent frame for broadcast transforms. |
| `sim_time.enabled` | `false` | Stamp from the simulation clock and publish `/clock`. |
| `sim_time.clock_topic` | `/clock` | Clock topic name. |
| `sim_time.min_step_ms` | `1.0` | Minimum advance between clock messages. |
| `depth.encoding` | `32FC1` | `32FC1` metres, `16UC1` millimetres, or legacy `mono8`. |
| `depth.max_range_m` | `0.0` | Out-of-range threshold; 0 disables the clamp. |
| `depth.mono8_max_range_m` | `6.0` | Range mapping to 255, `mono8` only. |
| `points.frame_convention` | `optical` | `optical` (X right, Y down, Z forward) or `ros`. |
| `points.decimation` | `1` | Keep every Nth pixel per axis. |
| `collision.message` | `none` | `none` or `gz_contacts`. |
| `tf.publish_robot_tf` | `true` | Broadcast the vehicle frame on `/tf`. |
| `tf.publish_sensor_tf` | `true` | Broadcast sensor frames on `/tf`. |
| `tf.ground_truth_topic` | `""` | Publish ground-truth transforms here instead of `/tf`. |
| `frames` | — | Glob pattern to transform frame ID. |
| `topics` | — | Glob pattern to ROS topic name, or a camera mapping. |

---

## Validation

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select projectairsim_rosbridge projectairsim_ros2
colcon test-result --verbose
```

> **Isolate the test run if a simulation is already running on this machine.** The suite publishes real `rosgraph_msgs/Clock` messages, which would jump the simulation time of any node running with `use_sim_time` on the same DDS domain. Set an unused domain first:
>
> ```bash
> export ROS_DOMAIN_ID=91     # anything but the default 0
> ```
>
> The same applies to running the bridge by hand alongside an unrelated simulation.

An end-to-end check needs a running environment. With simulated time or a profile in use, confirm:

```bash
ros2 topic hz /clock                                   # advancing
ros2 topic echo /clock --once                          # small seconds, not a Unix timestamp
ros2 topic list | grep rgbd_camera                     # aliased names present
ros2 topic echo /rgbd_camera/depth_image --field encoding --once   # 32FC1
ros2 topic echo /rgbd_camera/points --field height --once          # organised cloud
ros2 run tf2_tools view_frames                         # one publisher per frame
```

Then verify lifecycle services, `cmd_vel` motion, camera images and `CameraInfo`, configured sensor topics, scene replacement, landing and clean shutdown.

---

## Other tasks

### Changing scenes

```bash
ros2 topic pub --once /ProjectAirSim/node/projectairsim/load_scene std_msgs/msg/String \
  '{data: scene_drone_sensors.jsonc}'
```

Pending takeoff or landing operations are cancelled, and the previous scene's interfaces and transforms are removed before the new scene is discovered.

### Viewing rendered images

```bash
sudo apt install ros-humble-rqt-image-view
rqt_image_view
```

### Minimal waypoint navigation

The optional `projectairsim_navigation` package provides a small navigation example using simulator ground truth, a straight-line trajectory and a velocity feedback controller. It has no mapping or collision avoidance by design. See [its README](../../ros/node/projectairsim-navigation/README.md). The bridge needs an active scene before robot pose topics and lifecycle services exist.

### ROS 1 compatibility

The ROS 1 adapter is in `ros/node/projectairsim-ros1`. On a Noetic system, install the shared bridge and ROS 1 Python packages, or place that directory in a Catkin workspace. ROS 1 exposes the established topics and conversion behaviour; the lifecycle services are a ROS 2 addition.

The bridge and mission code cannot connect as separate Project AirSim clients. To combine a mission with ROS, use `ros/node/scripts/ros1/hello_ros1.py` or `ros/node/scripts/ros2/hello_ros2.py`, which share one client instance.

---

Copyright (C) Microsoft Corporation.  
Copyright (C) 2025 IAMAI CONSULTING CORP

MIT License. All rights reserved.
