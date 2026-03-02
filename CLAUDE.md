# AVROS - Autonomous Vehicle ROS2 Platform

## Project Overview

Migration of the AV2.1-API autonomous vehicle codebase to ROS2 (Humble). Five custom packages + upstream drivers (Velodyne, RealSense, Xsens) + Nav2 + robot_localization.

**Source reference:** `~/AV2.1-API`
**Build:** `cd ~/AVROS && colcon build --symlink-install`
**Source overlay:** `source install/setup.bash`
**Target hardware:** NVIDIA Jetson Orin (Ubuntu, `ssh jetson` = 100.93.121.3 via Tailscale, user `dinosaur`)

---

## Build & Test

```bash
# Clone source dependencies (one-time setup, requires python3-vcstool)
cd ~/AVROS
vcs import src < avros.repos
# Clones src/realsense-ros/ (4.56.4) and src/xsens_mti/ (ros2 branch, includes xsens_mti_ros2_driver + ntrip)

# Build all packages (avros_msgs must build first for message generation)
colcon build --symlink-install --packages-select avros_msgs
colcon build --symlink-install

# Test static TF
ros2 launch avros_bringup sensors.launch.py
ros2 run tf2_tools view_frames

# Test actuator (bench)
ros2 launch avros_bringup actuator.launch.py
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Test localization
ros2 launch avros_bringup localization.launch.py

# Keyboard teleop (bench test)
ros2 launch avros_bringup teleop.launch.py
# Keys: i=forward, ,=backward, j=left, l=right, k=stop, q/z=speed up/down

# Phone web UI (bench test — proportional joystick + mode buttons)
ros2 launch avros_bringup webui.launch.py
# Open https://<jetson-ip>:8000 on phone (accept self-signed cert)

# Full autonomous stack
ros2 launch avros_bringup navigation.launch.py

# Send a navigation goal (click "Nav2 Goal" in RViz or use CLI)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 50.0, y: 30.0}}}}"

# Regenerate campus road graph (offline, requires osmnx)
python3 src/avros_navigation/scripts/generate_graph.py \
  --output src/avros_bringup/config/cpp_campus_graph.geojson

# E-stop
ros2 topic pub --once /avros/actuator_command avros_msgs/msg/ActuatorCommand \
  '{estop: true}'
```

---

## Workspace Structure

```
AVROS/
├── CLAUDE.md
├── avros.repos                   # vcstool manifest — source dependencies
├── requirements.txt              # pip deps: osmnx, fastapi, uvicorn, websockets
├── src/
│   ├── avros_msgs/               # ament_cmake — ActuatorCommand, ActuatorState, PlanRoute
│   ├── avros_bringup/            # ament_python — launch, config, URDF, RViz
│   │   ├── launch/               # sensors, localization, actuator, navigation, teleop, webui
│   │   ├── config/               # ekf, navsat, nav2, actuator, velodyne, realsense, xsens, webui, cyclonedds
│   │   ├── urdf/                 # avros.urdf.xacro
│   │   └── rviz/                 # avros.rviz
│   ├── avros_control/            # ament_python — actuator_node (cmd_vel → Teensy UDP)
│   ├── avros_webui/              # ament_python — webui_node (phone joystick → ActuatorCommand)
│   │   └── static/               # index.html, app.js (nipplejs joystick UI)
│   └── avros_navigation/         # ament_python — generate_graph.py (offline OSMnx → GeoJSON)
├── firmware/                     # Teensy CAN code (unchanged from AV2.1-API)
└── docs/
```

---

## Packages

| Package | Build Type | Purpose |
|---------|-----------|---------|
| `avros_msgs` | ament_cmake | ActuatorCommand.msg, ActuatorState.msg, PlanRoute.srv |
| `avros_bringup` | ament_python | Launch files, URDF, all YAML configs, RViz config |
| `avros_control` | ament_python | `actuator_node`: cmd_vel → PID → Ackermann inverse → Teensy UDP |
| `avros_webui` | ament_python | `webui_node`: phone joystick WebSocket → ActuatorCommand (direct control) |
| `avros_navigation` | ament_python | `generate_graph.py`: offline OSMnx → nav2_route GeoJSON graph tool |

No `avros_sensors` — upstream drivers used directly. Source dependencies are managed via `avros.repos` (vcstool manifest) and git-ignored. `vcs import src < avros.repos` clones `realsense-ros` (4.56.4) to `src/realsense-ros/` and the Xsens monorepo to `src/xsens_mti/` (contains both `xsens_mti_ros2_driver` and `ntrip` packages). Velodyne uses `ros-humble-velodyne` (apt).

---

## Network Inventory (192.168.13.0/24)

| IP | Device | MAC | Notes |
|----|--------|-----|-------|
| 192.168.13.10 | Jetson Orin | — | Compute platform, runs all ROS2 nodes |
| 192.168.13.11 | Velodyne VLP-16 | 60:76:88:38:0F:20 | Reconfigured from factory 192.168.1.201 |
| 192.168.13.31 | Gateway/router | — | Network gateway |
| 192.168.13.177 | Teensy (PJRC) | 04:E9:E5:1C:70:4A | Actuator MCU, UDP port 5005 |

---

## Sensors

### Velodyne VLP-16

- **Package:** `ros-humble-velodyne` (apt, official)
- **Config:** `avros_bringup/config/velodyne.yaml`
- **IP:** 192.168.13.11, port 2368
- **Topics:** `/velodyne_packets` (~32 Hz), `/velodyne_points` (~70 Hz)
- **Nodes:** `velodyne_driver_node` (raw UDP → packets), `velodyne_convert_node` (packets → PointCloud2)
- **Range filter:** min 1.0m (car body), max 50.0m
- **Verified working** — data confirmed via tcpdump and topic echo

### Intel RealSense D455

- **Package:** Built from source — `realsense-ros` 4.56.4 in `~/AVROS/src/realsense-ros/` + librealsense 2.57.6 at `/usr/local/` (built from `~/librealsense` with RSUSB backend)
- **Firmware:** 5.13.0.50 (downgraded from 5.17.0.9)
- **Serial:** 215122251311
- **USB:** 3.2
- **Config:** `avros_bringup/config/realsense.yaml`
- **Resolution:** 1280x720 @ 30fps (color + depth)
- **Features:** depth align enabled, pointcloud disabled (Nav2 uses VoxelLayer instead)
- **IMU:** Disabled (`enable_gyro: false`, `enable_accel: false`) — D455 HID/IMU fails with RSUSB backend on JetPack 6; Xsens provides IMU instead
- **Verified working** — color 30fps, depth streaming, rqt_image_view confirmed
- **Visualization:** `~/Desktop/visualize_camera.sh` on Jetson (launches camera + rqt_image_view)

### Xsens MTi-680G (IMU + GNSS)

- **Package:** `xsens_mti_ros2_driver` (build from source, ros2 branch)
- **Device ID:** 0080005BF5
- **Firmware:** 1.12.0, build 42
- **Filter:** General_RTK
- **Config:** `avros_bringup/config/xsens.yaml`
- **Port:** `/dev/ttyUSB0` at 115200 baud
- **Output rate:** 100 Hz
- **Topics:** `/imu/data`, `/gnss` (NavSatFix), plus quaternion, euler, acceleration, mag, twist, NMEA
- **GNSS lever arm:** `[0.0, 0.0, 0.0]` — TODO: measure antenna offset on vehicle
- **u-Blox platform:** Automotive (type 4)
- **Required deps:** `ros-humble-mavros-msgs`, `ros-humble-nmea-msgs` (apt)
- **Verified working** — 100Hz IMU data confirmed on /dev/ttyUSB0

### NTRIP Client (RTK Corrections)

- **Package:** `ntrip` (C++, ament_cmake — from `Xsens_MTi_ROS_Driver_and_Ntrip_Client` repo, ros2 branch)
- **Config:** `avros_bringup/config/ntrip_params.yaml`
- **Data flow:** xsens `/nmea` (GPGGA) → ntrip_client → NTRIP caster → RTCM3 → `/rtcm` → xsens MTi-680G → RTK FIXED/FLOAT
- **Topics:** subscribes `/nmea`, publishes `/rtcm`
- **Launch:** enabled by default in sensors.launch.py; disable with `enable_ntrip:=false`
- **Credentials:** edit `ntrip_params.yaml` — set `mountpoint`, `username`, `password` for your NTRIP caster
- **Default caster:** rtk2go.com:2101 (free, requires mountpoint selection)
- **Setup:** Included in `avros.repos` — `vcs import src < avros.repos` clones the full Xsens monorepo to `src/xsens_mti/`, which contains both `xsens_mti_ros2_driver` and `ntrip` packages. Then `colcon build` discovers both automatically.
- **Not tracked in git** — `src/xsens_mti/` is in `.gitignore`, built from source like `src/realsense-ros/`

---

## TF Tree

```
map                                    ← navsat_transform_node
 └── odom                              ← robot_localization EKF
      └── base_link                    ← robot_state_publisher (URDF)
           ├── imu_link                ← static (URDF) — TODO: measure mount position
           ├── velodyne                ← static (URDF) — TODO: measure mount position
           ├── camera_link             ← static (URDF) — TODO: measure mount position
           │    ├── camera_color_optical_frame  ← realsense driver
           │    └── camera_depth_optical_frame  ← realsense driver
           └── base_footprint          ← static (URDF)
```

Sensor mount positions in URDF (`avros.urdf.xacro`) are approximate — measure on real vehicle.

---

## Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller / teleop_twist_keyboard |
| `/avros/actuator_state` | `avros_msgs/ActuatorState` | actuator_node @ 20 Hz |
| `/avros/actuator_command` | `avros_msgs/ActuatorCommand` | webui_node (direct control) / e-stop |
| `/plan_route` | `nav2_msgs/action/ComputeRoute` | route_server (nav2_route) |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | velodyne_convert_node (~70 Hz) |
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | velodyne_driver_node (~32 Hz) |
| `/imu/data` | `sensor_msgs/Imu` | xsens_mti_node (100 Hz) |
| `/gnss` | `sensor_msgs/NavSatFix` | xsens_mti_node |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (robot_localization) |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | realsense2_camera_node |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera_node |
| `/nmea` | `nmea_msgs/Sentence` | xsens_mti_node (GPGGA ~4 Hz) |
| `/rtcm` | `mavros_msgs/RTCM` | ntrip_client (RTCM3 corrections) |

---

## Messages

### ActuatorCommand.msg
```
std_msgs/Header header
bool estop
float32 throttle      # 0.0-1.0
string mode           # N, D, S, R
float32 brake         # 0.0-1.0
float32 steer         # -1.0-1.0 (normalized)
```

### ActuatorState.msg
```
std_msgs/Header header
bool estop
float32 throttle
string mode
float32 brake
float32 steer
bool watchdog_active
```

### PlanRoute.srv
```
float64 destination_lat
float64 destination_lon
---
bool success
string message
float64 distance_meters
uint32 num_waypoints
```

---

## Actuator Control

### UDP Protocol

Teensy at `192.168.13.177:5005`. Watchdog: 500 ms. Keepalive at 200 ms.

```
A E=0 T=0.500 M=D B=0.000 S=0.100    # all-in-one command
→ {"e":0,"t":0.500,"m":"D","b":0.000,"s":0.100,"w":1}
```

**Verified working** — Teensy UDP communication confirmed. Mode switching (N to D) works, e-stop works, watchdog active.

### Control Priority (actuator_node)

The actuator node has a 3-way priority system:
1. **Fresh ActuatorCommand** (< timeout) → direct control, skip PID (used by webui)
2. **Fresh cmd_vel** (< timeout) → PID speed control (used by Nav2/teleop)
3. **Neither** → brake to stop

This enables seamless handoff: when webui stops publishing, timeout expires and Nav2's cmd_vel takes over automatically.

---

## Vehicle Parameters

- Wheelbase: 1.23 m
- Track width: 0.9 m
- Max steering: 28 deg (0.489 rad)
- Min turning radius: 2.31 m
- Steering sign: -1 (hardware convention)
- PID: Kp=0.55, Ki=0.055, Kd=0.08
- Max throttle: 0.6 (actuator_node), 0.55 (webui safety limit)
- Robot radius: 0.8 m, inflation: 0.7 m

---

## Nav2 Config

- **Route Server:** nav2_route with GeoJSON campus road graph (52 nodes, 113 edges)
- **Planner:** SmacPlannerHybrid (DUBIN, min radius 2.31 m) — fallback for off-graph planning
- **Controller:** Regulated Pure Pursuit (lookahead 3-20 m)
- **BT:** `navigate_route_graph.xml` — ComputeRoute → FollowPath (no spin/backup recovery)
- **Local costmap:** VoxelLayer (LiDAR) + InflationLayer, 10x10 m
- **Global costmap:** ObstacleLayer + InflationLayer, 100x100 m rolling
- **Goal tolerance:** 2.0 m xy, 0.5 rad yaw
- **Datum:** 34.059270, -117.820934 (fixed in navsat.yaml, used by route graph)

---

## Web UI (avros_webui)

Phone-based joystick controller for bench testing. FastAPI + WebSocket + nipplejs.

- **Launch:** `ros2 launch avros_bringup webui.launch.py`
- **URL:** `https://<jetson-ip>:8000` (self-signed cert required for phone WebSocket)
- **Control path:** phone joystick → WebSocket → webui_node → `/avros/actuator_command` → actuator_node → Teensy UDP
- **Priority:** ActuatorCommand (direct) takes precedence over cmd_vel (PID). When webui stops publishing, timeout expires and Nav2's cmd_vel takes over.
- **Safety:** WebSocket disconnect → e-stop published automatically
- **Features:** proportional joystick (throttle/brake/steer), E-STOP button, drive mode buttons (N/D/S/R), live telemetry from ActuatorState
- **Config:** `avros_bringup/config/webui_params.yaml` — port, SSL paths, max_throttle
- **SSL setup:** `mkdir -p ~/avros_certs && openssl req -x509 -newkey rsa:2048 -keyout ~/avros_certs/key.pem -out ~/avros_certs/cert.pem -days 365 -nodes -subj '/CN=AVROS'`
- **Pip deps:** `pip install fastapi uvicorn[standard] websockets`
- **On Jetson:** SSL cert paths set in webui_params.yaml to `/home/dinosaur/avros_certs/{cert,key}.pem`

---

## Launch Files

| Launch File | What it starts |
|-------------|---------------|
| `sensors.launch.py` | robot_state_publisher + velodyne driver/convert + realsense + xsens + ntrip (conditional) |
| `actuator.launch.py` | actuator_node only |
| `teleop.launch.py` | actuator_node + teleop_twist_keyboard |
| `webui.launch.py` | actuator_node + webui_node |
| `localization.launch.py` | EKF + navsat_transform |
| `navigation.launch.py` | Full stack: sensors + localization + Nav2 + route_server |

---

## Config Files

| Config | Used By |
|--------|---------|
| `actuator_params.yaml` | actuator_node — Teensy IP/port, PID gains, vehicle geometry |
| `velodyne.yaml` | velodyne_driver_node + velodyne_convert_node |
| `realsense.yaml` | realsense2_camera_node |
| `xsens.yaml` | xsens_mti_node — IMU/GNSS, lever arm, output rate |
| `webui_params.yaml` | webui_node — port, SSL, max throttle |
| `ekf.yaml` | robot_localization EKF |
| `navsat.yaml` | navsat_transform_node |
| `nav2_params.yaml` | Nav2 (planner, controller, costmaps, BT, route_server) |
| `navigate_route_graph.xml` | BT tree using ComputeRoute (nav2_route) instead of ComputePathToPose |
| `cpp_campus_graph.geojson` | Pre-built CPP campus road graph for nav2_route (map-frame coords) |
| `cyclonedds.xml` | CycloneDDS config — shared memory disabled, socket buffer 10MB |
| `ntrip_params.yaml` | ntrip_client — NTRIP caster host, port, mountpoint, credentials |

---

## DDS Config

CycloneDDS (`cyclonedds.xml`):
- Socket receive buffer: 10 MB minimum
- Shared memory: **disabled** (iceoryx RouDi daemon not running; `<SharedMemory><Enable>false</Enable></SharedMemory>`)
- Network: auto-detect interface
- Set via: `CYCLONEDDS_URI=file://<path>` in sensors.launch.py
- **Must also set** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (defaults to FastDDS otherwise)

---

## Ported Code

| AV2.1-API Source | AVROS Destination |
|------------------|-------------------|
| `actuators/udp.py` | `avros_control/actuator_node.py` (UDP protocol) |
| `control/pid.py` | `avros_control/actuator_node.py` (PID class) |
| `control/ackermann_vehicle.py` | `avros_control/actuator_node.py` (Ackermann inverse) |
| `planning/navigator.py` | `nav2_route` route_server + `avros_navigation/scripts/generate_graph.py` |
| `config/default.yaml` | Split into per-component YAML in `avros_bringup/config/` |
| `webui/server_standalone.py` | `avros_webui/webui_node.py` (ROS2 ActuatorCommand instead of raw UDP) |
| `webui/static/` | `avros_webui/static/` (voice features removed) |

---

## Replaced by Upstream

| AV2.1-API | Upstream Package |
|-----------|-----------------|
| `sensors/xsens_receiver.py` | `xsens_mti_ros2_driver` |
| `sensors/lidar_interface.py` | `velodyne` |
| `sensors/camera_interface.py` | `realsense2_camera` |
| `perception/occupancy_grid.py` | `nav2_costmap_2d` VoxelLayer |
| `perception/costmap.py` | `nav2_costmap_2d` InflationLayer |
| `control/pure_pursuit.py` | Nav2 Regulated Pure Pursuit |
| `planning/dwa.py` | Nav2 SmacPlannerHybrid |
| `runner_*.py` | Nav2 Behavior Trees + launch files |

---

## Known Issues & Fixes

| Issue | Fix |
|-------|-----|
| `rclpy.time.Time()` clock type mismatch | Use `rclpy.time.Time(clock_type=self.get_clock().clock_type)` |
| Starlette StaticFiles 404 with `--symlink-install` | Add `follow_symlink=True` to `StaticFiles()` |
| sensors.launch.py xacro YAML parse error (Humble) | Wrap in `ParameterValue(Command([...]), value_type=str)` |
| Port 8000 held after webui crash/disconnect | `fuser -k 8000/tcp` before relaunch |
| RealSense D455 `bad_optional_access` crash | Downgrade FW to 5.13.0.50 + use RSUSB backend (see RealSense Setup Guide below) |
| realsense-ros 4.57.6 compile error (`RS2_STREAM_SAFETY`) | Use 4.56.4 — 4.57.x adds D457 safety features not in librealsense 2.57.6 |
| realsense-ros compiled version mismatch warning | Apt `ros-humble-librealsense2` headers shadow `/usr/local/include/` headers — remove apt package (see below) |
| RealSense USB interface busy on relaunch | `pkill -f realsense2_camera_node` + wait 2s before relaunching |
| `control_transfer returned error` warnings | Normal with RSUSB backend on JetPack 6 — non-fatal, does not affect streaming |
| `No HID info provided, IMU is disabled` | Expected — D455 HID/IMU not available with RSUSB backend; use Xsens for IMU |
| `rgb_camera.power_line_frequency` range error | D455 FW 5.13.0.50 supports range [0,2] but driver sends 3 — cosmetic, no effect |
| Xsens driver package name | Correct name is `xsens_mti_ros2_driver` (not `xsens_ros_mti_driver`) |
| Xsens driver missing deps | `ros-humble-mavros-msgs` and `ros-humble-nmea-msgs` must be installed via apt |
| CycloneDDS iceoryx/RouDi errors on launch | SharedMemory must be disabled in `cyclonedds.xml` unless RouDi daemon is running — set `<SharedMemory><Enable>false</Enable></SharedMemory>` |
| numpy binary incompatibility on Jetson | Pin `numpy<2` — numpy 2.x breaks system matplotlib/scipy on JetPack 6 |
| Camera topics have double prefix | Topics are at `/camera/camera/...` (e.g. `/camera/camera/color/image_raw`) due to `camera_name:=camera` config — both namespace and node name are "camera" |
| RMW_IMPLEMENTATION not set | Must export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in addition to `CYCLONEDDS_URI` — defaults to FastDDS otherwise |
| NTRIP client no data | Verify credentials and mountpoint in `ntrip_params.yaml`; check internet access from Jetson; try `enable_ntrip:=false` to isolate |
| NTRIP `mountpoint` still `CHANGE_ME` | Edit `ntrip_params.yaml` — pick a nearby mountpoint from your caster (e.g. rtk2go.com mount list) |

---

## RealSense D455 Setup Guide (Jetson Orin, JetPack 6)

This documents the full procedure to get the RealSense D455 working on the Jetson Orin running JetPack 6 (R36.x, kernel 5.15-tegra). The standard apt packages do not work due to HID/V4L2 incompatibilities.

### Problem

The D455 crashes with `std::bad_optional_access` when launched via the ROS2 node. Root cause: firmware 5.16+ presents HID descriptors that the RSUSB userspace backend cannot handle on JetPack 6. The `ds-motion-common.cpp` code logs "No HID info provided, IMU is disabled" then a `std::optional::value()` call on an empty optional throws, crashing the entire device initialization.

References:
- [librealsense #14169](https://github.com/IntelRealSense/librealsense/issues/14169) — D455 + RSUSB + FW 5.16+
- [librealsense #13341](https://github.com/IntelRealSense/librealsense/issues/13341) — JetPack 6 removed HID/hidraw
- [realsense-ros #3416](https://github.com/realsenseai/realsense-ros/issues/3416) — exact crash report

### Step 1: Build librealsense 2.57.6 from source with RSUSB

```bash
cd ~/librealsense
mkdir -p build && cd build
cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DBUILD_WITH_CUDA=true
make -j6    # -j6 not -j8 to avoid OOM on Orin
sudo make install
sudo ldconfig
```

Installs to `/usr/local/lib/` and `/usr/local/include/`.

### Step 2: Downgrade D455 firmware to 5.13.0.50

```bash
# Download firmware
wget https://librealsense.intel.com/Releases/RS4xx/FW/D4XX_FW_Image-5.13.0.50.bin -O ~/D4XX_FW_Image-5.13.0.50.bin

# Flash (camera must be connected via USB)
rs-fw-update -f ~/D4XX_FW_Image-5.13.0.50.bin

# Verify
rs-enumerate-devices --compact
# Should show: Intel RealSense D455  5.13.0.50  USB 3.2
```

### Step 3: Remove apt librealsense (prevents header/library conflicts)

The apt package `ros-humble-librealsense2` installs v2.56.4 headers at `/opt/ros/humble/include/librealsense2/` which shadow the correct v2.57.6 headers at `/usr/local/include/librealsense2/`. CMake's ament include path ordering puts apt headers first, causing realsense-ros to compile with stale version strings even when `-Drealsense2_DIR` points to the local build.

```bash
sudo apt remove ros-humble-librealsense2 ros-humble-librealsense2-dbgsym
# This also removes ros-humble-realsense2-camera (apt version) — we use source build anyway
```

After removal, only `/usr/local/` provides librealsense2 headers and libraries.

### Step 4: Build realsense-ros 4.56.4 from source

```bash
# Clone via vcstool (preferred) or manually:
cd ~/AVROS
vcs import src < avros.repos   # clones realsense-ros 4.56.4 + xsens_mti

colcon build --symlink-install \
  --packages-select realsense2_camera_msgs realsense2_description realsense2_camera \
  --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
source install/setup.bash
```

**Why 4.56.4 and not 4.57.6?** Tag 4.57.6 adds D457 safety camera features (`RS2_STREAM_SAFETY`, `RS2_STREAM_LABELED_POINT_CLOUD`, `RS2_STREAM_OCCUPANCY`) that don't exist in librealsense 2.57.6 — the build fails with undeclared identifier errors. Tag 4.56.4 requires `find_package(realsense2 2.56)` which is satisfied by 2.57.6.

### Step 5: Verify

```bash
source /opt/ros/humble/setup.bash
source ~/AVROS/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera enable_color:=true enable_depth:=true \
  enable_gyro:=false enable_accel:=false

# Expected output:
#   Built with LibRealSense v2.57.6
#   Running with LibRealSense v2.57.6
#   Device Name: Intel RealSense D455
#   Device FW version: 5.13.0.50
#   RealSense Node Is Up!
```

### Pitfalls encountered

1. **Stale build artifacts** — After checking out a new librealsense version, `make` can use old `.o` files. Always `rm -rf build/` or `make clean` before rebuilding.
2. **CMake finding apt before local** — Even with `-Drealsense2_DIR=/usr/local/lib/cmake/realsense2`, the ament build system adds `-isystem /opt/ros/humble/include` before the local include path. The compiler finds apt headers first. Removing the apt package is the only reliable fix.
3. **`--allow-overriding` flag** — Colcon on Humble doesn't support this flag. Not needed if the apt realsense2_camera package is removed.
4. **Symlink conflicts in install/** — If rebuilding after a failed build, stale symlinks can cause "File exists" errors. Fix: `rm -rf build/<pkg> install/<pkg>` before rebuilding.
5. **USB interface busy** — A previous camera node holds the USB interface. Always `pkill -f realsense2_camera_node && sleep 2` before relaunching.

---

## TODOs

- [ ] Measure physical sensor mount positions on vehicle (URDF imu_link, velodyne, camera_link)
- [ ] Calibrate GNSS lever arm in xsens.yaml (antenna offset from IMU)
- [x] Test full sensors.launch.py (all sensors together) — DONE, all 3 sensors working: camera 30fps, velodyne ~20Hz, IMU 100Hz
- [x] Verify RealSense D455 camera working (FW 5.13.0.50, librealsense 2.57.6, realsense-ros 4.56.4)
- [x] Verify Xsens MTi-680G on /dev/ttyUSB0 — DONE, device ID 0080005BF5, FW 1.12.0, 100Hz IMU
- [ ] Test localization stack (EKF + navsat)
- [ ] Test full Nav2 navigation stack
- [ ] Commit SSL cert paths for Jetson (currently only set locally)
- [ ] Configure NTRIP credentials in ntrip_params.yaml (mountpoint, username, password)
- [ ] Run `vcs import src < avros.repos` on Jetson to standardize source deps (replaces old `src/xsens_ros_mti_driver/` with `src/xsens_mti/`)
- [ ] Verify RTK FIXED/FLOAT status with NTRIP corrections enabled
