# AVROS Perception Integration

## Summary

This branch adds the road-segmentation and obstacle-bridge workflow into `AVROS` so that segmentation output can be consumed by AVROS/Nav2.

The integration was kept minimal:

- Added a new ROS 2 Python package: `src/avros_perception`
- Wired that package into `avros_bringup`
- Extended Nav2 costmap configuration so `/obstacle_points` is treated as an obstacle source
- Added a hardware-free local test path so the integration could be validated before commit

## What Was Added

### New package

The new package is:

- `src/avros_perception`

It contains:

- `avros_perception/seg_demo_node.py`
  - Runs the road-segmentation model
  - Publishes:
    - `/seg_ros/input_image`
    - `/seg_ros/overlay_image`
    - `/seg_ros/drivable_mask`
    - `/seg_ros/lane_mask`
    - `/seg_ros/detections`

- `avros_perception/detections_to_obstacles.py`
  - Subscribes to `/seg_ros/detections`
  - Converts detections into `sensor_msgs/PointCloud2`
  - Publishes `/obstacle_points`

- `launch/perception.launch.py`
  - Starts both the segmentation node and the obstacle bridge

### AVROS bringup changes

The following AVROS files were updated:

- `src/avros_bringup/launch/navigation.launch.py`
- `src/avros_bringup/launch/localization.launch.py`
- `src/avros_bringup/launch/sensors.launch.py`
- `src/avros_bringup/config/nav2_params.yaml`
- `src/avros_bringup/config/nav2_params_humble.yaml`
- `src/avros_bringup/package.xml`

## What Those Changes Do

### navigation.launch.py

Added perception support to AVROS navigation bringup.

New launch controls added:

- `enable_perception`
- `perception_venv_python`
- `perception_project_root`
- `perception_image_dir`
- `perception_weights_path`
- `perception_device`
- `perception_publish_rate_hz`

Also added local test controls:

- `enable_localization`
- `enable_mock_tf`
- `enable_foxglove`
- `use_cyclonedds`

This allows AVROS to:

- start Nav2
- optionally include perception
- optionally skip hardware-dependent localization
- optionally provide static TF for local validation

### localization.launch.py and sensors.launch.py

Added hardware gating so local validation does not require the full vehicle stack.

Added:

- `enable_xsens`
- `use_cyclonedds`

This makes it possible to avoid failures on machines that do not have:

- Xsens driver support
- CycloneDDS installed

### nav2_params.yaml and nav2_params_humble.yaml

Updated Nav2 costmaps so `/obstacle_points` is consumed as an obstacle source.

For Jazzy:

- local costmap voxel layer now includes `obstacle_points`
- global costmap obstacle layer now includes `obstacle_points`

For Humble:

- added obstacle/voxel layer support for `obstacle_points`

This is the key AVROS/Nav2 compatibility step.

## Runtime Assumptions

The perception package currently uses the already-working local segmentation runtime instead of trying to vendor the full model stack into AVROS.

That means it currently expects:

- segmentation project root:
  - `/home/alexander/Desktop/seg`
- segmentation weights:
  - `/home/alexander/Desktop/seg/data/weights/yolopv2.pt`
- segmentation image folder:
  - `/home/alexander/Desktop/seg/img`
- Python runtime with model dependencies:
  - `/home/alexander/github/av-perception/.venv/bin/python`

This was intentional to avoid overengineering and to keep the proven runtime intact.

## What Was Tested

## 1. Standalone package validation

Built:

- `avros_perception`

Verified:

- `/seg_ros/detections` publishes
- `/obstacle_points` publishes

## 2. Integrated AVROS validation

Built:

- `avros_bringup`

Then launched AVROS in an isolated ROS environment using:

- a separate `ROS_DOMAIN_ID`
- perception enabled
- localization disabled
- mock TF enabled
- hardware-specific nodes disabled

### Test launch settings used

- `enable_velodyne:=false`
- `enable_realsense:=false`
- `enable_ntrip:=false`
- `enable_xsens:=false`
- `enable_foxglove:=false`
- `enable_localization:=false`
- `enable_mock_tf:=true`
- `enable_perception:=true`
- `use_cyclonedds:=false`
- `perception_device:=cpu`

### Verified in the isolated AVROS test environment

- `/controller_server` reached `active [3]`
- `/planner_server` reached `active [3]`
- `/seg_ros/detections` published messages
- `/obstacle_points` published `PointCloud2`
- `/local_costmap` subscribed to `/obstacle_points`
- `/global_costmap` subscribed to `/obstacle_points`
- RViz opened against the isolated test environment

## Problems Encountered and Fixes

### 1. AVROS bringup build was blocked by optional machine dependencies

Issues encountered:

- `ntrip` package not installed locally
- `foxglove_bridge` not installed locally
- `xsens_mti_ros2_driver` not installed locally
- `rmw_cyclonedds_cpp` not installed locally

Fixes:

- added launch gating for optional runtime features
- removed the hard local build dependency on `ntrip` so `avros_bringup` could be built and tested with `enable_ntrip:=false`

### 2. Full Nav2 activation initially failed without localization

Problem:

- Nav2 local costmap could not activate because `base_link -> odom` was missing

Fix:

- added `enable_mock_tf` test mode
- publishes:
  - `map -> odom`
  - `odom -> base_link`

This is for local proof only and avoids pretending hardware localization exists when it does not.

### 3. Perception shutdown threw `rclpy.shutdown()` errors on Ctrl-C

Problem:

- both Python nodes could hit `rcl_shutdown already called`

Fix:

- guarded shutdown with `if rclpy.ok():`

## Why It Was Put In AVROS

The work was added to `AVROS`, not `av-perception`, because the goal was AVROS compatibility.

That means:

- the reusable perception ROS package belongs under `AVROS/src`
- Nav2 wiring belongs under `AVROS/src/avros_bringup`

So the new code is split between:

- `src/avros_perception`
- `src/avros_bringup`

## Current State

This branch provides a working AVROS-compatible perception integration and a repeatable local validation path.

What is proven:

- perception runs inside AVROS bringup
- detections flow into `/obstacle_points`
- Nav2 costmaps subscribe to the obstacle topic
- AVROS Nav2 lifecycle can be brought active in the isolated test environment

What is not claimed by this branch:

- a fully live vehicle hardware deployment
- a self-contained vendored model runtime
- a replacement for real localization sensors

## Branch and PR

Pushed branch:

- `codex/avros-perception-nav2`

Draft PR:

- `https://github.com/arassal/AVROS/pull/1`
