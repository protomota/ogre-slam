# Isaac Sim Setup Guide

This guide covers setting up and running the Project Ogre robot in NVIDIA Isaac Sim 5.0.

## Overview

Isaac Sim allows you to test SLAM mapping and Nav2 navigation before deploying to the real Jetson-based robot.

**Host Computer Requirements:**
- NVIDIA GPU with RTX support
- NVIDIA Isaac Sim 5.0+
- ROS2 Humble
- This package (`ogre_slam`) installed

## Robot Model (ogre.usd)

The Isaac Sim robot model is located at `usds/ogre.usd`.

### USD Stage Hierarchy

```
Ogre
├── omniwheel_fr_fixed
│   └── omniwheel_fr
├── omniwheel_rr_fixed
│   └── omniwheel_rr
├── omniwheel_rl_fixed
│   └── omniwheel_rl
├── omniwheel_fl_fixed
│   └── omniwheel_fl
└── base_link
    ├── barrel_fl
    ├── barrel_fr
    ├── battery
    ├── laser
    │   └── Lidar (PhysX Lidar sensor)
    ├── camera_link
    │   ├── front_camera (Camera sensor)
    │   └── camera_depth (optical frame for pointcloud)
    ├── chassis
    ├── camera_body
    └── lidar_body
```

### TF Frame Tree

```
odom (from Isaac Sim odometry)
  └── base_link (robot center at wheel axle height)
      ├── laser (LIDAR: 0.0m forward, 0.30m up, 180° rotated)
      ├── camera_link (camera body: 0.15m forward, 0.10m up)
      │   └── camera_depth (optical frame with -90°/-90° rotation)
      └── [other visual meshes]
```

**Note:** Unlike the real robot, Isaac Sim publishes `odom → base_link` directly. The `map` frame is added by slam_toolbox during SLAM.

### Physical Dimensions

| Component | Dimensions | Notes |
|-----------|------------|-------|
| **Body (chassis)** | 200mm × 160mm × 175mm | Main robot body |
| **Wheels** | 40mm radius, 40mm width | Mecanum omniwheels |
| **Wheelbase** | 95mm | Front-to-rear axle distance |
| **Track Width** | 205mm | Left-to-right wheel centers |
| **Total Footprint** | ~310mm × 205mm × 300mm | Including all components |
| **LIDAR Height** | 300mm above base_link | On 65mm posts |
| **Camera Position** | 150mm forward, 100mm up | Front-facing depth camera |

### Wheel Positions (relative to base_link)

| Wheel | Position (X, Y, Z) |
|-------|-------------------|
| Front-Left (FL) | (0.0475, 0.1025, 0.04) |
| Front-Right (FR) | (0.0475, -0.1025, 0.04) |
| Rear-Left (RL) | (-0.0475, 0.1025, 0.04) |
| Rear-Right (RR) | (-0.0475, -0.1025, 0.04) |

## ROS2 Configuration

### Domain ID

**CRITICAL:** Isaac Sim must use `ROS_DOMAIN_ID=42` to communicate with ROS2 nodes.

In your action graph, configure the **ROS2 Context** node:
- Set `domain_id` to `42`

### Required Action Graphs

The robot needs these action graphs for ROS2 integration:

#### 1. ROS2_LidarStream
Publishes laser scan data.
- **Node:** ROS2 Publish LaserScan
- **Topic:** `/scan`
- **Frame ID:** `laser`

#### 2. ROS2_MecanumDrive
Handles robot control and TF publishing.

**Nodes:**
- **ROS2 Subscribe Twist** → `/cmd_vel` (robot velocity commands)
- **Isaac Compute Odometry** → Calculates odometry from wheel motion
- **ROS2 Publish Odometry** → `/odom` topic
- **ROS2 Publish Raw Transform Tree** → `odom → base_link` TF
- **ROS2 Publish Transform Tree** → `base_link → laser`, `base_link → camera_link` TF (static)
- **ROS2 Publish Clock** → `/clock` topic (for use_sim_time)

#### 3. ROS2_RealSense (Camera)
Publishes RGB and depth data.

**Nodes:**
- **ROS2 Camera Helper** (type: `rgb`) → `/rgb_camera` topic, frame_id: `camera_link`
- **ROS2 Camera Helper** (type: `depth_pcl`) → `/camera_points` topic, frame_id: `camera_depth`
- **ROS2 Publish Transform Tree** → `camera_link → camera_depth` TF

### camera_depth Optical Frame

The depth pointcloud requires a rotated optical frame to convert from camera coordinates (Z forward) to ROS coordinates (X forward).

**camera_depth Xform settings:**
- **Parent:** camera_link
- **Translation:** (0, 0, 0)
- **Rotation:** X: -90°, Y: 0°, Z: -90°

This rotation converts:
- Camera Z (depth) → ROS X (forward)
- Camera X (right) → ROS Y (left)
- Camera Y (down) → ROS Z (up)

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | 2D LIDAR scan |
| `/odom` | nav_msgs/Odometry | Robot odometry |
| `/clock` | rosgraph_msgs/Clock | Simulation time |
| `/rgb_camera` | sensor_msgs/Image | RGB camera image |
| `/camera_points` | sensor_msgs/PointCloud2 | Depth camera pointcloud |
| `/tf` | tf2_msgs/TFMessage | Dynamic transforms |
| `/tf_static` | tf2_msgs/TFMessage | Static transforms |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## Mecanum Drive Kinematics

The action graph converts `/cmd_vel` to individual wheel velocities:

```
L = (wheelbase + track_width) / 2 = (0.095 + 0.205) / 2 = 0.15

wheel_fl =  (vx - vy - vtheta * L)
wheel_fr = -(vx + vy + vtheta * L)  # Negated (right side)
wheel_rl =  (vx + vy - vtheta * L)
wheel_rr = -(vx - vy + vtheta * L)  # Negated (right side)
```

**Joint names:** `fl_joint`, `fr_joint`, `rl_joint`, `rr_joint`

## Running SLAM Mapping

1. **Start Isaac Sim** and load `usds/ogre.usd`

2. **Press Play** (▶️) to start simulation

3. **Launch SLAM** on Host Computer:
   ```bash
   cd ~/ros2_ws && source install/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 launch ogre_slam mapping.launch.py use_sim_time:=true
   ```

4. **Launch teleop** in another terminal:
   ```bash
   export ROS_DOMAIN_ID=42
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

5. **Drive around** to build the map

6. **Save map**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/isaac_sim_map
   ```

## Running Navigation

1. **Start Isaac Sim** and load `usds/ogre.usd`

2. **Press Play** (▶️)

3. **Launch Navigation**:
   ```bash
   cd ~/ros2_ws && source install/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 launch ogre_slam navigation.launch.py \
     map:=~/ros2_ws/src/ogre-slam/maps/isaac_sim_map.yaml \
     use_sim_time:=true
   ```

4. **Set initial pose** in RViz ("2D Pose Estimate")

5. **Send navigation goal** ("Nav2 Goal")

## Visualizing in RViz

Use the Isaac Sim RViz launcher script:

```bash
./scripts/launch_isaac_sim_rviz.sh
```

This launches RViz with:
- `use_sim_time:=true` for clock synchronization
- Pre-configured displays for LaserScan, TF, Map, PointCloud

## Troubleshooting

### No /scan data
- Check that Isaac Sim is playing (not paused)
- Verify ROS_DOMAIN_ID=42 in ROS2 Context node
- Check Lidar sensor is enabled in stage hierarchy

### No /odom or TF
- Verify ROS2 Publish Odometry node is connected
- Check ROS2 Publish Raw Transform Tree is publishing `odom → base_link`
- Ensure Isaac Read Simulation Time is connected to timestamp inputs

### Pointcloud pointing wrong direction
- Verify `camera_depth` Xform has correct rotation (-90°, 0°, -90°)
- Check Camera Helper frame_id is set to `camera_depth`

### RViz "extrapolation into the future" errors
- Ensure `/clock` topic is being published
- Verify RViz is launched with `use_sim_time:=true`
- Increase queue depth in RViz display settings if needed

### Robot not moving with /cmd_vel
- Check ROS2 Subscribe Twist node is connected
- Verify Articulation Controller has correct robot path
- Check joint names match: `fl_joint`, `fr_joint`, `rl_joint`, `rr_joint`
- Ensure wheel velocity signs are correct (right wheels negated)

## File Locations

| File | Description |
|------|-------------|
| `usds/ogre.usd` | Main robot scene |
| `usds/ogre_stable.usd` | Wide-base variant (340mm track) |
| `rviz/slam_universal.rviz` | RViz config for Isaac Sim |
| `scripts/launch_isaac_sim_rviz.sh` | RViz launcher with sim time |
| `maps/isaac_sim_map.yaml` | Saved simulation maps |
