# Nav2 Autonomous Navigation for Project Ogre

Complete guide for autonomous waypoint navigation with 3D obstacle avoidance using RealSense D435 depth camera.

## Overview

This system adds autonomous navigation capabilities to the existing slam_toolbox-based SLAM system:

- **2D SLAM Mapping**: RPLIDAR A1 + slam_toolbox (existing)
- **3D Obstacle Avoidance**: RealSense D435 pointcloud
- **Waypoint Navigation**: Click-to-navigate in RViz
- **Path Planning**: Nav2 global and local planners
- **Mecanum Drive Control**: DWB controller with omnidirectional movement

## Prerequisites

### 1. Install Nav2 Packages (REQUIRED)

**You must run this installation command before using navigation:**

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-planner \
  ros-humble-nav2-controller \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-behavior-tree \
  ros-humble-nav2-dwb-controller \
  ros-humble-nav2-recoveries \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-core \
  ros-humble-dwb-plugins \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-theta-star-planner
```

### 2. Verify Installation

```bash
ros2 pkg list | grep nav2
# Should show: nav2_bringup, nav2_costmap_2d, nav2_planner, etc.
```

### 3. Hardware Requirements

- ✅ RPLIDAR A1 (already installed and working)
- ✅ RealSense D435 (already installed and working)
- ✅ Wheel encoders with calibrated odometry (gear_ratio: 224.0)
- ✅ Jetson Orin Nano with sufficient CPU/memory

## Quick Start

### Step 1: Create a Map

If you don't have a map yet, create one first using mapping mode:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ogre_slam mapping.launch.py
```

Drive around the area manually (web interface at http://10.21.21.45:8080) and save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/my_map
```

### Step 2: Launch Navigation

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml
```

This launches:
1. RPLIDAR driver
2. RealSense D435 with pointcloud
3. Wheel odometry
4. EKF sensor fusion
5. slam_toolbox (localization mode with saved map)
6. Nav2 stack (planners, controllers, costmaps)
7. RViz with Nav2 panel

### Step 3: Navigate to Waypoints

**In RViz:**

1. **Set Initial Pose** (IMPORTANT - Do this first!):
   - AMCL needs to know where the robot starts
   - Click "2D Pose Estimate" button in RViz toolbar
   - Click on the map where the robot actually is
   - Drag to set the robot's orientation (arrow direction)
   - You should see a green cloud of particles appear around the robot
   - As the robot moves, particles will converge to the correct pose

2. **Navigate to Goal**:
   - Click "Nav2 Goal" or "2D Goal Pose" button
   - Click destination on map
   - Robot will plan path and drive autonomously

3. **Monitor Progress**:
   - Green line = global plan
   - Red line = local trajectory
   - Colored costmap = obstacles (red = blocked, blue = free)

4. **Manual Override**:
   - Web interface always available at http://10.21.21.45:8080
   - Press SPACEBAR to emergency stop
   - Manual commands override autonomous navigation

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                 Sensor Inputs                           │
├─────────────────────────────────────────────────────────┤
│ RPLIDAR A1:                                             │
│   Topic: /scan                                          │
│   Type: sensor_msgs/LaserScan                          │
│   Rate: ~7-8 Hz                                        │
│   Coverage: 360° horizontal plane at robot height      │
├─────────────────────────────────────────────────────────┤
│ RealSense D435:                                         │
│   Topic: /camera/camera/depth/color/points             │
│   Type: sensor_msgs/PointCloud2                        │
│   Rate: ~20 Hz                                         │
│   Coverage: 87° H × 58° V, 0.3m-10m depth             │
├─────────────────────────────────────────────────────────┤
│ Wheel Odometry:                                         │
│   Topic: /odom                                          │
│   Type: nav_msgs/Odometry                              │
│   Rate: 50 Hz                                          │
│   Accuracy: ±0.8% (calibrated gear_ratio: 224.0)      │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│              Localization & Mapping                     │
├─────────────────────────────────────────────────────────┤
│ Map Server:                                             │
│   - Loads and publishes pre-built map (.pgm/.yaml)    │
│   - Provides static occupancy grid to Nav2             │
│                                                         │
│ AMCL (Adaptive Monte Carlo Localization):              │
│   - Particle filter localization using laser scans    │
│   - Estimates robot pose on the map                    │
│   - Publishes map → odom transform                     │
│   - Corrects position drift from odometry              │
│                                                         │
│ robot_localization EKF:                                 │
│   - Fuses wheel odometry for smooth tracking           │
│   - Publishes odom → base_link transform               │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│              Nav2 Navigation Stack                      │
├─────────────────────────────────────────────────────────┤
│ Global Costmap:                                         │
│   - Static Layer: Pre-built map from SLAM              │
│   - Obstacle Layer: RPLIDAR + RealSense pointcloud     │
│   - Inflation Layer: Safety margin around obstacles    │
│   - Size: Large (covers whole map)                     │
│   - Update: 1 Hz                                       │
├─────────────────────────────────────────────────────────┤
│ Local Costmap:                                          │
│   - Obstacle Layer: RPLIDAR + RealSense pointcloud     │
│   - Inflation Layer: Safety margin                     │
│   - Size: 3m × 3m rolling window around robot          │
│   - Update: 5 Hz (responsive obstacle avoidance)       │
├─────────────────────────────────────────────────────────┤
│ Global Planner (NavFn):                                 │
│   - Plans path from robot to goal on global costmap    │
│   - Dijkstra's algorithm (guaranteed optimal path)     │
│   - Replans when obstacles block path                  │
├─────────────────────────────────────────────────────────┤
│ Local Controller (DWB):                                 │
│   - Generates velocity commands to follow global plan  │
│   - Mecanum-aware (supports omnidirectional movement)  │
│   - Avoids dynamic obstacles using local costmap       │
│   - Publishes to /cmd_vel → motor_control_node         │
└─────────────────────────────────────────────────────────┘
```

## TF Frame Hierarchy

Complete transform tree after Nav2 integration:

```
map (world frame - from slam_toolbox)
  └─ odom (drift-corrected odometry - from EKF or slam_toolbox)
      └─ base_link (robot center)
          ├─ laser (RPLIDAR at front, 0.27m up, 180° rotated)
          ├─ camera_link (RealSense D435 at 0.15m forward, 0.10m up)
          │   ├─ camera_depth_frame
          │   │   └─ camera_depth_optical_frame (pointcloud data frame)
          │   └─ camera_color_frame
          │       └─ camera_color_optical_frame (RGB image frame)
          └─ imx477_camera_optical_frame (IMX477 Pi camera for teleop)
```

## Configuration Files

### `/home/jetson/ros2_ws/src/ogre-slam/config/nav2_params.yaml`

Main Nav2 configuration with:
- Behavior tree navigator settings
- Planner and controller parameters
- Costmap configuration references
- Recovery behavior settings

### `/home/jetson/ros2_ws/src/ogre-slam/config/costmap_common_params.yaml`

Shared costmap settings:
- **Obstacle Layer**: Combines RPLIDAR (2D) + RealSense (3D) pointcloud
  - `min_obstacle_height: 0.05m` - Ignore floor
  - `max_obstacle_height: 2.0m` - Detect up to ceiling
- **Inflation Layer**: 0.20m robot radius, 0.55m inflation radius
- **Observation Sources**: Scan (LIDAR) and pointcloud (RealSense)

## Obstacle Avoidance

### How It Works

1. **RealSense Pointcloud Processing**:
   - Receives 3D pointcloud from `/camera/camera/depth/color/points`
   - Filters points by height (5cm - 2m above ground)
   - Projects valid points into costmap

2. **RPLIDAR 2D Scan**:
   - Receives 2D laser scans from `/scan`
   - Marks obstacles at robot height

3. **Costmap Fusion**:
   - Combines LIDAR and RealSense data
   - Creates unified obstacle representation
   - Updates at 5 Hz for responsive avoidance

4. **Path Planning**:
   - Global planner finds path avoiding static obstacles
   - Local controller adjusts for dynamic obstacles
   - Robot stops if all paths blocked

### Coverage

- **RPLIDAR**: 360° horizontal at robot height (~27cm off ground)
  - Pros: Complete horizontal coverage
  - Cons: Misses obstacles above/below LIDAR plane

- **RealSense D435**: 87° H × 58° V in front of robot
  - Pros: Detects obstacles at all heights (5cm - 2m)
  - Cons: Limited field of view (only front)

**Combined**: Full 360° coverage at robot height + 3D detection in front

## Tuning Parameters

### If Robot is Too Cautious (won't go through narrow gaps)

Edit `/home/jetson/ros2_ws/src/ogre-slam/config/costmap_common_params.yaml`:

```yaml
footprint_padding: 0.01  # Reduce from 0.05
inflation_layer:
  inflation_radius: 0.45  # Reduce from 0.55
```

### If Robot Gets Too Close to Obstacles

```yaml
robot_radius: 0.25  # Increase from 0.20
inflation_radius: 0.65  # Increase from 0.55
```

### If Robot Moves Too Slowly

Edit `/home/jetson/ros2_ws/src/ogre-slam/config/nav2_params.yaml`:

```yaml
DWBLocalPlanner:
  max_vel_x: 0.5  # Increase from 0.3
  max_vel_theta: 1.5  # Increase from 1.0
```

### If Path Planning Fails Often

```yaml
NavfnPlanner:
  tolerance: 0.2  # Increase from 0.1 (allows less precise goals)
```

## Remote Visualization (Headless Operation)

If you're running the Jetson over SSH without a display, you have several options for visualizing navigation:

### Option 1: Launch Without RViz (Recommended)

Disable RViz on the Jetson to save resources:

```bash
ros2 launch ogre_slam navigation.launch.py \
  map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml \
  use_rviz:=false
```

Navigate using:
- **Web interface**: http://10.21.21.45:8080 (manual control)
- **Command line**: Send goals via ROS2 topics
- **Remote RViz**: Run RViz on another computer (see below)

### Option 2: Remote RViz from Another Computer

Run RViz on your desktop/laptop computer while the Jetson runs headless.

**On the Jetson** (launch without RViz):
```bash
ros2 launch ogre_slam navigation.launch.py \
  map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml \
  use_rviz:=false
```

**On your remote computer** (must have ROS2 Humble installed):

1. **Set ROS environment variables**:
   ```bash
   export ROS_DOMAIN_ID=42
   export ROS_LOCALHOST_ONLY=0
   ```

2. **Launch RViz with navigation config**:
   ```bash
   # If you have the ogre-slam repo cloned:
   rviz2 -d ~/ros2_ws/src/ogre-slam/rviz/navigation.rviz

   # Or create a new config manually:
   rviz2
   ```

3. **In RViz, configure displays**:
   - Add → Map → Topic: `/map`
   - Add → LaserScan → Topic: `/scan`
   - Add → Path → Topic: `/plan`
   - Add → PointCloud2 → Topic: `/camera/camera/depth/color/points`
   - Add → RobotModel (requires URDF)
   - Add → TF
   - Fixed Frame: `map`

4. **Add Nav2 tools** (top toolbar):
   - Click "+", select "rviz_default_plugins/SetInitialPose"
   - Click "+", select "rviz_default_plugins/SetGoal"

**Network Requirements**:
- Both computers on same network (e.g., 10.21.21.x)
- Firewall allows ROS2 DDS traffic
- For FastDDS (ROS2 default), ensure multicast is enabled on network
- If multicast doesn't work, use peer discovery:
  ```bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/peer_discovery.xml
  ```

**Troubleshooting Remote RViz**:
- No topics visible: Check `ROS_DOMAIN_ID` matches on both machines (42)
- Connection issues: Verify `ros2 topic list` shows topics from Jetson
- Slow performance: Reduce RViz display rates or disable PointCloud2

### Option 3: X11 Forwarding (Not Recommended)

While possible, X11 forwarding over SSH is very slow for RViz:

```bash
ssh -X jetson@10.21.21.45
ros2 launch ogre_slam navigation.launch.py map:=~/maps/my_map.yaml
```

This will be laggy and consume significant bandwidth. Use Options 1 or 2 instead.

### Option 4: Send Goals via Command Line

Control navigation without RViz using ROS2 topics:

**Set initial pose**:
```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

**Send navigation goal**:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'},
    pose: {position: {x: 2.0, y: 1.0, z: 0.0},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

**Monitor navigation status**:
```bash
# Watch current robot pose
ros2 topic echo /amcl_pose

# Check if goal reached
ros2 topic echo /navigate_to_pose/_action/status
```

## Troubleshooting

### Navigation Fails to Start

**Error**: `No path could be found`

**Solutions**:
1. Check robot localization is accurate:
   ```bash
   ros2 topic echo /amcl_pose  # Position should match RViz
   ```
2. Set initial pose in RViz (click "2D Pose Estimate")
3. Check costmap for errors:
   ```bash
   ros2 topic hz /local_costmap/costmap
   ros2 topic hz /global_costmap/costmap
   ```

### Robot Stops Unexpectedly

**Error**: Robot stops mid-navigation

**Causes**:
1. Obstacle detected by RealSense
2. Costmap updates detected new obstacle
3. Watchdog timeout (no velocity commands)

**Debug**:
```bash
# Check what's in the costmap
ros2 topic echo /local_costmap/costmap --once

# View controller output
ros2 topic echo /cmd_vel

# Check behavior tree status
ros2 topic echo /behavior_tree_log
```

### RealSense Not Detecting Obstacles

**Symptoms**: Robot drives into obstacles that RealSense should see

**Checks**:
1. **Pointcloud publishing?**
   ```bash
   ros2 topic hz /camera/camera/depth/color/points
   # Should show ~20 Hz
   ```

2. **TF frames correct?**
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
   # Should show transform without errors
   ```

3. **Costmap receiving pointcloud?**
   ```bash
   ros2 param get /local_costmap/local_costmap obstacle_layer.pointcloud.topic
   # Should be: /camera/camera/depth/color/points
   ```

4. **View pointcloud in RViz**:
   - Add PointCloud2 display
   - Topic: `/camera/camera/depth/color/points`
   - Should see 3D colored points in front of robot

### Robot Won't Move (Path Planned but No Motion)

**Causes**:
1. Motor control node not receiving `/cmd_vel`
2. Web teleop watchdog blocking commands
3. Emergency stop activated

**Solutions**:
```bash
# Check if cmd_vel is being published
ros2 topic hz /cmd_vel

# Check motor control node status
ros2 node info /motor_control_node

# Restart teleop if needed (clears watchdog)
pkill -f web_teleop_node
```

### Poor Localization (Robot Position Wrong in RViz)

**Symptoms**: Robot appears in wrong place on map

**Solutions**:
1. **Manually set pose**: Click "2D Pose Estimate" in RViz
2. **Drive around**: SLAM will correct position as it sees landmarks
3. **Check encoder calibration**:
   ```bash
   ros2 param get /odometry_node gear_ratio
   # Should be: 224.0 for 25GA-370 motors
   ```

## Performance Optimization

### Reduce CPU Load

**On Jetson**, navigation can be CPU-intensive. Optimize:

1. **Lower costmap update rates**:
   ```yaml
   global_costmap:
     update_frequency: 0.5  # From 1.0 Hz
   local_costmap:
     update_frequency: 3.0  # From 5.0 Hz
   ```

2. **Reduce pointcloud density**:
   Launch RealSense with decimation filter:
   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     enable_color:=true \
     enable_depth:=true \
     pointcloud.enable:=true \
     decimation_filter.enable:=true
   ```

3. **Disable RViz on robot**:
   ```bash
   ros2 launch ogre_slam navigation.launch.py use_rviz:=false
   # Run RViz on remote computer instead
   ```

### Monitor Resource Usage

```bash
# CPU and memory
tegrastats

# ROS node CPU usage
ros2 run ros2_benchmark benchmark_runner
```

## Future Enhancements (Phase 2)

- [ ] **RTAB-Map Integration**: Upgrade to 3D visual-inertial SLAM
- [ ] **Multi-Waypoint Missions**: Program routes, execute sequences
- [ ] **Dynamic Obstacle Prediction**: Anticipate moving obstacles
- [ ] **Advanced Recovery Behaviors**: Get unstuck from complex situations
- [ ] **IMU Integration**: Better orientation estimation
- [ ] **AprilTag Docking**: Precision docking at charging stations
- [ ] **Frontier Exploration**: Autonomous exploration of unknown areas

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [RealSense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [DWB Local Planner](https://navigation.ros.org/configuration/packages/dwb-params/index.html)

---

**Robot IP**: 10.21.21.45
**ROS2 Distro**: Humble
**Platform**: Jetson Orin Nano
**Date**: November 2025
