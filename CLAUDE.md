# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 Humble package for SLAM mapping and autonomous navigation on a Jetson Orin Nano-powered mecanum drive robot. The system uses wheel encoder odometry, RPLIDAR A1 for 2D laser scanning, RealSense D435 for 3D obstacle avoidance, and Nav2 for autonomous waypoint navigation.

**Robot Platform:** Project Ogre mecanum drive
**Hardware:** Jetson Orin Nano, RPLIDAR A1, RealSense D435, 25GA-370 motors with 2 PPR Hall sensors
**Network:** Robot IP is 10.21.21.45, ROS_DOMAIN_ID=42

## Common Commands

### Building

```bash
cd ~/ros2_ws
colcon build --packages-select ogre_slam --symlink-install
source install/setup.bash
```

### Running SLAM Mapping

```bash
# Full mapping session (launches SLAM + teleop together)
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_mapping_session.sh

# SLAM only (without teleop or RViz)
ros2 launch ogre_slam mapping.launch.py use_rviz:=false use_teleop:=false

# Save map after mapping
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/my_map
```

### Running Autonomous Navigation

```bash
# Full navigation stack (requires saved map)
ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml

# Without RViz (headless operation)
ros2 launch ogre_slam navigation.launch.py map:=~/maps/my_map.yaml use_rviz:=false
```

### Testing Components

```bash
# Test encoder reading (hardware)
cd ~/ros2_ws/src/ogre-slam
python3 ogre_slam/encoder_reader.py

# Test odometry node alone
ros2 run ogre_slam odometry_node

# Check odometry output
ros2 topic echo /odom --once
ros2 topic hz /odom  # Should be ~50 Hz

# Check encoder ticks
ros2 topic echo /encoder_ticks

# Verify TF tree
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames  # Creates frames.pdf
```

### Debugging

```bash
# Monitor all critical topics
ros2 topic list | grep -E "odom|scan|map|cmd_vel"

# Check topic rates
ros2 topic hz /odom        # ~50 Hz
ros2 topic hz /scan        # ~5-6 Hz (RPLIDAR A1)
ros2 topic hz /map         # ~0.5 Hz

# View AMCL localization pose
ros2 topic echo /amcl_pose

# Check navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Monitor Jetson performance
tegrastats
```

## Architecture

### Node Responsibilities

**Odometry Flow:**
- `odometry_node` (ogre_slam): Reads GPIO encoders → publishes `/odom` (nav_msgs/Odometry) and `odom→base_link` TF
- `ekf_filter_node` (robot_localization): Fuses `/odom` → publishes `/odometry/filtered` for smooth tracking
- `slam_toolbox` (mapping mode): Consumes scans + odometry → publishes `map→odom` TF
- `amcl` (navigation mode): Particle filter localization → publishes `map→odom` TF

**Sensor Integration:**
- `rplidar_node`: Publishes `/scan` (sensor_msgs/LaserScan) at ~7 Hz
- `realsense2_camera_node`: Publishes `/camera/camera/depth/color/points` (sensor_msgs/PointCloud2) at ~20 Hz
- Both sensors feed into Nav2 costmaps for obstacle detection

**Navigation Stack:**
- `controller_server`: DWB local planner generates `/cmd_vel` commands
- `planner_server`: NavFn global planner computes paths on map
- `behavior_server`: Recovery behaviors (backup, spin, wait)
- `bt_navigator`: Behavior tree coordinator
- Costmaps combine LIDAR + RealSense pointcloud for 2D + 3D obstacle avoidance

### TF Tree Structure

```
map (from slam_toolbox in mapping OR amcl in navigation)
  └─ odom (from ekf_filter_node OR odometry_node)
      └─ base_link (robot center at wheel axle height)
          ├─ laser (RPLIDAR: 0.0m forward, 0.30m up, 180° rotated - mounted on 65mm posts)
          ├─ front_camera (Camera: 0.15m forward, 0.10m up - Isaac Sim camera sensor)
          ├─ camera_link (RealSense D435: 0.15m forward, 0.10m up - real robot only)
          │   ├─ camera_depth_frame
          │   │   └─ camera_depth_optical_frame (pointcloud frame)
          │   └─ camera_color_frame
          └─ imx477_camera_optical_frame (RPi camera for teleop)
```

### Critical Configuration Parameters

**Odometry (config/odometry_params.yaml):**
- `gear_ratio: 224.0` - CRITICAL calibrated value for 25GA-370 motors. DO NOT change without recalibration.
- `wheel_radius: 0.040` (40mm) - Measured wheel radius
- `wheel_base: 0.095` (95mm) - Front/rear axle distance
- `track_width: 0.205` (205mm) - Left/right wheel distance
- `encoder_ppr: 2` - Hall sensor resolution (very coarse, hence EKF is essential)

**SLAM (config/slam_toolbox_params.yaml):**
- `mode: mapping` (mapping.launch.py) or `mode: localization` (navigation.launch.py)
- `resolution: 0.05` - 5cm grid for Jetson memory optimization
- `map_update_interval: 2.0` - Publish map every 2s to reduce CPU load
- `minimum_travel_distance: 0.0` and `minimum_travel_heading: 0.0` - Process scans based on time only (critical for low-resolution encoders)

**Nav2 (config/nav2_params.yaml):**
- Global costmap: Uses static layer (saved map) + obstacle layer (LIDAR + pointcloud)
- Local costmap: 3m × 3m rolling window, 5 Hz updates for responsive avoidance
- DWB controller configured for mecanum drive omnidirectional movement
- `robot_radius: 0.20` and `inflation_radius: 0.55` in costmap params

**AMCL (config/amcl_params.yaml):**
- Particle filter localization for navigation mode
- Initial pose MUST be set in RViz ("2D Pose Estimate") before first navigation
- Publishes `map→odom` transform to correct odometry drift

## File Structure

### Core Odometry Modules (ogre_slam/)
- `encoder_reader.py`: GPIO interrupt-based encoder tick counting for 4 motors (M1-M4)
- `mecanum_odometry.py`: Forward kinematics converting encoder ticks → robot velocity (vx, vy, vtheta)
- `odometry_node.py`: ROS2 node publishing `/odom` topic and TF transforms
- `dummy_odom_node.py`: Fallback static odometry for testing (NOT used in production)

### Launch Files (launch/)
- `mapping.launch.py`: SLAM mapping mode - builds new maps
  - Launches: RPLIDAR, odometry, EKF, slam_toolbox (async mapping), map_saver_server, optional RViz, optional teleop
- `navigation.launch.py`: Autonomous navigation mode - uses saved maps
  - Launches: RPLIDAR, RealSense, odometry, EKF, map_server, AMCL, Nav2 stack, optional RViz, optional teleop
  - **Requires** `map:=<path>` argument

### Configuration (config/)
- `odometry_params.yaml`: Robot physical dimensions and encoder specs
- `ekf_params.yaml`: Sensor fusion configuration (critical for 2 PPR encoders)
- `slam_toolbox_params.yaml`: SLAM settings (mapping OR localization mode)
- `amcl_params.yaml`: Particle filter localization parameters
- `nav2_params.yaml`: Complete Nav2 stack configuration (planners, controllers, behavior tree)

### Helper Scripts (scripts/)
- `launch_mapping_session.sh`: Unified launcher for SLAM + teleop (recommended for mapping on real robot)
- `launch_isaac_sim_rviz.sh`: Launch RViz for Isaac Sim visualization (uses `ROS_USE_SIM_TIME=true`)
- `remote_launch_slam_rviz.sh`: Launch RViz on remote computer for real robot mapping visualization
- `remote_launch_nav_rviz.sh`: Launch RViz on remote computer for real robot navigation visualization

**IMPORTANT:** Isaac Sim and real robot use different RViz scripts:
- **Isaac Sim:** Uses `launch_isaac_sim_rviz.sh` which sets `ROS_USE_SIM_TIME=true` to sync with simulation clock (`/clock` topic)
- **Real Robot:** Uses `remote_launch_*_rviz.sh` scripts which use system time (real time)
- These scripts do NOT interfere with each other - they're for completely separate use cases

## Important Gotchas

### Gear Ratio Calibration
The `gear_ratio: 224.0` parameter is the most critical value for accurate odometry. This was calibrated by driving exactly 1.0m and measuring reported distance. Incorrect gear ratio causes massive position errors (e.g., reporting 4.5m when actual is 1.0m) and prevents SLAM from working. DO NOT change this value unless recalibrating.

### Encoder Resolution Limitations
The 2 PPR Hall sensors provide very coarse odometry. The EKF is essential for smoothing noise. Odometry drift is expected - SLAM loop closures and AMCL localization correct this. For production, adding an IMU is recommended.

### SLAM Requires Real Odometry
slam_toolbox needs meaningful odometry changes to process scans, even in scan-matching mode. Using dummy odometry (always reporting 0,0,0) will cause the map to freeze and not update. Always use real encoder odometry.

### AMCL Initial Pose
When launching navigation mode, AMCL needs an initial pose estimate. In RViz, click "2D Pose Estimate" and set the robot's starting position/orientation before sending navigation goals. Without this, localization will fail.

### Map Frame Must Be Fixed
In RViz, the Fixed Frame should be `map` for both mapping and navigation. If you see "No tf data" warnings, check that all TF publishers are running correctly.

### GPIO Conflicts (False Alarm)
There is NO GPIO conflict between `odometry_node` and `motor_control_node`. They use different hardware:
- odometry_node: GPIO pins 7,11,13,15,29,31,32,33 (encoder reading)
- motor_control_node: I2C (PCA9685 for PWM motor control)

Both can run simultaneously. If you see "Device or resource busy" errors, it's from incomplete cleanup of a previous run. Solution: `pkill -9 python3` or reboot.

### Nav2 RealSense Integration
The RealSense D435 provides 3D obstacle avoidance. The pointcloud topic `/camera/camera/depth/color/points` is consumed by Nav2's obstacle_layer in both global and local costmaps. Height filtering (5cm-2m) ignores floor and ceiling. Ensure `pointcloud.enable:=true` in RealSense launch args.

### Mecanum Drive Kinematics
The mecanum odometry uses X-configuration forward kinematics:
```
vx = (v_fl + v_fr + v_rl + v_rr) / 4
vy = (-v_fl + v_fr + v_rl - v_rr) / 4
vtheta = (-v_fl + v_fr - v_rl + v_rr) / (4 * L)
```
where L = (wheelbase + trackwidth) / 2. Incorrect wheel dimensions will cause rotation drift.

## Integration with Other Packages

**This package is standalone** - it declares all its own dependencies and can run independently.

**Optional integration with:**
- **ogre_teleop**: Web-based manual control (http://10.21.21.45:8080). Can be launched alongside for manual override and emergency stop.
- **rplidar_ros**: LIDAR driver (separate package, launched via IncludeLaunchDescription)
- **realsense2_camera**: RealSense driver (separate package, launched for navigation only)
- **Motor control**: Any ROS2-compatible motor controller that subscribes to `/cmd_vel` and provides encoder data

## Performance Tuning for Jetson

The Jetson Orin Nano has limited resources. Optimizations applied:
- SLAM resolution: 0.05m (vs default 0.01m) → 80% memory savings
- Map update interval: 2s (vs default 1s) → Lower CPU
- Costmap update rates: Global 1Hz, Local 5Hz
- Optional: `decimation_filter.enable:=true` for RealSense to reduce pointcloud density

Monitor performance with `tegrastats`. Expected usage: 40-60% CPU, 1-3GB RAM during navigation.

## Common Development Workflows

### Adding New Odometry Source (e.g., IMU)
1. Modify `config/ekf_params.yaml` to add IMU input (example config already included as comments)
2. Update `odom0_config` to fuse IMU orientation with wheel odometry position
3. No changes needed to launch files (EKF auto-subscribes to configured topics)

### Tuning Navigation Behavior
- Too cautious: Reduce `inflation_radius` in costmap params
- Too aggressive: Increase `robot_radius`
- Too slow: Increase `max_vel_x` and `max_vel_theta` in DWB controller params
- Path planning fails: Increase `tolerance` in NavFn planner params

### Creating Maps for Different Areas
1. Launch `./scripts/launch_mapping_session.sh`
2. Drive slowly using web teleop (http://10.21.21.45:8080)
3. Ensure loop closure by returning to start
4. Save map: `ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/<area_name>`
5. Use in navigation: `ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/<area_name>.yaml`

### Remote Development (Headless Jetson)
1. On Jetson: Launch with `use_rviz:=false`
2. On dev computer: Set `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=0`
3. Run `./scripts/remote_launch_nav_rviz.sh` (navigation) or `./scripts/remote_launch_slam_rviz.sh` (mapping)
4. Both computers must be on same network (10.21.21.x subnet)

## Testing Strategy

When making changes, test incrementally:
1. Hardware: `python3 ogre_slam/encoder_reader.py` (verify GPIO)
2. Math: `python3 ogre_slam/mecanum_odometry.py` (verify kinematics)
3. ROS integration: `ros2 run ogre_slam odometry_node` + check `/odom` topic
4. SLAM: Launch mapping.launch.py and verify map updates in RViz
5. Navigation: Launch navigation.launch.py and send test goals

Expected topic rates:
- `/odom`: 50 Hz
- `/scan`: 5-6 Hz (RPLIDAR A1)
- `/map`: 0.5 Hz (every 2 seconds)
- `/camera/camera/depth/color/points`: 20 Hz

## Dependencies Installation

Required before building:
```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-dwb-plugins
```

Python dependencies (declared in setup.py):
- Jetson.GPIO (for GPIO encoder reading)
- numpy (for mecanum kinematics)
- rclpy (from ROS2 Humble)

## Isaac Sim Simulation

This package includes USD robot models for testing in NVIDIA Isaac Sim 5.0+ before deploying to hardware.

### Isaac Sim Files

- `ogre.usd`: Main Isaac Sim scene with summit_xl_omni_four prebuilt mecanum robot
- `ogre2-5.usd`: Experimental robot configurations (not tracked in git)

### Robot Physical Dimensions

**Body (Main Chassis):**
- Length (X): 0.20m (200mm)
- Width (Y): 0.16m (160mm)
- Height (Z): 0.175m (175mm)
- Position: 20mm above wheel axle (body bottom at Z=0.06m, center at Z=0.1475m)
- Mass: ~2.7kg

**Weighted Barrels (Front):**
- Two cylinders: 55mm diameter × 70mm tall
- Position: In front of body, flush to base
  - Left barrel: (0.1275, 0.04, 0.095)
  - Right barrel: (0.1275, -0.04, 0.095)
- Mass: 0.355kg each (0.71kg total)

**Battery Pack (Rear):**
- Dimensions: 55mm (depth) × 160mm (width) × 55mm (height)
- Position: (-0.1275, 0, 0.0875) - centered, behind body
- Mass: ~0.71kg (counterweight to barrels)

**Overall Robot Footprint:**
- Total Length: ~310mm (barrel front to battery back)
- Total Width: 205mm (track width)
- Total Height: 300mm (to top of LIDAR)
- Total Mass: ~4.5-5.0kg

**Wheels:**
- Radius: 0.040m (40mm)
- Width: 0.040m (40mm)
- Wheelbase: 0.095m (front-to-rear axle distance)
- Track width: 0.205m (left-to-right wheel center distance)
- Mass: ~0.1kg each

**Wheel Positions (relative to base_link at wheel axle height):**
- Front-Left (FL): (0.0475, 0.1025, 0.04)
- Front-Right (FR): (0.0475, -0.1025, 0.04)
- Rear-Left (RL): (-0.0475, 0.1025, 0.04)
- Rear-Right (RR): (-0.0475, -0.1025, 0.04)

**LIDAR Mounting:**
- 65mm posts on top of robot body (represented as 0.05×0.05×0.065m cube at Z=0.2675m)
- LIDAR frame at Z=0.30m, rotated 180° around Z axis

**Camera Mounting:**
- Isaac Sim: `front_camera` frame at (0.15, 0, 0.10) - 15cm forward, 10cm above base_link
- Real Robot: `camera_link` for RealSense D435 at same position (0.15, 0, 0.10)

**LIDAR Sensor (Isaac Sim 5.0):**
- Use **2D PhysXLidar** sensor (recommended by NVIDIA for Isaac Sim 5.0)
- Attach to `laser` frame at Z=0.30m
- Configure for RPLIDAR A1 specs:
  - Horizontal FOV: 360°
  - Min Range: 0.15m
  - Max Range: 12.0m
  - Rotation Rate: ~6 Hz

### ROS2 TF Transform Publishing

**CRITICAL for RViz and SLAM:** Isaac Sim must publish TF transforms to match the real robot's TF tree.

**Required TF Tree:**
```
odom
  └─ base_link
      └─ laser
```

**Action Graph Nodes Needed:**

**1. Publish Odometry (odom → base_link transform):**
- Add node: **ROS2 Publish Odometry**
- Configuration:
  - **robotPath**: `/World/summit_xl_omni_four` (your robot path)
  - **Topic Name**: `/odom`
  - **odomFrameId**: `odom`
  - **chassisFrameId**: `base_link`
- This publishes both `/odom` topic AND the `odom→base_link` TF transform

**2. Publish Static Transform (base_link → laser):**
- Add node: **ROS2 Publish Transform Tree** (with Static Publisher enabled)
- Configuration:
  - **Static Publisher**: True (enables /tf_static publishing)
  - **Parent Prim**: Path to base_link (e.g., `/World/summit_xl_omni_four/base_link`)
  - **Target Prims**: Array with laser path (e.g., `["/World/summit_xl_omni_four/laser"]`)
  - **Topic Name**: `/tf_static`

**Note:** This node reads transform from USD stage. Ensure `laser` Xform in Stage is positioned at:
  - **Translation** (relative to base_link): (0, 0, 0.17) - laser is 0.17m above base_link center
  - **Rotation**: 180° around Z axis (RPLIDAR orientation)

**3. Publish LaserScan with correct frame:**
- In your LIDAR ROS2 publisher node:
  - **Topic Name**: `/scan`
  - **Frame ID**: `laser`

**Execution Flow:**
```
On Playback Tick
  ├─> ROS2 Publish Odometry (publishes odom→base_link TF)
  ├─> ROS2 Publish Static Transform (publishes base_link→laser TF)
  └─> ROS2 Publish LaserScan (publishes /scan with frame_id=laser)
```

**Verify TF Tree:**
```bash
# In terminal with ROS_DOMAIN_ID=42
ros2 run tf2_tools view_frames
# Should create frames.pdf showing: odom → base_link → laser
```

Without proper TF publishing, RViz will show errors like "frame 'laser' does not exist" or drop messages.

### Mecanum Drive Action Graph Setup

The Isaac Sim robot uses an action graph for keyboard teleop with proper mecanum kinematics:

**Keyboard Controls:**
- W/S: Forward/backward (vx)
- A/D: Rotate left/right (vtheta)
- Q/E: Strafe left/right (vy)

**Mecanum Wheel Equations:**
```
wheel_fl = vx - vy - vtheta_L
wheel_fr = vx + vy + vtheta_L
wheel_rl = vx + vy - vtheta_L
wheel_rr = vx - vy + vtheta_L
```
Where L = (wheelbase + trackwidth) / 2

**Important Notes:**
- Joint names must match robot exactly (e.g., `wheel_joint_fl`, `wheel_joint_fr`, etc.)
- Velocity array order must match joint names array order in Articulation Controller
- For prebuilt robots, check actual articulation path (e.g., `/World/summit_xl_omni_four`)
- Wheel negations depend on physical wheel orientation - test and adjust if motion is inverted

### Isaac Sim Wheel Joint Configuration

Each wheel joint (RevoluteJoint) requires:
```
Body0: body (robot base)
Body1: . (wheel itself)
Axis: Y
Local Rotation 0: (0, 0, 0)
Local Rotation 1: (90, 0, 0)  ← CRITICAL for proper wheel orientation

Drive → Angular:
  Type: force
  Max Force: 1000-10000 (tune for stability)
  Target Velocity: 0 (controlled by action graph)
  Damping: 0.1-1.0 (higher = smoother motion)
  Stiffness: 0.0
```

### Common Isaac Sim Issues

**"Failed to find articulation" error:**
- Articulation Controller path is wrong or ArticulationRoot not applied
- Solution: Right-click robot root → Copy Prim Path → paste in action graph

**Inverted controls after switching robots:**
- Different wheel orientations require different velocity signs
- Solution: Test each motion, add/remove negations on wheel velocities as needed

**Robot tips over during rotation:**
- Center of mass too high or forces too aggressive
- Solution: Lower robot body, reduce Max Force, increase Damping

**Strafe doesn't work:**
- Simple cylinder wheels cannot strafe (need actual mecanum roller physics)
- Complex mecanum wheels (with rollers) work but may reduce FPS significantly