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

**Real Robot (Jetson):**
```
map (from slam_toolbox in mapping OR amcl in navigation)
  └─ odom (from ekf_filter_node OR odometry_node)
      └─ base_link (robot center at wheel axle height)
          ├─ laser (RPLIDAR: 0.0m forward, 0.30m up, 180° rotated)
          └─ camera_link (RealSense D435: 0.15m forward, 0.10m up)
              ├─ camera_depth_frame
              │   └─ camera_depth_optical_frame (pointcloud frame)
              └─ camera_color_frame
```

**Isaac Sim (Host Computer):**
```
odom (from Isaac Sim odometry)
  └─ base_link (robot center at wheel axle height)
      ├─ laser (LIDAR: 0.0m forward, 0.30m up, 180° rotated)
      └─ camera_link (camera: 0.15m forward, 0.10m up)
          └─ camera_depth (optical frame for pointcloud)
```
Note: In Isaac Sim, `map` frame is added by slam_toolbox during SLAM mapping.

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

```
ogre-slam/
├── CLAUDE.md                    # This file - Claude Code guidance
├── README.md                    # Main documentation
├── package.xml                  # ROS2 package manifest
├── setup.py / setup.cfg         # Python package setup
│
├── ogre_slam/                   # Core Python modules
│   ├── encoder_reader.py        # GPIO interrupt-based encoder tick counting (M1-M4)
│   ├── mecanum_odometry.py      # Forward kinematics: ticks → velocity (vx, vy, vtheta)
│   ├── odometry_node.py         # ROS2 node: /odom topic + TF transforms
│   └── dummy_odom_node.py       # Fallback static odometry (NOT for production)
│
├── launch/                      # ROS2 launch files
│   ├── mapping.launch.py        # SLAM mapping mode - builds new maps
│   └── navigation.launch.py     # Nav2 autonomous navigation - uses saved maps
│
├── config/                      # Configuration files
│   ├── odometry_params.yaml     # Robot dimensions, encoder specs
│   ├── ekf_params.yaml          # Sensor fusion (critical for 2 PPR encoders)
│   ├── slam_toolbox_params.yaml # SLAM settings (mapping/localization mode)
│   ├── amcl_params.yaml         # Particle filter localization
│   └── nav2_params.yaml         # Nav2 stack (planners, controllers, behavior tree)
│
├── scripts/                     # Helper scripts
│   ├── launch_mapping_session.sh   # Unified SLAM + teleop launcher (real robot)
│   ├── launch_isaac_sim_rviz.sh    # RViz for Isaac Sim (use_sim_time=true)
│   ├── remote_launch_slam_rviz.sh  # Remote RViz for mapping
│   ├── remote_launch_nav_rviz.sh   # Remote RViz for navigation
│   ├── generate_maze.py            # Wide maze generator for Isaac Sim
│   └── run_maze_generator.sh       # Maze generator launcher
│
├── rviz/                        # RViz configuration files
│   ├── mapping.rviz             # SLAM mapping visualization
│   ├── navigation.rviz          # Nav2 navigation visualization
│   ├── ogre.rviz                # General robot visualization
│   └── slam_universal.rviz      # Universal SLAM config
│
├── maps/                        # Saved maps (.yaml + .pgm)
│   ├── my_map.yaml              # Example saved map
│   └── wide_maze.yaml           # Wide maze for Nav2 testing
│
├── usds/                        # Isaac Sim USD files
│   ├── ogre.usd                 # Main robot scene with maze
│   ├── ogre_stable.usd          # Wide-base configuration
│   ├── ogre_backup.usd          # Backup before maze generation
│   └── ogre-*.usd               # Experimental configurations
│
└── docs/                        # Additional documentation
    ├── IMPLEMENTATION.md        # Implementation details and testing
    ├── NAV2_README.md           # Nav2 autonomous navigation guide
    ├── MAZE_GENERATOR.md        # Maze generator documentation
    └── OGRE_WIDE.md             # Wide-base robot configuration
```

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

### Isaac Sim Files (usds/)

- `usds/ogre.usd`: Main Isaac Sim scene with mecanum robot and maze
- `usds/ogre_stable.usd`: Wide-base configuration (340mm track width)
- `usds/ogre_backup.usd`: Backup before maze generation
- `usds/ogre-*.usd`: Experimental robot configurations

### Robot Physical Dimensions (ogre.usd)

**Current Robot Configuration**

This matches the real Project Ogre hardware for accurate simulation.

**Body (Main Chassis):**
- Length (X): 0.20m (200mm)
- Width (Y): 0.16m (160mm)
- Height (Z): 0.175m (175mm)
- Position: Center at Z=0.1475m (body bottom at ~70mm above ground)
- Mass: ~2.7kg

**Weighted Components:**
- **Barrels:** Two 55mm diameter × 70mm tall cylinders in front (0.71kg total)
- **Battery Pack:** 55mm × 160mm × 55mm behind body (0.71kg, counterweight)

**Overall Robot Footprint:**
- Total Length: ~310mm (wheelbase + wheel width + components)
- Total Width: 205mm (track width)
- Total Height: ~300mm (to top of LIDAR)
- Total Mass: 4.5-5.0kg

**Wheels:**
- Radius: 0.040m (40mm)
- Width: 0.040m (40mm)
- Wheelbase: 0.095m (95mm front-to-rear axle distance)
- Track width: 0.205m (205mm left-to-right wheel centers)
- Mass: ~0.1kg each

**Wheel Positions (relative to base_link at wheel axle height):**
- Front-Left (FL): (0.0475, 0.1025, 0.04)
- Front-Right (FR): (0.0475, -0.1025, 0.04)
- Rear-Left (RL): (-0.0475, 0.1025, 0.04)
- Rear-Right (RR): (-0.0475, -0.1025, 0.04)

**⚠️ NOTE:** An alternative wide-base configuration (`usds/ogre_stable.usd`) exists with 200mm wheelbase and 340mm track width. See `docs/OGRE_WIDE.md` for details.

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

### Mecanum Drive Action Graph Setup (/cmd_vel Subscriber)

The Isaac Sim robot requires an action graph to subscribe to `/cmd_vel` and control the wheels. This enables:
- ✅ Nav2 autonomous navigation
- ✅ ROS2 keyboard teleop (`teleop_twist_keyboard`)
- ✅ Web-based teleop (ogre_teleop)
- ✅ Any ROS2 node publishing Twist messages

**Action Graph Nodes:**
1. **On Playback Tick** (trigger)
2. **ROS2 Subscribe Twist** → Topic: `/cmd_vel`, Queue Size: 10
3. **Break 3-Vector** (linear) → Outputs: x (vx), y (vy), z (unused)
4. **Break 3-Vector** (angular) → Outputs: x (unused), y (unused), z (vtheta)
5. **Math Nodes** → Compute mecanum wheel velocities (see equations below)
6. **Multiply Nodes** → Negate RIGHT wheel velocities only: `wheel_fr * -1`, `wheel_rr * -1`
7. **Make Array** → Order: `[wheel_fl, -wheel_fr, wheel_rl, -wheel_rr]`
8. **Articulation Controller** → Apply velocities to joints

**Mecanum Wheel Equations (for ogre.usd):**
```
L = (wheelbase + trackwidth) / 2 = (0.095 + 0.205) / 2 = 0.15

# RIGHT wheels (FR, RR) negated - they have opposite joint axis orientation
# EMPIRICALLY TESTED: [+,+,+,+] causes CCW spin, proving asymmetric axes
wheel_fl =  (vx - vy - vtheta * L)      # fl_joint (no negation)
wheel_fr = -(vx + vy + vtheta * L)      # fr_joint (NEGATED - right wheel)
wheel_rl =  (vx + vy - vtheta * L)      # rl_joint (no negation)
wheel_rr = -(vx - vy + vtheta * L)      # rr_joint (NEGATED - right wheel)
```

**Important Notes:**
- **Joint names for ogre.usd:** `fl_joint`, `fr_joint`, `rl_joint`, `rr_joint`
- **VERIFIED JOINT ORDER:** Isaac Lab returns joints as `[FR, RR, RL, FL]` (indices 0,1,2,3) - NOT the query order!
- Velocity array order in Articulation Controller: `["fr_joint", "rr_joint", "rl_joint", "fl_joint"]` (actual USD order)
- **RIGHT wheels (FR=index 0, RR=index 1) must be negated** because they have opposite joint axis orientation
- Robot articulation path: `/World/Ogre/base_link` (or check your USD stage)

**RL Policy Deployment:**
When deploying a trained RL policy from ogre-lab, the Isaac Sim action graph must negate FR and RR wheel velocities. The training environment applies the same sign corrections, so the policy outputs normalized wheel velocities where `[+,+,+,+]` = forward motion. Without matching corrections in the action graph, the robot will spin instead of going forward. See the [ogre-lab README](https://github.com/protomota/ogre-lab#training-notes) for details.

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

### Testing /cmd_vel Action Graph

After creating the `/cmd_vel` subscriber action graph, test each motion independently:

```bash
# Start Isaac Sim, load ogre.usd, press Play ▶️
# In terminal:
export ROS_DOMAIN_ID=42

# Test 1: Pure forward (should go STRAIGHT, no rotation)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Test 2: Pure strafe right (should slide RIGHT, no rotation)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Test 3: Pure rotation CCW (should spin counter-clockwise in place)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10

# Test 4: Use ROS2 keyboard teleop
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Troubleshooting:**
- **Forward causes rotation** → Right wheels spinning wrong direction, check negations
- **Strafe causes rotation** → Front/rear or left/right wheels swapped in joint array
- **Rotation backwards** → Negate vtheta in all equations
- **No movement** → Check Articulation Controller path and joint names match
- anything ROS should go into omni-slam

## RL Policy Training (Isaac Lab)

The RL policy for velocity tracking is trained using Isaac Lab and deployed via the `ogre_policy_controller` ROS2 package.

### Training Environment Location

```
~/isaac-lab/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/ogre_navigation/
├── ogre_navigation_env.py     # Main training environment
├── __init__.py                # Task registration
└── agents/
    └── rsl_rl_ppo_cfg.py      # PPO hyperparameters
```

### Training Commands

```bash
# Activate Isaac Lab environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
cd ~/isaac-lab/IsaacLab

# Train (headless for speed)
python scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Ogre-Navigation-Direct-v0 --headless

# Train with visualization
python scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Ogre-Navigation-Direct-v0

# Export trained policy to ONNX
python scripts/reinforcement_learning/rsl_rl/export_policy.py --task Isaac-Ogre-Navigation-Direct-v0 --checkpoint <path_to_model.pt>
```

### Critical Training Configuration

**File:** `ogre_navigation_env.py`

```python
# Action scaling - CRITICAL
action_scale = 8.0  # Scales policy output [-1,1] to wheel velocity [rad/s]

# Target velocity ranges
max_lin_vel = 0.3   # Max body velocity (m/s) for random targets
max_ang_vel = 1.0   # Max angular velocity (rad/s) for random targets

# Safety limit for wheel velocities
max_wheel_vel = 8.0  # rad/s - robot flips above this

# Reward configuration
rew_scale_vel_tracking = 2.0     # Main reward for velocity tracking
rew_scale_exceed_limit = -1.0    # Penalty ONLY for velocities > max_wheel_vel
rew_scale_uprightness = 1.0      # Bonus for staying upright
```

### Observation Space (10 dimensions)

```
[target_vx, target_vy, target_vtheta,  # Target velocity (3)
 current_vx, current_vy, current_vtheta,  # Current body velocity (3)
 wheel_vel_fl, wheel_vel_fr, wheel_vel_rl, wheel_vel_rr]  # Wheel velocities (4)
```

### Action Space (4 dimensions)

Policy outputs 4 values in range [-1, 1], which are scaled by `action_scale` to get wheel velocities in rad/s:
- `[FL, FR, RL, RR]` wheel velocity targets

### Lessons Learned (CRITICAL)

#### 1. Never Clip After Scaling

**BUG:** Clipping actions AFTER scaling destroys proportional control.

```python
# WRONG - clips scaled values back to [-1, 1], losing all range
self.actions = self.action_scale * actions.clone()  # Scale to [-8, 8]
clipped = torch.clamp(self.actions, -1.0, 1.0)  # Clips to [-1, 1]!

# CORRECT - no clipping, policy learns appropriate outputs
self.actions = self.action_scale * actions.clone()  # Scale to [-8, 8]
# Apply directly to robot
```

#### 2. Energy Penalty Kills Performance

**BUG:** Penalizing ALL wheel velocities causes policy to output minimal values.

```python
# WRONG - penalizes all velocities, policy learns to output ~0.1
rew_energy = -0.01 * torch.sum(actions ** 2, dim=-1)
# Result: wheel velocities ~1.6 rad/s when 7.5 needed for 0.3 m/s

# CORRECT - only penalize velocities ABOVE the safe limit
excess = torch.clamp(torch.abs(actions) - max_wheel_vel, min=0.0)
rew_exceed_limit = -1.0 * torch.sum(excess ** 2, dim=-1)
# Result: 0-8 rad/s = no penalty, >8 rad/s = strong penalty
```

#### 3. Wheel Sign Corrections

Right wheels (FR, RR) have opposite joint axis orientation in the USD. Apply corrections in `_apply_action()`:

```python
corrected_actions = self.actions.clone()
corrected_actions[:, 0] *= -1  # FR (index 0 in Isaac Lab joint order)
corrected_actions[:, 1] *= -1  # RR (index 1 in Isaac Lab joint order)
self.robot.set_joint_velocity_target(corrected_actions, joint_ids=self._wheel_joint_ids)
```

**Isaac Lab joint order:** `[FR, RR, RL, FL]` (indices 0,1,2,3) - NOT the query order!

#### 4. Spawn Position

Robot should spawn at Z = wheel_radius (0.04m) to place wheels exactly on ground:

```python
init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.04),  # wheel_radius = 0.04m
)
```

### ROS2 Policy Controller

**Package:** `ogre_policy_controller`
**Node:** `policy_controller_node.py`

The ROS2 controller loads the ONNX policy and runs inference at 50Hz:

```yaml
# config/policy_controller_params.yaml
action_scale: 8.0       # Must match training
max_lin_vel: 0.3        # Must match training
max_ang_vel: 1.0        # Must match training
output_mode: "joint_state"  # Publish JointState to Isaac Sim
output_topic: "/joint_command"
```

**Deploy new policy:**
```bash
# Copy exported ONNX to controller
cp ~/isaac-lab/IsaacLab/logs/rsl_rl/ogre_navigation/<run>/exported/policy.onnx \
   ~/ros2_ws/src/ogre-slam/ogre_policy_controller/models/

# Rebuild and launch
cd ~/ros2_ws && colcon build --packages-select ogre_policy_controller
ros2 launch ogre_policy_controller policy_controller.launch.py
```

### Debugging Policy Issues

```bash
# Check policy outputs
ros2 topic echo /joint_command

# Expected for forward motion (0.3 m/s):
# All 4 wheel velocities ~7.5 rad/s (same sign)

# Check observations being sent to policy
# Look for log lines: "Target: [...] | Obs: [...] | Actions: [...]"
```

**Common issues:**
- **Low velocities (1-2 rad/s):** Energy penalty too strong, or action clipping bug
- **Velocities > 8 rad/s:** Missing exceed-limit penalty in training
- **Wrong direction:** Sign corrections not matching between training and deployment
- **Asymmetric wheel speeds:** Check wheel order matches between training and controller