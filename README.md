# ogre-slam

SLAM mapping and autonomous navigation for Project Ogre mecanum drive robot.

## Overview

This is a **standalone ROS2 package** - it declares all its own dependencies and can run independently.

It provides SLAM (Simultaneous Localization and Mapping) and autonomous navigation capabilities for mecanum drive robots using:
- **Wheel odometry** from 4 mecanum drive encoders (2 PPR Hall sensors, calibrated gear_ratio: 224.0)
- **RPLIDAR A1** 2D laser scanner for mapping and localization
- **RealSense D435** depth camera for 3D obstacle avoidance
- **slam_toolbox** for map building and localization
- **robot_localization** EKF for sensor fusion
- **Nav2** for autonomous waypoint navigation

## Features

- ✅ Wheel encoder odometry with mecanum drive kinematics (±0.8% accuracy)
- ✅ Async SLAM mapping optimized for Jetson Orin Nano
- ✅ Sensor fusion (EKF) to handle low encoder resolution
- ✅ Map saving/loading for autonomous navigation
- ✅ **NEW:** Autonomous waypoint navigation with Nav2
- ✅ **NEW:** 3D obstacle avoidance using RealSense D435 pointcloud
- ✅ **NEW:** Mecanum-aware path planning (omnidirectional movement)
- ✅ RViz visualization with Nav2 panel
- ✅ Manual override via web teleop interface

## Hardware Requirements

- **Jetson Orin Nano** Developer Kit
- **Mecanum drive** robot with 4 motors (25GA-370 with gear_ratio 224.0)
- **Encoders**: 2 PPR Hall sensors on each motor
- **RPLIDAR** (A1/A2/A3 series) - 2D laser scanner
- **RealSense D435** depth camera - 3D obstacle detection (NEW)
- **GPIO access** for encoder reading

### Robot Configuration

**Physical Dimensions:**
- **Body**: 200mm (L) × 160mm (W) × 175mm (H), positioned 20mm above wheel axle
- **Weighted Barrels**: Two 55mm dia × 70mm tall cylinders in front (0.71kg total)
- **Battery Pack**: 55mm × 160mm × 55mm behind body (0.71kg, counterweight)
- **Wheels**: 40mm radius, 40mm width
- **Wheelbase**: 95mm (front-to-rear axle distance)
- **Track Width**: 205mm (left-to-right wheel centers)
- **Total Footprint**: ~310mm length × 205mm width × 300mm height
- **LIDAR Mount**: On 65mm posts, LIDAR at 0.30m above base_link (rotated 180°)
- **Camera Mount**: Front camera at 0.15m forward, 0.10m above base_link (`front_camera` in Isaac Sim, `camera_link` for RealSense D435 on real robot)

**Motor Layout:**
```
M4 (FL) --- M1 (FR)
   |    X    |
M3 (RL) --- M2 (RR)
```

**Encoder Pins (BOARD mode):**
- M1 (Front-Right): Pins 7, 11
- M2 (Rear-Right): Pins 13, 15
- M3 (Rear-Left): Pins 29, 31
- M4 (Front-Left): Pins 32, 33

## Dependencies

### System Packages (REQUIRED - Install First!)

**⚠️ IMPORTANT:** Install these dependencies BEFORE building the package:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-tf2-tools
```

**Note:** If you see errors about `packages.ros.org/ros` repository, ignore them - we only need ROS2 packages which will install correctly.

### Python Dependencies

Required Python packages (installed automatically during build):
- `Jetson.GPIO` - For GPIO encoder reading on Jetson hardware
- `numpy` - For mecanum kinematics calculations

**Manual installation (if needed):**
```bash
pip3 install Jetson.GPIO numpy
```

## Installation

1. **Clone repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone git@github.com:protomota/ogre-slam.git
   ```

2. **Install dependencies** (see above)

3. **Measure robot dimensions** (CRITICAL):
   ```bash
   cd ogre-slam
   nano config/odometry_params.yaml
   ```

   Update these values with actual measurements:
   - `wheel_radius`: Distance from wheel axle to ground (meters)
   - `wheel_base`: Distance between front and rear axles (meters)
   - `track_width`: Distance between left and right wheels (meters)
   - `gear_ratio`: Motor gear ratio (see Gear Ratio Calibration below)

4. **Build package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ogre_slam --symlink-install
   source install/setup.bash
   ```

## Quick Start

### Navigation Mode (Autonomous Waypoint Navigation) ⭐ NEW!

**Prerequisites:**
1. Install Nav2 packages (see NAV2_README.md)
2. Have a saved map from mapping mode

**Launch autonomous navigation:**
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml
```

This launches:
1. **Localization**: slam_toolbox with saved map
2. **Sensors**: RPLIDAR + RealSense D435 pointcloud
3. **Nav2 Stack**: Path planning, obstacle avoidance, controller
4. **Control**: Web interface for manual override

**Navigate to waypoints:**
1. In RViz, click "2D Pose Estimate" and set robot's initial position
2. Click "Nav2 Goal" button and click destination on map
3. Robot autonomously navigates, avoiding obstacles with RealSense depth
4. Manual override always available at `http://10.21.21.45:8080`

**📖 Full documentation:** See [NAV2_README.md](NAV2_README.md) for complete guide

---

### Mapping Mode (Build a Map)

**Single command launch:**
```bash
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_mapping_session.sh
```

This launches:
1. SLAM system (RPLIDAR, odometry, slam_toolbox, RViz)
2. ogre_teleop web interface for manual control

**Drive the robot:**
1. Open browser: `http://10.21.21.45:8080`
2. Drive slowly around the area
3. Ensure good loop closures (revisit starting point)
4. Watch RViz to see map being built

**Save map when done:**
```bash
# In another terminal
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/my_map
```

This creates:
- `my_map.yaml` - Map metadata
- `my_map.pgm` - Map image

**Stop system:**
```bash
# Press Ctrl+C in launch terminal
```

### Remote Visualization

**Visualize SLAM from another computer on the same network:**

1. **On the robot:** Launch SLAM without RViz to save resources
   ```bash
   ros2 launch ogre_slam mapping.launch.py use_rviz:=false
   ```

2. **On remote computer:** Run the RViz launcher script
   ```bash
   cd ~/ros2_ws/src/ogre-slam
   ./scripts/remote_launch_slam_rviz.sh
   ```

This will:
- Check ROS2 connection to robot (10.21.21.45)
- Set correct ROS_DOMAIN_ID (42)
- Create pre-configured RViz setup with:
  - LaserScan visualization (`/scan`)
  - Map display (`/map`)
  - TF frame visualization
- Launch RViz with automatic display configuration

**Requirements:**
- Remote computer must have ROS2 Humble installed
- Both computers on same network (10.21.21.x)
- ROS_DOMAIN_ID=42 set on both machines
- Network allows multicast or FastDDS peer discovery

### Manual Launch (Advanced)

If you want more control, launch systems separately:

```bash
# Terminal 1: SLAM system only
ros2 launch ogre_slam mapping.launch.py rplidar_model:=a1

# Terminal 2: Teleop separately
ros2 launch ogre_teleop web_teleop.launch.py
```

## Isaac Sim Simulation

For testing with NVIDIA Isaac Sim 5.0:

### ROS2 Domain Configuration

**IMPORTANT:** Isaac Sim must use the same ROS_DOMAIN_ID as your ROS2 system.

This project uses **ROS_DOMAIN_ID=42** (same as the real robot).

**To configure Isaac Sim:**
1. In your action graph, select the **ROS2 Context** node
2. In the Property panel, set **Domain ID** to: **42**
3. Restart simulation (Stop ⏸️ then Play ▶️)

**Verify domain match:**
```bash
# In terminal
export ROS_DOMAIN_ID=42
ros2 topic list  # Should see Isaac Sim topics when simulation is running
```

### ROS2 Keyboard Teleoperation

Control the Isaac Sim robot using standard ROS2 keyboard teleop:

**Install (if not already installed):**
```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

**Launch teleoperation:**
```bash
# Set domain to match Isaac Sim and robot
export ROS_DOMAIN_ID=42

# Run teleop keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard controls:**
- **i**: Move forward
- **,**: Move backward
- **j**: Rotate left
- **l**: Rotate right
- **k**: Stop
- **Shift + J**: Strafe left (holonomic mode)
- **Shift + L**: Strafe right (holonomic mode)
- **u/o/m/.**: Combined forward/backward + rotation
- **q/z**: Increase/decrease linear speed
- **w/x**: Increase/decrease angular speed

**Note:** The teleop terminal window must be in focus for keypresses to work.

### SLAM Mapping with RViz

Visualize SLAM mapping from Isaac Sim using the same RViz configuration as the real robot.

**Prerequisites:**
- Isaac Sim running with ROS2 bridge (Domain ID = 42)
- LIDAR configured to publish to `/scan` topic with frame `laser`
- **TF transforms published**: `odom→base_link→laser` (see CLAUDE.md for setup)
- Simulation playing (Press Play ▶️)

**Launch RViz for Isaac Sim:**
```bash
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_isaac_sim_rviz.sh
```

This launches RViz with the universal SLAM configuration that works for both Isaac Sim and real robot.

**⚠️ IMPORTANT - Separate Scripts for Sim vs Real Robot:**

The package provides **different scripts** for Isaac Sim and real robot visualization:

| Use Case | Script | Time Source | ROS_USE_SIM_TIME |
|----------|--------|-------------|------------------|
| **Isaac Sim** | `launch_isaac_sim_rviz.sh` | Simulation clock (`/clock` topic) | `true` |
| **Real Robot (Mapping)** | `remote_launch_slam_rviz.sh` | System time | Not set (real time) |
| **Real Robot (Navigation)** | `remote_launch_nav_rviz.sh` | System time | Not set (real time) |

**Why this matters:**
- Isaac Sim publishes a `/clock` topic with simulation time
- RViz must use `ROS_USE_SIM_TIME=true` to sync with simulation clock
- Real robot uses system time, so RViz uses normal time
- **These scripts do NOT interfere with each other** - they're for completely separate use cases

**Expected displays:**
- **Grid** - Reference grid
- **LaserScan** - LIDAR scan points from `/scan`
- **Map** - SLAM-generated map from `/map` (if running slam_toolbox)
- **TF** - Transform frames (base_link, laser, odom, map)
- **Odometry** - Robot path from `/odom` or `/odometry/filtered`

**Switching between Isaac Sim and real robot:**
- Same RViz config works for both (both use `/scan`, `/odom`, `/map` topics)
- Same TF frame names (map, odom, base_link, laser)
- Use the appropriate launch script for your use case
- Set `ROS_DOMAIN_ID=42` for both

**Troubleshooting:**

If you see "Message Filter dropping message: frame 'laser' at time..." errors:
- **Cause**: Isaac Sim not publishing TF transforms
- **Fix**: Set up ROS2 TF publishing in Isaac Sim action graph (see CLAUDE.md "ROS2 TF Transform Publishing" section)
- **Quick workaround**: In RViz, change Fixed Frame from `map` to `laser` (but won't show full odometry/SLAM)

If RViz shows "waiting for transform...":
- Verify TF tree: `ros2 run tf2_tools view_frames` (should show odom→base_link→laser)
- Check topic: `ros2 topic echo /odom` (should see odometry messages)

See `CLAUDE.md` for complete Isaac Sim robot configuration (dimensions, sensors, action graphs, TF setup).

## Configuration

### Gear Ratio Calibration (CRITICAL!)

The `gear_ratio` parameter is the most important value to calibrate for accurate odometry. Incorrect gear ratio causes massive position errors and prevents SLAM from working.

**For 25GA-370 motors:** The calibrated gear_ratio is **224.0**

**To calibrate for your specific motors:**

1. Launch SLAM system and ensure odometry is publishing
2. Mark the robot's starting position with tape
3. Drive the robot forward **exactly 1.0 meter** in a straight line (use measuring tape)
4. Check reported distance:
   ```bash
   ros2 topic echo /odom --once | grep -A 3 "position"
   ```
5. Calculate correction:
   ```
   new_gear_ratio = current_gear_ratio × (reported_x / actual_distance)
   ```
   Example: If reported x=4.483m with gear_ratio=50.0:
   ```
   new_gear_ratio = 50.0 × (4.483 / 1.0) = 224.0
   ```
6. Update `gear_ratio` in `config/odometry_params.yaml`
7. Restart SLAM and test again until x ≈ 1.0m (±5cm tolerance)

**Calibration results for this robot:**
- Initial test with gear_ratio=50.0: reported 4.483m for 1.0m actual
- Final gear_ratio=224.0: reported 1.008m for 1.0m actual (0.8% error) ✅

### Odometry Parameters

Edit `config/odometry_params.yaml`:

```yaml
wheel_radius: 0.040     # Wheel radius in meters (MEASURED: 40mm)
wheel_base: 0.095       # Front/rear axle distance (MEASURED: 95mm)
track_width: 0.205      # Left/right wheel distance (MEASURED: 205mm)
encoder_ppr: 2          # Hall sensors: 2 PPR (fixed)
gear_ratio: 224.0       # Motor gear ratio (CALIBRATED for 25GA-370)
publish_rate: 50.0      # Hz
```

### SLAM Parameters

Edit `config/slam_toolbox_params.yaml`:

Key parameters:
- `resolution: 0.05` - Map grid size (5cm)
- `max_laser_range: 12.0` - RPLIDAR max range
- `minimum_travel_distance: 0.0` - Process scans based on time only
- `minimum_travel_heading: 0.0` - Process scans based on time only
- `minimum_time_interval: 0.1` - Process scans every 0.1 seconds
- `map_update_interval: 2.0` - Publish map every 2 seconds (Jetson optimization)

### EKF Sensor Fusion

**EKF (Extended Kalman Filter)** combines multiple noisy sensor measurements to produce accurate state estimates.

**Why This Robot Needs EKF:**

The 2 PPR Hall sensors provide very coarse encoder resolution, resulting in:
- Noisy velocity measurements
- Jittery odometry output
- Poor SLAM performance without filtering

**How EKF Works:**

The `robot_localization` package implements a two-step process:

1. **Predict**: Estimates robot state based on motion model (wheel velocities)
2. **Update**: Corrects prediction using sensor measurements with confidence weighting

**Data Flow:**
```
Raw Encoders → odometry_node → /odom (noisy)
                                   ↓
                            ekf_filter_node (smoothing)
                                   ↓
                          /odometry/filtered (clean)
                                   ↓
                        SLAM & Nav2 (accurate mapping/navigation)
```

**Configuration:**

Edit `config/ekf_params.yaml` to:
- Add additional sensors (IMU recommended for production)
- Tune covariance values for sensor noise characteristics
- Configure which state variables to fuse (position, velocity, orientation)

The EKF is **essential** for this robot - without it, the 2 PPR encoders produce unusable odometry.

## Package Structure

```
ogre-slam/
├── ogre_slam/                   # Python module
│   ├── encoder_reader.py        # GPIO encoder reading
│   ├── mecanum_odometry.py      # Mecanum kinematics
│   └── odometry_node.py         # ROS2 odometry node
├── launch/
│   └── mapping.launch.py        # Main mapping launch file
├── config/
│   ├── odometry_params.yaml     # Robot dimensions
│   ├── slam_toolbox_params.yaml # SLAM configuration
│   └── ekf_params.yaml          # Sensor fusion config
├── scripts/
│   ├── launch_mapping_session.sh    # Unified launcher (robot)
│   └── remote_launch_slam_rviz.sh   # Remote RViz launcher
├── maps/                        # Saved maps directory
├── rviz/
│   └── mapping.rviz            # RViz configuration
├── package.xml
├── setup.py
└── README.md
```

## Topics

### Published

- `/odom` (nav_msgs/Odometry) - Wheel odometry
- `/odometry/filtered` (nav_msgs/Odometry) - EKF-filtered odometry
- `/map` (nav_msgs/OccupancyGrid) - SLAM-generated map
- `/encoder_ticks` (std_msgs/Int32MultiArray) - Raw encoder counts (debugging)

### Subscribed

- `/scan` (sensor_msgs/LaserScan) - From RPLIDAR

### TF Frames

```
map (from slam_toolbox)
  └─ odom (from EKF or odometry_node)
      └─ base_link (robot center at wheel axle height)
          ├─ laser (RPLIDAR: 0.30m up, 180° rotated - on 65mm posts)
          ├─ front_camera (Isaac Sim camera: 0.15m forward, 0.10m up)
          ├─ camera_link (Real robot RealSense D435: 0.15m forward, 0.10m up)
          └─ imx477_camera_optical_frame (RPi camera)
```

## Troubleshooting

### Odometry Not Working

**Problem:** No /odom topic or encoder errors

**Common Misconception:** "odometry_node conflicts with motor_control_node on GPIO"
- **FALSE!** There is NO GPIO conflict
- motor_control_node uses I2C (PCA9685) for PWM control
- odometry_node uses GPIO pins (7,11,13,15,29,31,32,33) for encoders
- Both can run simultaneously without issues

**Solutions:**
1. Check GPIO permissions:
   ```bash
   sudo usermod -a -G gpio $USER
   # Log out and back in
   ```

2. If you see "Device or resource busy" error:
   - Previous instance didn't clean up GPIO
   - Solution: Restart the Jetson or manually cleanup:
   ```bash
   # Kill all Python processes using GPIO
   pkill -9 python3
   # Wait a moment, then restart SLAM
   ```

3. Verify encoder connections:
   ```bash
   # Test encoder reader
   cd ~/ros2_ws/src/ogre-slam
   python3 ogre_slam/encoder_reader.py
   # Rotate wheels manually, should see counts
   ```

4. Check logs:
   ```bash
   ros2 topic echo /encoder_ticks
   # Should show non-zero values when wheels turn
   ```

### SLAM Not Building Map / Map Not Updating

**Problem:** Map stays empty or frozen in RViz, doesn't update when robot moves

**Root Cause:** This was a critical issue caused by incorrect gear_ratio and/or using dummy odometry that always reported (0,0,0). slam_toolbox requires meaningful odometry changes to trigger scan processing, even in scan-matching mode.

**Solution:**
1. **Use real encoder odometry** (NOT dummy odometry)
   - Ensure `use_odometry: true` in launch file
   - Verify `/odom` topic shows changing position when driving

2. **Calibrate gear_ratio correctly** (see Gear Ratio Calibration section)
   - Incorrect gear_ratio causes massive position errors
   - SLAM won't process scans if odometry shows unrealistic movement
   - For 25GA-370 motors: use gear_ratio=224.0

3. **Set SLAM thresholds to 0.0** to process scans based on time only:
   ```yaml
   # In slam_toolbox_params.yaml
   minimum_travel_distance: 0.0
   minimum_travel_heading: 0.0
   ```

**Quick Checks:**
1. **LIDAR working?**
   ```bash
   ros2 topic hz /scan
   # Should show ~7-8 Hz for RPLIDAR A1
   ```

2. **Odometry accurate?**
   ```bash
   ros2 topic echo /odom --once | grep -A 3 "position"
   # Drive 1m forward, x should be ~1.0m (not 100m or 0.01m!)
   ```

3. **TF tree complete?**
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   # Should show transform without errors
   ```

4. **Map being published?**
   ```bash
   ros2 topic hz /map
   # Should show ~0.5 Hz (updates every 2 seconds)
   ```

### Map Quality Poor

**Tips for better maps:**
1. **Drive slowly** - Give SLAM time to process
2. **Good lighting** - Helps if using visual SLAM (future)
3. **Loop closures** - Return to starting point
4. **Avoid slippage** - Mecanum wheels can slip on smooth floors
5. **Multiple passes** - Drive same area from different angles

### High Odometry Drift

**Problem:** Position estimate drifts quickly

**Causes:**
- 2 PPR encoders are very coarse
- Mecanum wheel slippage
- Incorrect wheel dimensions in config

**Solutions:**
1. **Verify measurements** in `odometry_params.yaml`
2. **Add IMU** (highly recommended for production)
3. **Tune EKF** covariance in `ekf_params.yaml`
4. **Use SLAM localization** (fuses laser with odometry)

### RViz Shows No Data

1. **Check Fixed Frame:**
   - Should be `map` for mapping mode
   - Change in Global Options

2. **Add displays:**
   - LaserScan → `/scan`
   - Map → `/map`
   - TF → (check this to see frames)

3. **Check topic connections:**
   ```bash
   ros2 topic list
   # Should see /scan, /odom, /map
   ```

## Performance Tuning

### Jetson Optimization

**Enable max performance:**
```bash
sudo /usr/bin/jetson_clocks
```

**Monitor resource usage:**
```bash
tegrastats
```

**Reduce CPU load:**
- Increase `map_update_interval` in slam_toolbox config
- Lower `publish_rate` in odometry config
- Reduce RViz displays

### Memory Constraints

Jetson Orin Nano has 8GB RAM. If SLAM crashes:

1. **Reduce map resolution:**
   ```yaml
   resolution: 0.10  # Instead of 0.05
   ```

2. **Limit search space:**
   ```yaml
   correlation_search_space_dimension: 0.3  # Instead of 0.5
   ```

3. **Disable RViz:**
   ```bash
   ros2 launch ogre_slam mapping.launch.py use_rviz:=false
   ```

## Future Work

- [ ] Add IMU integration for better orientation
- [ ] Nav2 integration for autonomous navigation
- [ ] Localization-only mode (use saved maps)
- [ ] Wall-following algorithm for maze solving
- [ ] Waypoint navigation
- [ ] Obstacle avoidance with RealSense depth

## Contributing

This is part of Project Ogre. For issues or contributions:
- GitHub: https://github.com/protomota/ogre-slam

## License

MIT License

## Acknowledgments

- **slam_toolbox**: Steve Macenski
- **robot_localization**: Tom Moore
- **RPLIDAR ROS**: SLAMTEC

---

**Robot IP:** 10.21.21.45
**ROS2 Distro:** Humble
**Platform:** Jetson Orin Nano
