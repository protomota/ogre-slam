# ogre-slam

SLAM mapping and autonomous navigation for Project Ogre mecanum drive robot.

## Overview

This ROS2 package provides SLAM (Simultaneous Localization and Mapping) capabilities for the Project Ogre robot using:
- **Wheel odometry** from 4 mecanum drive encoders (2 PPR Hall sensors)
- **RPLIDAR** 2D laser scanner for mapping
- **slam_toolbox** for map building and localization
- **robot_localization** EKF for sensor fusion

## Features

- ✅ Wheel encoder odometry with mecanum drive kinematics
- ✅ Async SLAM mapping optimized for Jetson Orin Nano
- ✅ Sensor fusion (EKF) to handle low encoder resolution
- ✅ Unified launcher for easy operation
- ✅ Map saving/loading for autonomous navigation
- ✅ RViz visualization
- 🚧 Nav2 integration (future work)

## Hardware Requirements

- **Jetson Orin Nano** Developer Kit
- **Mecanum drive** robot with 4 motors
- **Encoders**: 2 PPR Hall sensors on each motor
- **RPLIDAR** (A1/A2/A3 series)
- **GPIO access** for encoder reading

### Robot Configuration

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

Already included from project-ogre:
- `Jetson.GPIO`
- `numpy`

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

4. **Build package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ogre_slam --symlink-install
   source install/setup.bash
   ```

## Quick Start

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

### Manual Launch (Advanced)

If you want more control, launch systems separately:

```bash
# Terminal 1: SLAM system only
ros2 launch ogre_slam mapping.launch.py rplidar_model:=a1

# Terminal 2: Teleop separately
ros2 launch ogre_teleop web_teleop.launch.py
```

## Configuration

### Odometry Parameters

Edit `config/odometry_params.yaml`:

```yaml
wheel_radius: 0.05      # MEASURE THIS!
wheel_base: 0.25        # MEASURE THIS!
track_width: 0.30       # MEASURE THIS!
encoder_ppr: 2          # Hall sensors: 2 PPR
publish_rate: 50.0      # Hz
```

### SLAM Parameters

Edit `config/slam_toolbox_params.yaml`:

Key parameters:
- `resolution: 0.05` - Map grid size (5cm)
- `max_laser_range: 12.0` - RPLIDAR max range
- `minimum_travel_distance: 0.2` - Don't map while stationary
- `map_update_interval: 2.0` - Jetson optimization

### EKF Sensor Fusion

Edit `config/ekf_params.yaml`:

The EKF fuses wheel odometry to smooth out noise from 2 PPR encoders.

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
│   └── launch_mapping_session.sh # Unified launcher
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
      └─ base_link (robot center)
          ├─ laser (RPLIDAR)
          ├─ camera_link (RealSense)
          └─ imx477_camera_optical_frame (RPi camera)
```

## Troubleshooting

### Odometry Not Working

**Problem:** No /odom topic or encoder errors

**Solutions:**
1. Check GPIO permissions:
   ```bash
   sudo usermod -a -G gpio $USER
   # Log out and back in
   ```

2. Verify encoder connections:
   ```bash
   # Test encoder reader
   cd ~/ros2_ws/src/ogre-slam
   python3 ogre_slam/encoder_reader.py
   # Rotate wheels manually, should see counts
   ```

3. Check logs:
   ```bash
   ros2 topic echo /encoder_ticks
   # Should show non-zero values when wheels turn
   ```

### SLAM Not Building Map

**Problem:** Map stays empty in RViz

**Checks:**
1. **LIDAR working?**
   ```bash
   ros2 topic hz /scan
   # Should show ~5-6 Hz for RPLIDAR A1
   ```

2. **Robot moving?**
   ```bash
   ros2 topic echo /odom
   # Position should change when driving
   ```

3. **TF tree complete?**
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   # Should show transform
   ```

4. **Drive faster:** Minimum travel distance is 0.2m

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
