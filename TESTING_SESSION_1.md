# SLAM Testing Session 1 - November 16, 2025

## Summary
Successfully implemented and tested ogre-slam package through Test 7 of 10. Full SLAM system is operational with RViz visualization. System ready for mapping session.

## Tests Completed ✅

### Test 1: Encoder Reading (Hardware)
**Status**: ✅ PASSED
**Result**: All 4 encoders reading correctly via GPIO interrupts
- M1 (Front-Right): Pins 7, 11
- M2 (Rear-Right): Pins 13, 15
- M3 (Rear-Left): Pins 29, 31
- M4 (Front-Left): Pins 32, 33

### Test 2: Mecanum Odometry (Math)
**Status**: ✅ PASSED
**Result**: Mecanum kinematics correctly implemented
- Forward kinematics formulas verified
- Velocity calculations accurate for 4-wheel mecanum drive

### Test 3: Robot Dimensions (Config)
**Status**: ✅ PASSED
**Measured Values**:
- Wheel radius: 40mm (0.04m)
- Wheel base: 95mm (0.095m)
- Track width: 205mm (0.205m)
- Robot weight: 10 lb 15 oz (4.96 kg)
- Encoder PPR: 2 (Hall sensor)

**Updated**: `config/odometry_params.yaml`

### Test 4: Odometry Node (ROS2)
**Status**: ✅ PASSED
**Result**: Odometry node publishing at 50 Hz
- `/odom` topic: nav_msgs/Odometry
- `/encoder_ticks` topic: std_msgs/Int32MultiArray
- TF transform: odom → base_link
- High covariance (0.1) due to 2 PPR encoders

### Test 5: RPLIDAR (Sensor)
**Status**: ✅ PASSED
**Result**: RPLIDAR A1 operational
- Publishing at 7.6 Hz on `/scan`
- Scan mode: Sensitivity
- Sample rate: 8 kHz
- Max distance: 12.0m
- Frame ID: `laser`

### Test 6: Full SLAM System (Integration)
**Status**: ✅ PASSED
**Issues Fixed**:
1. ❌ → ✅ EKF YAML type mismatch: Changed all integer `0` to float `0.0` in covariance matrices
2. ❌ → ✅ Missing static TF transform: Added `base_link → laser` publisher
3. ❌ → ✅ LIDAR position configured: (0, 0, 0.27m) - centered, 270mm above ground
4. ❌ → ✅ GPIO busy error: Killed stray odometry_node process

**Final TF Tree**:
```
map (SLAM @ 7.9 Hz)
 └─ odom (EKF/Odometry @ 50.2 Hz)
     └─ base_link
         └─ laser (Static @ 0, 0, 0.27m)
```

**Nodes Running**:
- `/rplidar_node` - LIDAR driver
- `/odometry_node` - Wheel odometry
- `/ekf_filter_node` - Sensor fusion
- `/slam_toolbox` - SLAM mapping
- `/map_saver_server` - Map persistence
- `/lifecycle_manager_slam` - Lifecycle management
- `/base_to_laser_broadcaster` - Static TF publisher

**Topics Active**:
- `/scan` - LaserScan data
- `/odom` - Raw odometry
- `/odometry/filtered` - EKF-filtered odometry
- `/map` - Occupancy grid map
- `/tf`, `/tf_static` - Transform tree
- `/encoder_ticks` - Raw encoder counts

### Test 7: SLAM with RViz
**Status**: ✅ PASSED
**Result**: RViz2 successfully launched and visualizing SLAM
- OpenGL 4.6 initialized
- Map visualization active (65x81 grid)
- SLAM sensor registered
- All displays configured

## Tests Remaining 🔄

### Test 8: Full Mapping Session ⏳
**Objective**: Launch ogre_teleop and drive robot to create map
**Expected**: Map grows in real-time as robot explores environment

### Test 9: Map Saving ⏳
**Objective**: Save completed map using map_saver_server
**Command**: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

### Test 10: Graceful Shutdown ⏳
**Objective**: Clean shutdown of all nodes
**Command**: Ctrl+C on launch, verify GPIO cleanup

## Key Configuration Files

### Modified Files (Committed)
- `config/odometry_params.yaml` - Robot dimensions (40mm wheels, 95mm base, 205mm track)
- `config/ekf_params.yaml` - Fixed type mismatch (all floats in covariance)
- `launch/mapping.launch.py` - Added static TF publisher for LIDAR

### Launch Command
```bash
# SLAM with RViz (current)
ros2 launch ogre_slam mapping.launch.py

# SLAM without RViz
ros2 launch ogre_slam mapping.launch.py use_rviz:=false
```

## System Requirements
**Installed Dependencies**:
```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-tf2-tools
```

**Python Dependencies**:
```bash
pip3 install -r requirements.txt
```

## Hardware Configuration

### LIDAR Mounting
- **Position**: Robot center
- **Height**: 270mm above ground
- **TF Transform**: `base_link → laser` at (0, 0, 0.27)

### Encoders
- **Type**: 25GA-370 Hall sensors
- **Resolution**: 2 PPR (very coarse)
- **GPIO Mode**: BOARD numbering
- **Wiring**: M1/M2 RPWM/LPWM swapped

### Motor Configuration
- **Controller**: PCA9685 on I²C Bus 7 (0x40)
- **PWM Frequency**: 1000 Hz
- **Layout**: Mecanum X-configuration
  - M1 (Front-Right): CH1 (RPWM), CH0 (LPWM)
  - M2 (Rear-Right): CH3 (RPWM), CH2 (LPWM)
  - M3 (Rear-Left): CH4 (RPWM), CH5 (LPWM)
  - M4 (Front-Left): CH6 (RPWM), CH7 (LPWM)

## Performance Metrics

### Publishing Rates
- RPLIDAR: 7.6 Hz
- Odometry (raw): 50 Hz
- EKF (filtered): 50 Hz
- SLAM map→odom TF: 7.9 Hz
- Static TF: Continuous

### Resource Usage
- Jetson Orin Nano platform
- OpenGL 4.6 acceleration for RViz
- SLAM using Ceres solver with SCHUR_JACOBI preconditioner

## Known Issues & Warnings

### Non-Critical Warnings
1. **Lifecycle Manager**: Continuously waits for `slam_toolbox/get_state` service
   - **Cause**: async_slam_toolbox doesn't provide lifecycle services
   - **Impact**: None - system fully functional
   - **Action**: Can be ignored

2. **LIDAR Range Warning**: "minimum laser range setting (0.0m) exceeds capabilities (0.2m)"
   - **Cause**: SLAM config has 0.0m min range, LIDAR can only measure >0.2m
   - **Impact**: Minimal - SLAM filters invalid ranges
   - **Action**: Could update `slam_toolbox_params.yaml` min_laser_range to 0.2

### Resolved Issues
- ✅ EKF YAML parsing error (integer vs float types)
- ✅ GPIO device busy (stray process holding pins)
- ✅ Missing TF transform (base_link → laser)
- ✅ SLAM dropping messages (waiting for complete TF tree)

## Next Steps

### For Remote Control Session
1. **On Jetson**: Keep SLAM system running with RViz
2. **On Remote Computer**:
   - SSH to Jetson: `ssh jetson@10.21.21.45`
   - Launch ogre_teleop: `ros2 launch ogre_teleop web_teleop.launch.py`
   - Access web interface: `http://10.21.21.45:8080`
3. **Drive robot** using keyboard controls (WASD, arrow keys)
4. **Monitor RViz** to watch map being built in real-time

### Test 8 Checklist
- [ ] Launch ogre_teleop alongside SLAM
- [ ] Verify motor control works with encoders
- [ ] Drive robot in controlled pattern
- [ ] Observe map building in RViz
- [ ] Check odometry drift over time
- [ ] Verify loop closure (if revisiting areas)

### Test 9 Checklist
- [ ] Save map to file
- [ ] Verify PGM and YAML files created
- [ ] Check map quality and coverage
- [ ] Test map reloading

### Test 10 Checklist
- [ ] Stop ogre_teleop cleanly
- [ ] Stop SLAM launch with Ctrl+C
- [ ] Verify GPIO cleanup (no pins held)
- [ ] Check for any error messages

## Git Repository
**Remote**: git@github.com:protomota/ogre-slam.git
**Latest Commit**: `381e93f` - "Fix SLAM integration issues - Test 6 complete"
**Branch**: main

## Session Notes
- All core functionality verified and working
- System stable and ready for mapping
- Encoder resolution (2 PPR) provides basic odometry but high uncertainty
- EKF successfully fuses odometry with IMU-free operation
- SLAM registered LIDAR sensor and creating map
- RViz visualization confirmed operational

**Session completed at**: Test 7 of 10
**Ready for**: Remote mapping session with ogre_teleop
**System status**: ✅ Fully operational, all nodes running
