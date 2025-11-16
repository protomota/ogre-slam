# SLAM Debugging Session - November 16, 2025

## Summary
Successfully debugged and fixed critical SLAM issues that prevented mapping. System now fully operational.

## Issues Found & Fixed

### Issue 1: TF Transform Conflict ✅ FIXED
**Commit**: 3d45c84

**Problem**: LIDAR scattering all over map, map jumping to random locations

**Root Cause**:
- Both `odometry_node` and `slam_toolbox` publishing `odom→base_link` transform
- Two nodes fighting over same TF at 50 Hz → unstable transform tree
- This was NOT encoder noise - it was a software bug!

**Solution**:
- Set `publish_tf: false` in `config/odometry_params.yaml`
- Only odometry_node publishes `/odom` topic (sensor data)
- SLAM/EKF handle all TF transforms

**File Changed**: `config/odometry_params.yaml` (Line 17)

---

### Issue 2: Missing odom→base_link Transform ✅ FIXED
**Commit**: ae81038

**Problem**: `/map` topic not publishing, incomplete TF tree

**Root Cause**:
- Disabled odometry TF but didn't provide replacement
- SLAM needs complete TF chain: map→odom→base_link→laser

**Solution**:
- Enable EKF by default (`use_ekf: true` in launch file)
- EKF publishes `odom→base_link` (filtered odometry)
- SLAM publishes `map→odom` (drift correction)
- Clean separation of responsibilities

**Architecture**:
```
map ─(slam)─> odom ─(ekf)─> base_link ─(static)─> laser
```

**File Changed**: `launch/mapping.launch.py` (Line 49)

---

### Issue 3: SLAM Silent Crash ✅ FIXED
**Commit**: 9611029

**Problem**:
- SLAM node not appearing in `ros2 node list`
- RViz not showing LIDAR or map
- `/map` topic exists but no data

**Root Cause**:
- SLAM crashing immediately after sensor registration
- Error: `"Smear deviation too small: Must be between 0.005 and 0.1"`
- Set `correlation_search_space_smear_deviation: 0.3` (out of range)

**Solution**:
- Changed back to `0.1` (maximum valid value)
- Valid range: 0.005 to 0.1

**File Changed**: `config/slam_toolbox_params.yaml` (Line 56)

---

### Issue 4: Inverted Strafe Direction ✅ FIXED
**Commit**: 4bfb7e3 (ogre_teleop repo)

**Problem**: Q/E strafe controls backwards

**Root Cause**: Y-axis sign error in mecanum kinematics

**Solution**: Inverted `vy` in all motor speed calculations

**File Changed**: `ogre_teleop/motor_controller.py` (Lines 123-126)

---

## Final Configuration

### TF Transform Publishers
- **odometry_node**: Publishes `/odom` topic only (NO TF)
- **ekf_node**: Publishes `odom→base_link` (filters encoder noise)
- **slam_toolbox**: Publishes `map→odom` (corrects drift via scan matching)
- **static_tf**: Publishes `base_link→laser` (LIDAR position)

### Key Parameters

**Odometry** (`config/odometry_params.yaml`):
- `publish_tf: false` - Critical fix
- `publish_rate: 50.0` Hz
- High covariance (1.0, 0.5) - signals uncertainty to SLAM

**EKF** (`config/ekf_params.yaml`):
- `publish_tf: true` - Publishes filtered odom→base_link
- Enabled by default in launch

**SLAM** (`config/slam_toolbox_params.yaml`):
- `minimum_travel_distance: 0.1` m (10cm updates)
- `minimum_travel_heading: 0.1` rad (~6° updates)
- `correlation_search_space_dimension: 1.5` m (wide search)
- `correlation_search_space_smear_deviation: 0.1` (max valid)
- `distance_variance_penalty: 0.3` (moderate odometry trust)
- `angle_variance_penalty: 0.5` (allow deviation)

---

## Performance Results

### Test 8: Full Mapping Session ✅ PASSED
- SLAM running without crashes
- Map updates smoothly as robot drives (~10cm intervals)
- LIDAR points stable and locked to map features
- No scattering or random jumps
- Strafe controls working correctly (Q=left, E=right)

### System Status
**All nodes operational**:
- `/rplidar_node` - Publishing /scan at ~7.6 Hz
- `/odometry_node` - Publishing /odom at 50 Hz (NO TF)
- `/ekf_filter_node` - Publishing odom→base_link TF
- `/slam_toolbox` - Publishing map→odom TF + /map topic
- `/motor_control_node` - Mecanum drive control
- `/camera_node` - Camera streaming
- `/web_teleop_node` - Web interface (http://10.21.21.45:8080)
- `/rviz` - Visualization

**TF Tree (verified working)**:
```
map (SLAM correction)
 └─ odom (EKF filtering)
     └─ base_link (robot frame)
         └─ laser (LIDAR sensor)
```

---

## Lessons Learned

1. **Always check for TF conflicts** - Multiple publishers of same transform = disaster
2. **Not all problems are hardware** - Encoders were fine, software config was wrong
3. **Silent crashes are hard to debug** - Check stderr logs and run nodes manually
4. **Parameter validation matters** - SLAM crashed on out-of-range value without clear error
5. **EKF is essential** - Filters noise before it becomes a TF transform

---

## Next Steps

- ⏳ **Test 9**: Map Saving - Save completed maps for future use
- ⏳ **Test 10**: Graceful Shutdown - Clean system shutdown

---

## Git Commits Summary

**ogre-slam repository**:
- `3d45c84` - Fix TF transform conflict (disable odometry TF)
- `ae81038` - Enable EKF to complete TF chain
- `9611029` - Fix SLAM crash (smear_deviation range)
- Plus earlier commits for config tuning

**ogre_teleop repository**:
- `4bfb7e3` - Fix inverted strafe direction

All code committed and pushed to GitHub.

---

## Current System State

**Repository**: git@github.com:protomota/ogre-slam.git
**Latest Commit**: 9611029
**Status**: ✅ Fully operational
**Ready for**: Mapping sessions and map persistence testing

**Launch Command**:
```bash
ros2 launch ogre_slam mapping.launch.py
```

**Web Interface**: http://10.21.21.45:8080

**Session**: Test 8 completed successfully! 🎉
