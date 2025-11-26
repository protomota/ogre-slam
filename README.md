# ogre-slam

SLAM mapping and autonomous navigation for Project Ogre mecanum drive robot.

## Overview

This is a **standalone ROS2 package** for SLAM and autonomous navigation on mecanum drive robots.

**Two-Computer Setup:**
- **Host Computer (Development)**: Runs Isaac Sim for simulation, RViz for visualization
- **Jetson Orin Nano (Robot)**: Runs on the actual robot with real sensors

**Workflow:** Develop and test in Isaac Sim on your host computer first, then deploy to the Jetson.

**Components:**
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
- ✅ Autonomous waypoint navigation with Nav2
- ✅ 3D obstacle avoidance using RealSense D435 pointcloud
- ✅ Mecanum-aware path planning (omnidirectional movement)
- ✅ RViz visualization with Nav2 panel
- ✅ Manual override via web teleop interface

## Hardware Requirements

- **Jetson Orin Nano** Developer Kit
- **Mecanum drive** robot with 4 motors (25GA-370 with gear_ratio 224.0)
- **Encoders**: 2 PPR Hall sensors on each motor
- **RPLIDAR** (A1/A2/A3 series) - 2D laser scanner
- **RealSense D435** depth camera - 3D obstacle detection
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

## Package Structure

```
ogre-slam/
├── ogre_slam/                   # Python module
│   ├── encoder_reader.py        # GPIO encoder reading
│   ├── mecanum_odometry.py      # Mecanum kinematics
│   └── odometry_node.py         # ROS2 odometry node
├── launch/
│   ├── mapping.launch.py        # SLAM mapping launch file
│   └── navigation.launch.py     # Nav2 navigation launch file
├── config/
│   ├── odometry_params.yaml     # Robot dimensions
│   ├── slam_toolbox_params.yaml # SLAM configuration
│   ├── nav2_params.yaml         # Nav2 stack configuration
│   └── ekf_params.yaml          # Sensor fusion config
├── scripts/
│   ├── launch_mapping_session.sh    # Unified launcher (Jetson)
│   ├── launch_isaac_sim_rviz.sh     # RViz for Isaac Sim
│   └── remote_launch_slam_rviz.sh   # Remote RViz launcher
├── maps/                        # Saved maps directory
├── rviz/                        # RViz configurations
├── usds/                        # Isaac Sim USD files
├── docs/                        # Additional documentation
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

### System Packages (Both Computers)

**⚠️ IMPORTANT:** Install these dependencies BEFORE building the package on BOTH Host Computer and Jetson:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-tf2-tools \
  ros-humble-teleop-twist-keyboard
```

**Note:** If you see errors about `packages.ros.org/ros` repository, ignore them - we only need ROS2 packages which will install correctly.

### Python Dependencies

**On Jetson only** (for GPIO encoder reading):
```bash
pip3 install Jetson.GPIO numpy
```

**On Host Computer** (for simulation):
```bash
pip3 install numpy
```

Note: `Jetson.GPIO` is only needed on the Jetson - it won't install on non-Jetson systems.

## Installation

**Install on BOTH Host Computer and Jetson:**

1. **Clone repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone git@github.com:protomota/ogre-slam.git
   ```

2. **Install dependencies** (see above)

3. **Build package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ogre_slam --symlink-install
   source install/setup.bash
   ```

4. **Jetson only - Configure robot dimensions** (CRITICAL):
   ```bash
   cd ~/ros2_ws/src/ogre-slam
   nano config/odometry_params.yaml
   ```

   Update these values with actual measurements:
   - `wheel_radius`: Distance from wheel axle to ground (meters)
   - `wheel_base`: Distance between front and rear axles (meters)
   - `track_width`: Distance between left and right wheels (meters)
   - `gear_ratio`: Motor gear ratio (see Gear Ratio Calibration below)

## SLAM Mapping

Build maps using Isaac Sim simulation first, then deploy to the real robot.

### Quick Comparison: Host Computer vs Jetson

| Aspect | Host Computer (Isaac Sim) | Jetson (Real Robot) |
|--------|---------------------------|---------------------|
| **Where** | Your development computer | Robot (10.21.21.45) |
| **Hardware** | Simulated sensors | Real RPLIDAR + encoders |
| **Launch Command** | `ros2 launch ogre_slam mapping.launch.py` with sim flags | `./scripts/launch_mapping_session.sh` |
| **Teleop** | ROS2 keyboard teleop | Web interface (http://10.21.21.45:8080) |
| **Odometry** | From Isaac Sim (`use_sim_time:=true`) | Encoder-based (default) |
| **ROS_DOMAIN_ID** | 42 | 42 |

---

### Mapping in Isaac Sim (Host Computer)

**Prerequisites:**
- NVIDIA Isaac Sim 5.0+ installed
- ogre.usd scene loaded
- Action graph configured (see "Isaac Sim Simulation" section below)

**Step 1: Start Isaac Sim (MUST DO FIRST)**

1. Launch Isaac Sim and load the `usds/ogre.usd` scene
2. Verify ROS2 Context node has **Domain ID = 42**
3. **Press Play ▶️** to start the simulation
4. Verify topics are publishing:
   ```bash
   export ROS_DOMAIN_ID=42
   ros2 topic list  # Should see /scan, /odom, /clock, /tf
   ```

**Step 2: Launch SLAM**

On your **Host Computer**:
```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_USE_SIM_TIME=true

ros2 launch ogre_slam mapping.launch.py \
  use_rviz:=true \
  use_teleop:=false \
  use_odometry:=false \
  use_ekf:=false \
  use_sim_time:=true
```

**What these flags mean:**
- `use_sim_time:=true` - Sync with Isaac Sim's simulation clock
- `use_odometry:=false` - Isaac Sim provides `/odom` topic
- `use_ekf:=false` - Sim odometry is already clean (no noisy encoders)
- `use_teleop:=false` - Don't launch ogre_teleop (not needed)
- `use_rviz:=true` - Show visualization

**Step 3: Drive the robot**

In another terminal:
```bash
export ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard controls:**
- **i** - Forward
- **,** - Backward
- **j** - Rotate left
- **l** - Rotate right
- **k** - Stop
- **q/z** - Increase/decrease max speeds

Drive slowly around the Isaac Sim environment and watch the map build in RViz!

**Step 4: Save the map**

When mapping is complete, in another terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/isaac_sim_map
```

This creates:
- `isaac_sim_map.yaml` - Map metadata
- `isaac_sim_map.pgm` - Map image

Preview the saved map:
```bash
eog ~/ros2_ws/src/ogre-slam/maps/isaac_sim_map.pgm
```

**Step 5: Stop the system**
```bash
# Press Ctrl+C in both terminals
```

---

### Mapping on Real Robot (Jetson)

**Prerequisites:**
- Robot powered on and connected to network (10.21.21.45)
- RPLIDAR connected and powered
- Encoders wired to GPIO pins
- ogre_teleop package installed

**Step 1: Launch mapping session**

On the **Jetson**:
```bash
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_mapping_session.sh
```

This launches:
- RPLIDAR driver
- Encoder-based odometry node
- EKF sensor fusion
- slam_toolbox (mapping mode)
- RViz visualization
- ogre_teleop web interface

**Step 2: Drive the robot**

Open browser on any device on the network:
```
http://10.21.21.45:8080
```

Drive slowly around the area:
- Use smooth, controlled movements
- Ensure good loop closures (revisit starting point)
- Watch RViz to see map being built

**Step 3: Save the map**

When mapping is complete, in another terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/my_map
```

This creates:
- `my_map.yaml` - Map metadata
- `my_map.pgm` - Map image

Preview the saved map:
```bash
eog ~/ros2_ws/src/ogre-slam/maps/my_map.pgm
```

**Step 4: Stop the system**
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

---

## Navigation Mode (Autonomous Waypoint Navigation)

Navigate using a saved map. Can run on either Host Computer (Isaac Sim) or Jetson (real robot).

**Prerequisites:**
1. Install Nav2 packages (see [NAV2_README.md](docs/NAV2_README.md))
2. Have a saved map from mapping mode (see SLAM Mapping above)

### Navigation in Isaac Sim (Host Computer)

**Step 1: Start Isaac Sim (MUST DO FIRST)**

1. Launch Isaac Sim and load the `usds/ogre.usd` scene
2. Verify ROS2 Context node has **Domain ID = 42**
3. **Press Play ▶️** to start the simulation
4. Verify topics are publishing:
   ```bash
   export ROS_DOMAIN_ID=42
   ros2 topic list  # Should see /scan, /odom, /clock, /tf
   ```

**Step 2: Launch Navigation**

```bash
cd ~/ros2_ws && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ogre_slam navigation.launch.py \
  map:=~/ros2_ws/src/ogre-slam/maps/isaac_sim_map.yaml \
  use_sim_time:=true
```

### Navigation on Real Robot (Jetson)

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml
```

This launches:
1. **Localization**: AMCL with saved map
2. **Sensors**: RPLIDAR + RealSense D435 pointcloud
3. **Nav2 Stack**: Path planning, obstacle avoidance, controller
4. **Control**: Web interface for manual override (Jetson only)

**Navigate to waypoints:**
1. In RViz, click "2D Pose Estimate" and set robot's initial position
2. Click "Nav2 Goal" button and click destination on map
3. Robot autonomously navigates, avoiding obstacles with depth camera
4. Manual override available at `http://10.21.21.45:8080` (Jetson only)

**📖 Full documentation:** See [NAV2_README.md](docs/NAV2_README.md) for complete guide

---

## Isaac Sim Simulation (Host Computer Only)

For testing with NVIDIA Isaac Sim 5.0 on your development computer:

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

### Isaac Sim Action Graph Configuration

**Required nodes for SLAM mapping:**

1. **ROS2 Context** - Set Domain ID = 42
2. **Isaac Read Simulation Time** - Provides timestamps
3. **ROS2 Publish Clock** - Publishes `/clock` topic
4. **Isaac Compute Odometry** - Computes robot odometry
5. **ROS2 Publish Odometry** - Publishes `/odom` topic
6. **ROS2 Publish Raw Transform Tree** - Publishes dynamic `odom→base_link` TF
7. **ROS2 Publish Transform Tree** - Publishes static `base_link→laser` TF
8. **ROS2 Publish LaserScan** - Publishes `/scan` topic from 2D PhysXLidar

**Critical connections:**
- Connect **Simulation Time** output from Isaac Read Simulation Time to **timeStamp** input on ALL ROS2 Publish nodes
- Set `parentFrameId: odom` and `childFrameId: base_link` in ROS2 Publish Raw Transform Tree
- Set `parentFrameId: base_link` and `childFrameId: laser` in ROS2 Publish Transform Tree

See `CLAUDE.md` for detailed action graph setup, robot dimensions, and complete configuration.

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

## Topics

### Published

- `/odom` (nav_msgs/Odometry) - Wheel odometry
- `/odometry/filtered` (nav_msgs/Odometry) - EKF-filtered odometry
- `/map` (nav_msgs/OccupancyGrid) - SLAM-generated map
- `/encoder_ticks` (std_msgs/Int32MultiArray) - Raw encoder counts (debugging)

### Subscribed

- `/scan` (sensor_msgs/LaserScan) - From RPLIDAR

### TF Frames

**Jetson (Real Robot):**
```
map (from slam_toolbox or amcl)
  └─ odom (from EKF or odometry_node)
      └─ base_link (robot center at wheel axle height)
          ├─ laser (RPLIDAR: 0.30m up, 180° rotated)
          └─ camera_link (RealSense D435: 0.15m forward, 0.10m up)
```

**Host Computer (Isaac Sim):**
```
odom (from Isaac Sim)
  └─ base_link
      ├─ laser (LIDAR: 0.30m up, 180° rotated)
      └─ camera_link → camera_depth (optical frame)
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

### Isaac Sim Physics Troubleshooting

If your robot exhibits bouncing, flipping, or unstable behavior in Isaac Sim, this is commonly due to physics configuration issues. Here are the primary causes and solutions:

#### Common Causes of Physics Instability

**1. Collider Misconfiguration (MOST COMMON)**
- Mecanum wheels require precise collision geometry
- Overlapping colliders (wheels with body/ground/each other) cause instability
- Complex mesh collisions (with roller details) overwhelm the physics solver

**Solution:**
- Enable collision visualization: **View → Show by Type → Physics → Colliders**
- Check for RED overlaps at rest - there should be minimal intrusion
- Use simple collision approximations: **Property → Physics → Collision → Approximation: "boundingCube"**
- Disable collision on passive rollers (keep only main wheel hub collision)

**2. Improper Mass/Inertia Settings**
- Unrealistic or unbalanced mass causes abnormal dynamics
- Auto-computed inertia from complex geometry can be incorrect

**Solution:**
- Verify realistic masses: Robot base ~3.5kg, wheels ~0.1kg each
- Check: **Property → Physics → Rigid Body → Mass**
- Ensure inertia tensors are computed from actual geometry (not default values)

**3. Excessive Maximum Velocity (CRITICAL - COMMON ISSUE)**
- Default Maximum Velocity on RevoluteJoints is often 1000000 (1 million!)
- This allows wheels to spin out of control, causing physics chaos and instability
- **This is the most common cause of Isaac Sim mecanum wheel bouncing**
- Too low Maximum Velocity prevents proper mecanum wheel slip behavior

**Solution:**
- For each wheel RevoluteJoint: **Property → Physics → Revolute Joint → Drive → Angular**
  - **Maximum Velocity:** **10000** (NOT 1000000!) - This is the critical fix!
    - Allows proper mecanum wheel slip while maintaining stability
    - Too low (e.g., 100) prevents wheels from slipping correctly
  - **Damping:** 1.0-10.0 (NOT 0!)
  - **Stiffness:** 0
  - **Max Force:** 100-1000 (tune for stability)

**4. Insufficient Joint Damping**
- Missing or low damping on wheel joints causes oscillations
- Too high stiffness can cause divergence

**Solution:**
- Apply damping values as listed above (1.0-10.0)

**5. Poor Friction Parameters**
- Mecanum wheels need balanced lateral and longitudinal friction
- Mismatched coefficients cause force buildup and "popping"

**Solution:**
- Wheels: **Property → Physics → Physics Material**
  - **Static Friction:** 0.8-1.0
  - **Dynamic Friction:** 0.6-0.8
- Ground plane: Match or slightly lower friction values

**6. Physics Timestep Issues**
- Too large timestep makes contacts unstable
- Insufficient solver iterations can't handle articulated complexity

**Solution:**
- Select `/physicsScene`: **Property → Physics Scene**
  - **Position Iteration Count:** 8 (default: 4)
  - **Velocity Iteration Count:** 2 (default: 1)
  - **Time Steps Per Second:** 120+ (smaller timestep = more stable)
  - **Enable CCD:** True (Continuous Collision Detection)

#### Diagnostic Steps

1. **Check Maximum Velocity (FIX THIS FIRST!):** Verify all wheel joints have Maximum Velocity = 10000 (NOT 1000000)
2. **Check for overlaps:** Enable collision visualization and look for red overlaps
3. **Increase damping:** Set joint damping to 5-10 if currently low/zero
4. **Simplify collision:** Use boundingCube approximation on wheels, disable roller collision
5. **Verify masses:** Check all rigid bodies have realistic mass values
6. **Increase solver quality:** Set position iterations to 8, velocity to 2

#### Quick Fix Checklist

- ✅ **MOST IMPORTANT:** Wheel joints Maximum Velocity set to **10000** (NOT 1000000!)
  - Allows proper mecanum wheel slip while maintaining stability
- ✅ Wheel joints have damping ≥ 1.0
- ✅ No red collision overlaps visible at rest
- ✅ Wheel collision approximation set to "boundingCube" or "boundingSphere"
- ✅ All 28 passive rollers have collision disabled
- ✅ Physics scene position iterations ≥ 8
- ✅ Wheels not penetrating ground (Z position = wheel radius + 5-10mm clearance)

**Reference:** [NVIDIA Isaac Sim Forum - Mecanum Wheel Physics](https://forums.developer.nvidia.com/t/351181)

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

## Related Projects

### Project Ogre Ecosystem

This repository is part of the **Project Ogre** ecosystem for mecanum drive robot navigation:

| Repository | Purpose | Key Contents |
|------------|---------|--------------|
| **ogre-slam** (this repo) | SLAM & Navigation | ROS2 package, Nav2 config, robot USD models, sensor drivers |
| **[ogre-lab](https://github.com/protomota/ogre-lab)** | RL policy training | Isaac Lab environment, trained models, ROS2 policy controller |

### How They Work Together

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           TRAINING (ogre-lab)                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                  │
│  │ Isaac Lab    │───▶│ Train Policy │───▶│ Export ONNX  │                  │
│  │ Environment  │    │ (RSL-RL/PPO) │    │ & JIT Models │                  │
│  └──────────────┘    └──────────────┘    └──────────────┘                  │
│         │                                        │                          │
│         │ uses                                   │ produces                 │
│         ▼                                        ▼                          │
│  ogre_robot.usd                          models/policy.onnx                 │
│  (from ogre-slam)                        models/policy.pt                   │
└─────────────────────────────────────────────────────────────────────────────┘
                                   │
                                   │ deploy
                                   ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          DEPLOYMENT (ogre-slam)                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                  │
│  │    Nav2      │───▶│   Policy     │───▶│ Robot/Isaac  │                  │
│  │ (planning)   │    │  Controller  │    │     Sim      │                  │
│  └──────────────┘    └──────────────┘    └──────────────┘                  │
│                             │                                               │
│                             │ runs                                          │
│                             ▼                                               │
│                      policy.onnx (from ogre-lab)                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Key Files Shared Between Repos

| File | Location | Used By |
|------|----------|---------|
| `ogre_robot.usd` | ogre-slam/usds/ | ogre-lab training environment |
| `ogre.usd` | ogre-slam/usds/ | Isaac Sim deployment testing |
| `policy.onnx` | ogre-lab/models/ | ogre-slam policy controller (via ROS2 package) |

### Using the Trained RL Policy

The trained policy from ogre-lab improves velocity tracking by learning the robot's dynamics. It sits between Nav2 and the robot.

#### One-Time Setup

```bash
# Symlink the policy controller package (from ogre-lab)
ln -sf ~/ogre-lab/ros2_controller ~/ros2_ws/src/ogre_policy_controller

# Install onnxruntime (required for policy inference)
pip install onnxruntime

# Build the package
cd ~/ros2_ws
colcon build --packages-select ogre_policy_controller --symlink-install
source install/setup.bash
```

#### Testing with Isaac Sim

**Terminal 1: Start Isaac Sim**
- Open Isaac Sim
- Load `~/ros2_ws/src/ogre-slam/usds/ogre.usd`
- Press **Play** ▶️

**Terminal 2: Launch the Policy Controller**
```bash
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash
ros2 launch ogre_policy_controller policy_controller.launch.py
```

**Terminal 3: Send Test Commands**
```bash
export ROS_DOMAIN_ID=42

# Test forward motion
ros2 topic pub /policy_cmd_vel_in geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Test strafe left
ros2 topic pub /policy_cmd_vel_in geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Test rotation
ros2 topic pub /policy_cmd_vel_in geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
```

**Terminal 4: Monitor Output (Optional)**
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /cmd_vel
```

#### Integration with Nav2

To use the policy with Nav2, remap Nav2's velocity output:

```yaml
# In your Nav2 params file
controller_server:
  ros__parameters:
    cmd_vel_topic: "/policy_cmd_vel_in"
```

The data flow becomes:
```
Nav2 → /policy_cmd_vel_in → Policy Controller → /cmd_vel → Robot
```

See [ogre-lab README](https://github.com/protomota/ogre-lab) for training instructions.

## Contributing

This is part of Project Ogre. For issues or contributions:
- GitHub: https://github.com/protomota/ogre-slam

## License

MIT License

## Acknowledgments

- **slam_toolbox**: Steve Macenski
- **robot_localization**: Tom Moore
- **RPLIDAR ROS**: SLAMTEC
- **Isaac Lab**: NVIDIA (for RL training framework)

---

**Robot IP:** 10.21.21.45
**ROS2 Distro:** Humble
**Platform:** Jetson Orin Nano
