# Maze Generator for Isaac Sim

Automatically generate mazes for SLAM testing in Isaac Sim.

## Quick Start

### Option 1: Run from Script Editor (Recommended)

1. **Open Isaac Sim**
2. **Window → Script Editor**
3. **File → Open** → Select `scripts/generate_maze.py`
4. **Click "Run"** button
5. **Reload ogre.usd** in Isaac Sim to see the maze

### Option 2: Run from Command Line

```bash
cd ~/ros2_ws/src/ogre-slam
./scripts/run_maze_generator.sh
```

Then reload ogre.usd in Isaac Sim.

## Default Configuration (Wide Maze for Nav2)

- **Maze Size:** 4×4 cells (smaller grid, WIDER corridors)
- **Cell Size:** 1.5m × 1.5m (HUGE clearance for fast autonomous navigation!)
- **Wall Dimensions:** 1.5m long × 1.0m tall × 2cm thick
- **Total Physical Size:** 6.0m × 6.0m
- **Position:** Centered at (-3.0m, -3.0m, 0.6m) with 4-cell open center
- **Physics:** Rigid body collision with kinematic mode (static walls)
- **Algorithm:** Recursive backtracking (guarantees solution)

**Note:** The 4 center cells (1,1), (1,2), (2,1), (2,2) are always cleared to provide a large open starting area for the robot.

**Robot Fit:** Designed for 205mm×95mm robot (diagonal 226mm). Cell size provides **1274mm clearance** (5.6× robot diagonal) for comfortable high-speed autonomous navigation with Nav2.

**Why So Wide?**
- Nav2's DWB controller needs room for trajectory planning
- 1.5m corridors allow robot to navigate at 8 m/s without obstacle avoidance interference
- Wide corridors prevent conservative behavior from inflation layer
- Perfect for testing autonomous navigation algorithms at high speeds

**Previous Configuration:** 5×5 cells with 0.6m size (3.0m × 3.0m total, 374mm clearance) - too narrow for aggressive Nav2 settings.

## Customization

Edit `scripts/generate_maze.py`:

```python
# Change maze size
maze = MazeGenerator(width=10, height=10)  # 10x10 instead of 5x5

# Change cell size and wall dimensions
create_maze_usd(
    maze,
    cell_size=1.00,        # 100cm cells (even more space)
    wall_height=1.00,      # 100cm tall (default height)
    wall_thickness=0.02,   # 2cm thick
    maze_x=-2.5,           # Maze Xform position X (adjust for new size)
    maze_y=-2.5,           # Maze Xform position Y (adjust for new size)
    maze_z=0.6             # Maze Xform position Z (wall base height)
)

# Get same maze every time (reproducible)
random.seed(42)  # Add at top of main()
maze = MazeGenerator(width=5, height=5)
```

## Features

✅ **Physics Collision** - Walls have rigid body physics (kinematic mode)
✅ **Random Generation** - Different maze each time
✅ **Guaranteed Solution** - Always has a path from start to end
✅ **Customizable** - Easy to adjust size, dimensions, position
✅ **Static Walls** - Walls don't move when robot collides

## Backup & Restore

A backup is automatically created: `ogre_backup.usd`

**To restore original:**
```bash
cd ~/ros2_ws/src/ogre-slam
cp ogre_backup.usd ogre.usd
```

## Using the Maze

1. **Generate maze** (see Quick Start above)
2. **Load ogre.usd** in Isaac Sim
3. **Press Play** ▶️
4. **Run SLAM mapping:**
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
5. **Drive robot** with teleop keyboard
6. **Watch map build** in RViz!

## Troubleshooting

**"ModuleNotFoundError: No module named 'pxr'"**
- You're not using Isaac Sim's Python
- Use Script Editor method instead

**Maze too big/small?**
- Adjust `MazeGenerator(width=X, height=Y)`
- Default is 5x5 cells = 3.0m × 3.0m (60cm per cell)

**Robot collides with walls?**
- Walls already have collision enabled
- Check robot radius vs. maze cell size
- Increase cell_size for wider paths

**Want different maze?**
- Run script again (generates random maze)
- OR use `random.seed(N)` for reproducible mazes

## Algorithm Details

**Recursive Backtracking (Depth-First Search):**
1. Start at cell (0,0)
2. Pick random unvisited neighbor
3. Remove wall between cells
4. Move to neighbor and repeat
5. Backtrack when no unvisited neighbors
6. Guarantees perfect maze (exactly one solution path)

## Files

- `scripts/generate_maze.py` - Main maze generator script
- `scripts/run_maze_generator.sh` - Launcher script (auto-finds Isaac Sim Python)
- `ogre.usd` - Your robot scene (maze will be added here)
- `ogre_backup.usd` - Backup created automatically

---

**Tips:**
- Start with default 4×4 wide maze for Nav2 testing (6m × 6m)
- 1.5m cell size provides 1274mm clearance for fast autonomous navigation
- Wider corridors prevent Nav2 from being overly conservative
- Smaller grid (4×4) maps quickly in Isaac Sim
- For slower navigation/tighter spaces, reduce cell_size to 0.6m-1.0m
- Thin walls (2cm) match real cardboard for realistic collision testing
