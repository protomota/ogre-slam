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

## Default Configuration

- **Maze Size:** 5×5 cells (smaller, easier to map)
- **Cell Size:** 60cm × 60cm (open space for driving)
- **Wall Dimensions:** 60cm long × 100cm tall × 2cm thick
- **Total Physical Size:** 3.0m × 3.0m
- **Position:** Centered at (-1.5m, -1.5m, 0.6m) with open center cell
- **Physics:** Rigid body collision with kinematic mode (static walls)
- **Algorithm:** Recursive backtracking (guarantees solution)

**Note:** The center cell (2,2) is always cleared to provide an open starting area for the robot. Cell size controls the drivable space between walls.

**Robot Fit:** Designed for 205mm×95mm robot (diagonal 226mm). Cell size provides 374mm clearance for comfortable mecanum omnidirectional navigation.

**Alternative:** For wider robot configuration (340mm track), see `OGRE_WIDE.md` for 80cm cell size.

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
- Start with default 5x5 maze for testing (3m × 3m)
- Smaller mazes (3x3) map faster but less interesting
- Larger mazes (8x8+) test SLAM loop closure
- Cell size 60cm fits 205mm robot with 374mm clearance
- For wider robot (340mm track), use 80cm cells (see OGRE_WIDE.md)
- Thin walls (2cm) match real cardboard
