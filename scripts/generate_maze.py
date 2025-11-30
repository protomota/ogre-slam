#!/usr/bin/env python3
"""
Generate a maze for Isaac Sim using cubes as walls.

Wall dimensions: 25.5cm wide x 38.5cm tall x 2cm thick
Maze size: 8x8 cells
Output: maze.usd (standalone USD file)

Usage:
    python3 generate_maze.py
"""

import random
from pxr import Usd, UsdGeom, Gf, UsdPhysics


class MazeGenerator:
    """Generate a maze using recursive backtracking algorithm."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[{'N': True, 'S': True, 'E': True, 'W': True}
                      for _ in range(width)] for _ in range(height)]
        self.generate()

    def generate(self):
        """Generate maze using depth-first search with backtracking."""
        visited = set()
        stack = [(0, 0)]
        visited.add((0, 0))

        while stack:
            x, y = stack[-1]
            neighbors = []

            # Check all four directions
            for dx, dy, direction, opposite in [
                (0, -1, 'N', 'S'),  # North
                (0, 1, 'S', 'N'),   # South
                (1, 0, 'E', 'W'),   # East
                (-1, 0, 'W', 'E')   # West
            ]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.width and 0 <= ny < self.height and
                    (nx, ny) not in visited):
                    neighbors.append((nx, ny, direction, opposite))

            if neighbors:
                # Choose random unvisited neighbor
                nx, ny, direction, opposite = random.choice(neighbors)

                # Remove walls between current cell and chosen neighbor
                self.grid[y][x][direction] = False
                self.grid[ny][nx][opposite] = False

                visited.add((nx, ny))
                stack.append((nx, ny))
            else:
                stack.pop()


def create_maze_usd(maze, cell_size=0.255, wall_height=0.385, wall_thickness=0.02,
                    usd_file="ogre.usd", maze_x=-3.6, maze_y=-1.3, maze_z=0.6):
    """
    Add maze walls to an existing USD file (or create new one).

    Args:
        maze: MazeGenerator instance
        cell_size: Size of each cell in meters (0.255m = 25.5cm)
        wall_height: Height of walls in meters (0.385m = 38.5cm)
        wall_thickness: Thickness of walls in meters (0.02m = 2cm)
        usd_file: USD file to modify (creates if doesn't exist)
        maze_x: X position for maze Xform (meters)
        maze_y: Y position for maze Xform (meters)
        maze_z: Z position for maze Xform (meters)
    """
    # Open existing USD stage or create new one
    import os
    if os.path.exists(usd_file):
        print(f"📂 Opening existing file: {usd_file}")
        stage = Usd.Stage.Open(usd_file)
    else:
        print(f"📄 Creating new file: {usd_file}")
        stage = Usd.Stage.CreateNew(usd_file)

    # Remove old maze if it exists
    old_maze = stage.GetPrimAtPath('/Maze')
    if old_maze.IsValid():
        print(f"🗑️  Removing old maze...")
        stage.RemovePrim('/Maze')

    # Create root prim for maze with position
    maze_prim = UsdGeom.Xform.Define(stage, '/Maze')
    maze_prim.AddTranslateOp().Set(Gf.Vec3d(maze_x, maze_y, maze_z))

    # Enable physics scene
    scene = UsdPhysics.Scene.Define(stage, '/physicsScene')
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    wall_count = 0

    # Create walls
    for y in range(maze.height):
        for x in range(maze.width):
            cell = maze.grid[y][x]

            # Cell center position (relative to maze origin, no offset since Xform handles it)
            cx = x * cell_size
            cy = y * cell_size
            cz = wall_height / 2  # Center wall vertically

            # North wall (horizontal, along X axis)
            if cell['N']:
                wall_name = f'/Maze/Wall_{wall_count}'
                wall_count += 1

                cube = UsdGeom.Cube.Define(stage, wall_name)
                cube.CreateSizeAttr(1.0)  # Unit cube, will be scaled

                # Position at north edge of cell
                pos = Gf.Vec3d(cx + cell_size/2, cy, cz)
                cube.AddTranslateOp().Set(pos)

                # Scale: X=cell_size, Y=thickness, Z=height
                scale = Gf.Vec3f(cell_size, wall_thickness, wall_height)
                cube.AddScaleOp().Set(scale)

                # Add physics - collision and static rigid body
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                rigid_body.CreateKinematicEnabledAttr(True)  # Static wall, doesn't move

            # West wall (horizontal, along Y axis)
            if cell['W']:
                wall_name = f'/Maze/Wall_{wall_count}'
                wall_count += 1

                cube = UsdGeom.Cube.Define(stage, wall_name)
                cube.CreateSizeAttr(1.0)

                # Position at west edge of cell
                pos = Gf.Vec3d(cx, cy + cell_size/2, cz)
                cube.AddTranslateOp().Set(pos)

                # Scale: X=thickness, Y=cell_size, Z=height
                scale = Gf.Vec3f(wall_thickness, cell_size, wall_height)
                cube.AddScaleOp().Set(scale)

                # Add physics - collision and static rigid body
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                rigid_body.CreateKinematicEnabledAttr(True)  # Static wall, doesn't move

    # Add south boundary wall for last row
    y = maze.height - 1
    for x in range(maze.width):
        if maze.grid[y][x]['S']:
            wall_name = f'/Maze/Wall_{wall_count}'
            wall_count += 1

            cx = x * cell_size
            cy = (y + 1) * cell_size
            cz = wall_height / 2

            cube = UsdGeom.Cube.Define(stage, wall_name)
            cube.CreateSizeAttr(1.0)

            pos = Gf.Vec3d(cx + cell_size/2, cy, cz)
            cube.AddTranslateOp().Set(pos)

            scale = Gf.Vec3f(cell_size, wall_thickness, wall_height)
            cube.AddScaleOp().Set(scale)

            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid_body.CreateKinematicEnabledAttr(True)

    # Add east boundary wall for last column
    x = maze.width - 1
    for y in range(maze.height):
        if maze.grid[y][x]['E']:
            wall_name = f'/Maze/Wall_{wall_count}'
            wall_count += 1

            cx = (x + 1) * cell_size
            cy = y * cell_size
            cz = wall_height / 2

            cube = UsdGeom.Cube.Define(stage, wall_name)
            cube.CreateSizeAttr(1.0)

            pos = Gf.Vec3d(cx, cy + cell_size/2, cz)
            cube.AddTranslateOp().Set(pos)

            scale = Gf.Vec3f(wall_thickness, cell_size, wall_height)
            cube.AddScaleOp().Set(scale)

            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid_body.CreateKinematicEnabledAttr(True)

    # Save stage
    stage.GetRootLayer().Save()

    print(f"✅ Maze added successfully!")
    print(f"   File: {usd_file}")
    print(f"   Walls: {wall_count}")
    print(f"   Maze size: {maze.width}x{maze.height} cells")
    print(f"   Physical size: {maze.width * cell_size:.2f}m x {maze.height * cell_size:.2f}m")
    print(f"   Wall dimensions: {cell_size*100:.1f}cm long x {wall_height*100:.1f}cm tall x {wall_thickness*100:.1f}cm thick")
    print(f"   Maze position: ({maze_x:.2f}m, {maze_y:.2f}m, {maze_z:.2f}m)")
    print(f"   Physics: ✅ Rigid body collision enabled")
    print(f"\nTo view in Isaac Sim:")
    print(f"   File → Open → {usd_file}")


def main():
    """Generate maze and add to ogre.usd file."""
    print("🏗️  Generating 4x4 WIDE maze for Nav2 testing...")

    # Generate smaller maze with wider corridors
    random.seed()  # Random each time
    maze = MazeGenerator(width=4, height=4)

    # Clear center cells to ensure large open space for robot start
    # For 4x4, clear all 4 center cells
    maze.grid[1][1] = {'N': False, 'S': False, 'E': False, 'W': False}
    maze.grid[1][2] = {'N': False, 'S': False, 'E': False, 'W': False}
    maze.grid[2][1] = {'N': False, 'S': False, 'E': False, 'W': False}
    maze.grid[2][2] = {'N': False, 'S': False, 'E': False, 'W': False}

    # Calculate centered position
    # Robot: 205mm wide × 95mm long (diagonal: 226mm)
    # Cell size: 1.5m = 1500mm (HUGE clearance for fast navigation!)
    # 4 cells × 1.5m = 6.0m total maze size
    # To center at origin: offset by -half_size = -3.0m
    maze_size = 4 * 1.5  # 6.0m
    center_offset = -maze_size / 2  # -3.0m

    # Add maze to ogre.usd (wide corridors for Nav2)
    create_maze_usd(
        maze,
        cell_size=1.5,        # 1.5m WIDE corridors (plenty of room!)
        wall_height=1.00,     # 100cm tall (1 meter - excellent LIDAR visibility)
        wall_thickness=0.20,  # 20cm thick (4x costmap resolution for reliable detection)
        usd_file="/home/brad/ros2_ws/src/ogre-slam/ogre.usd",
        maze_x=center_offset, # Centered X (-3.0m)
        maze_y=center_offset, # Centered Y (-3.0m)
        maze_z=0.6            # Wall base at 60cm height
    )

    print("\n📝 Wide Maze Configuration:")
    print("   - Maze size: 4×4 cells (smaller but WIDER)")
    print("   - Cell size: 1.5m × 1.5m (2.5x wider than before!)")
    print("   - Total size: 6.0m × 6.0m")
    print("   - Clearance: 1274mm (5.6x robot diagonal!)")
    print("   - Center: 4 cells cleared for large starting area")
    print("\n⚠️  Note: This modifies ogre.usd - make a backup first!")
    print("💡 Tip: Wide corridors allow Nav2 to drive fast and turn easily")
    print("💡 Perfect for testing autonomous navigation at high speeds")


if __name__ == "__main__":
    main()
