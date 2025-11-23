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
                    usd_file="ogre.usd", offset_x=1.0, offset_y=1.0):
    """
    Add maze walls to an existing USD file (or create new one).

    Args:
        maze: MazeGenerator instance
        cell_size: Size of each cell in meters (0.255m = 25.5cm)
        wall_height: Height of walls in meters (0.385m = 38.5cm)
        wall_thickness: Thickness of walls in meters (0.02m = 2cm)
        usd_file: USD file to modify (creates if doesn't exist)
        offset_x: X offset for maze placement (meters)
        offset_y: Y offset for maze placement (meters)
    """
    # Open existing USD stage or create new one
    import os
    if os.path.exists(usd_file):
        print(f"📂 Opening existing file: {usd_file}")
        stage = Usd.Stage.Open(usd_file)
    else:
        print(f"📄 Creating new file: {usd_file}")
        stage = Usd.Stage.CreateNew(usd_file)

    # Create root prim for maze
    maze_prim = UsdGeom.Xform.Define(stage, '/Maze')

    # Enable physics scene
    scene = UsdPhysics.Scene.Define(stage, '/physicsScene')
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    wall_count = 0

    # Create walls
    for y in range(maze.height):
        for x in range(maze.width):
            cell = maze.grid[y][x]

            # Cell center position (with offset)
            cx = offset_x + x * cell_size
            cy = offset_y + y * cell_size
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

                # Add collision
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

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

                # Add collision
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Add south boundary wall for last row
    y = maze.height - 1
    for x in range(maze.width):
        if maze.grid[y][x]['S']:
            wall_name = f'/Maze/Wall_{wall_count}'
            wall_count += 1

            cx = offset_x + x * cell_size
            cy = offset_y + (y + 1) * cell_size
            cz = wall_height / 2

            cube = UsdGeom.Cube.Define(stage, wall_name)
            cube.CreateSizeAttr(1.0)

            pos = Gf.Vec3d(cx + cell_size/2, cy, cz)
            cube.AddTranslateOp().Set(pos)

            scale = Gf.Vec3f(cell_size, wall_thickness, wall_height)
            cube.AddScaleOp().Set(scale)

            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Add east boundary wall for last column
    x = maze.width - 1
    for y in range(maze.height):
        if maze.grid[y][x]['E']:
            wall_name = f'/Maze/Wall_{wall_count}'
            wall_count += 1

            cx = offset_x + (x + 1) * cell_size
            cy = offset_y + y * cell_size
            cz = wall_height / 2

            cube = UsdGeom.Cube.Define(stage, wall_name)
            cube.CreateSizeAttr(1.0)

            pos = Gf.Vec3d(cx, cy + cell_size/2, cz)
            cube.AddTranslateOp().Set(pos)

            scale = Gf.Vec3f(wall_thickness, cell_size, wall_height)
            cube.AddScaleOp().Set(scale)

            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Save stage
    stage.GetRootLayer().Save()

    print(f"✅ Maze added successfully!")
    print(f"   File: {usd_file}")
    print(f"   Walls: {wall_count}")
    print(f"   Maze size: {maze.width}x{maze.height} cells")
    print(f"   Physical size: {maze.width * cell_size:.2f}m x {maze.height * cell_size:.2f}m")
    print(f"   Wall dimensions: {cell_size*100:.1f}cm wide x {wall_height*100:.1f}cm tall x {wall_thickness*100:.1f}cm thick")
    print(f"   Position offset: ({offset_x:.2f}m, {offset_y:.2f}m)")
    print(f"\nTo view in Isaac Sim:")
    print(f"   File → Open → {usd_file}")


def main():
    """Generate maze and add to ogre.usd file."""
    print("🏗️  Generating 8x8 maze for Project Ogre...")

    # Generate maze
    maze = MazeGenerator(width=8, height=8)

    # Add maze to ogre.usd
    create_maze_usd(
        maze,
        cell_size=0.60,       # 60cm open space (robot can drive through)
        wall_height=0.385,    # 38.5cm tall
        wall_thickness=0.02,  # 2cm thick
        usd_file="/home/brad/ros2_ws/src/ogre-slam/ogre.usd",
        offset_x=1.0,         # 1m offset from origin
        offset_y=1.0          # 1m offset from origin
    )

    print("\n📝 Customization options:")
    print("   Edit the script to change:")
    print("   - Maze size: MazeGenerator(width=8, height=8)")
    print("   - Cell size: cell_size=0.60 (60cm paths)")
    print("   - Wall height: wall_height=0.385 (38.5cm tall)")
    print("   - Wall thickness: wall_thickness=0.02 (2cm thick)")
    print("   - Position: offset_x=1.0, offset_y=1.0")
    print("   - Random seed: random.seed(42) for reproducible mazes")
    print("\n⚠️  Note: This modifies ogre.usd - make a backup first!")
    print("💡 Tip: cell_size controls the drivable space, not wall dimensions!")


if __name__ == "__main__":
    main()
