"""
Mecanum Drive Odometry Kinematics
Calculates robot velocity and pose from wheel encoder measurements
"""
import math
from typing import Tuple, List
import numpy as np


class MecanumOdometry:
    """
    Forward kinematics for mecanum drive robot

    Robot Configuration (X-configuration):
        M4 (FL) --- M1 (FR)
           |    X    |
        M3 (RL) --- M2 (RR)

    Wheel arrangement:
    - M1 (Front-Right): \ rollers
    - M2 (Rear-Right):  / rollers
    - M3 (Rear-Left):   \ rollers
    - M4 (Front-Left):  / rollers

    Coordinate system:
    - X: Forward (positive = robot moves forward)
    - Y: Left (positive = robot moves left)
    - Theta: Rotation (positive = CCW)
    """

    def __init__(self,
                 wheel_radius: float,
                 wheel_base: float,
                 track_width: float,
                 encoder_ppr: int = 2,
                 gear_ratio: float = 1.0):
        """
        Initialize mecanum odometry calculator

        Args:
            wheel_radius: Radius of mecanum wheel (meters)
            wheel_base: Distance between front and rear axles (meters)
            track_width: Distance between left and right wheels (meters)
            encoder_ppr: Encoder pulses per revolution (default 2 for Hall sensors)
            gear_ratio: Motor gear ratio (default 1.0, adjust if geared)
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.encoder_ppr = encoder_ppr
        self.gear_ratio = gear_ratio

        # Robot geometry constant
        self.lx_plus_ly = (wheel_base + track_width) / 2.0

        # Conversion factor: ticks → meters
        self.ticks_to_meters = (2 * math.pi * wheel_radius) / (encoder_ppr * gear_ratio)

        # Robot pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def wheel_velocities_to_robot_velocity(self,
                                           v_fl: float,
                                           v_fr: float,
                                           v_rl: float,
                                           v_rr: float) -> Tuple[float, float, float]:
        """
        Convert wheel velocities to robot velocity (forward kinematics)

        Args:
            v_fl: Front-left wheel velocity (m/s) - M4
            v_fr: Front-right wheel velocity (m/s) - M1
            v_rl: Rear-left wheel velocity (m/s) - M3
            v_rr: Rear-right wheel velocity (m/s) - M2

        Returns:
            (vx, vy, vtheta): Robot linear X, linear Y, angular Z velocities
        """
        # Mecanum forward kinematics (X-configuration)
        vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        vtheta = (-v_fl - v_fr + v_rl + v_rr) / (4.0 * self.lx_plus_ly)

        return vx, vy, vtheta

    def ticks_to_wheel_velocities(self,
                                   ticks: List[int],
                                   dt: float) -> Tuple[float, float, float, float]:
        """
        Convert encoder ticks to wheel velocities

        Args:
            ticks: List of signed encoder ticks [M1, M2, M3, M4] = [FR, RR, RL, FL]
            dt: Time interval (seconds)

        Returns:
            (v_fl, v_fr, v_rl, v_rr): Wheel velocities in m/s
        """
        if dt <= 0.0:
            return 0.0, 0.0, 0.0, 0.0

        # ticks order: [M1, M2, M3, M4] = [FR, RR, RL, FL]
        # Rearrange to [FL, FR, RL, RR]
        ticks_fl = ticks[3]  # M4
        ticks_fr = ticks[0]  # M1
        ticks_rl = ticks[2]  # M3
        ticks_rr = ticks[1]  # M2

        # Convert ticks to distance
        dist_fl = ticks_fl * self.ticks_to_meters
        dist_fr = ticks_fr * self.ticks_to_meters
        dist_rl = ticks_rl * self.ticks_to_meters
        dist_rr = ticks_rr * self.ticks_to_meters

        # Calculate velocities
        v_fl = dist_fl / dt
        v_fr = dist_fr / dt
        v_rl = dist_rl / dt
        v_rr = dist_rr / dt

        return v_fl, v_fr, v_rl, v_rr

    def update_pose(self, vx: float, vy: float, vtheta: float, dt: float):
        """
        Update robot pose using robot velocities (dead reckoning)

        Args:
            vx: Linear velocity in X (forward) direction (m/s)
            vy: Linear velocity in Y (left) direction (m/s)
            vtheta: Angular velocity (rad/s)
            dt: Time interval (seconds)
        """
        # Update pose using simple integration
        # Transform robot velocities to global frame
        dx_global = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        dy_global = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        dtheta = vtheta * dt

        self.x += dx_global
        self.y += dy_global
        self.theta += dtheta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get current robot pose

        Returns:
            (x, y, theta): Position in meters and orientation in radians
        """
        return self.x, self.y, self.theta

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        Reset robot pose to specified values

        Args:
            x: X position (meters)
            y: Y position (meters)
            theta: Orientation (radians)
        """
        self.x = x
        self.y = y
        self.theta = theta

    def compute_odometry(self,
                        ticks: List[int],
                        dt: float) -> Tuple[float, float, float, float, float, float]:
        """
        Complete odometry calculation from encoder ticks

        Args:
            ticks: Encoder ticks [M1, M2, M3, M4]
            dt: Time since last update (seconds)

        Returns:
            (x, y, theta, vx, vy, vtheta): Pose and velocities
        """
        # Convert ticks to wheel velocities
        v_fl, v_fr, v_rl, v_rr = self.ticks_to_wheel_velocities(ticks, dt)

        # Calculate robot velocities
        vx, vy, vtheta = self.wheel_velocities_to_robot_velocity(v_fl, v_fr, v_rl, v_rr)

        # Update pose
        self.update_pose(vx, vy, vtheta, dt)

        return self.x, self.y, self.theta, vx, vy, vtheta


if __name__ == "__main__":
    # Test mecanum odometry calculations
    print("Testing Mecanum Odometry Kinematics")
    print("=" * 50)

    # Robot parameters (estimates - adjust after measurement)
    wheel_radius = 0.05  # 5cm radius
    wheel_base = 0.25    # 25cm between front/rear
    track_width = 0.30   # 30cm between left/right
    encoder_ppr = 2      # 2 pulses per revolution

    odom = MecanumOdometry(
        wheel_radius=wheel_radius,
        wheel_base=wheel_base,
        track_width=track_width,
        encoder_ppr=encoder_ppr
    )

    print(f"Wheel radius: {wheel_radius}m")
    print(f"Wheel base: {wheel_base}m")
    print(f"Track width: {track_width}m")
    print(f"Encoder PPR: {encoder_ppr}")
    print(f"Ticks to meters: {odom.ticks_to_meters:.6f} m/tick")
    print()

    # Test 1: Forward movement (all wheels same direction)
    print("Test 1: Forward movement")
    ticks = [10, 10, 10, 10]  # [M1, M2, M3, M4]
    dt = 0.1
    x, y, theta, vx, vy, vtheta = odom.compute_odometry(ticks, dt)
    print(f"  Ticks: {ticks}")
    print(f"  Robot velocity: vx={vx:.3f} m/s, vy={vy:.3f} m/s, vtheta={vtheta:.3f} rad/s")
    print(f"  Pose: x={x:.3f}m, y={y:.3f}m, theta={math.degrees(theta):.1f}°")
    print()

    # Reset for next test
    odom.reset_pose()

    # Test 2: Strafe left (mecanum-specific)
    print("Test 2: Strafe left")
    ticks = [-10, 10, 10, -10]  # [M1, M2, M3, M4]
    x, y, theta, vx, vy, vtheta = odom.compute_odometry(ticks, dt)
    print(f"  Ticks: {ticks}")
    print(f"  Robot velocity: vx={vx:.3f} m/s, vy={vy:.3f} m/s, vtheta={vtheta:.3f} rad/s")
    print(f"  Pose: x={x:.3f}m, y={y:.3f}m, theta={math.degrees(theta):.1f}°")
    print()

    # Reset for next test
    odom.reset_pose()

    # Test 3: Rotate CCW
    print("Test 3: Rotate counter-clockwise")
    ticks = [-10, -10, 10, 10]  # [M1, M2, M3, M4]
    x, y, theta, vx, vy, vtheta = odom.compute_odometry(ticks, dt)
    print(f"  Ticks: {ticks}")
    print(f"  Robot velocity: vx={vx:.3f} m/s, vy={vy:.3f} m/s, vtheta={vtheta:.3f} rad/s")
    print(f"  Pose: x={x:.3f}m, y={y:.3f}m, theta={math.degrees(theta):.1f}°")
    print()

    print("✅ Tests complete!")
