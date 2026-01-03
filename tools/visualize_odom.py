#!/usr/bin/env python3
"""Visualize odometry trajectory from a ROS bag file."""

import argparse
import math
import matplotlib.pyplot as plt
import numpy as np

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    parser = argparse.ArgumentParser(description='Visualize odometry from bag file')
    parser.add_argument('bag_file', help='Path to bag file')
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic')
    parser.add_argument('--output', default='odom_trajectory.png', help='Output image file')
    args = parser.parse_args()

    # Read odom data
    positions = []
    orientations = []
    timestamps = []

    print(f"Reading odom from {args.bag_file}...")

    typestore = get_typestore(Stores.ROS1_NOETIC)

    with Reader(args.bag_file) as reader:
        # Get connections for odom topic
        connections = [c for c in reader.connections if c.topic == args.odom_topic]

        if not connections:
            print(f"Topic {args.odom_topic} not found in bag!")
            print("Available topics:")
            for c in reader.connections:
                print(f"  {c.topic}")
            return

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            positions.append((x, y))
            orientations.append(yaw)
            timestamps.append(timestamp / 1e9)  # Convert to seconds

    if not positions:
        print("No odom data found!")
        return

    positions = np.array(positions)
    orientations = np.array(orientations)
    timestamps = np.array(timestamps)

    # Relative timestamps
    timestamps = timestamps - timestamps[0]

    print(f"Loaded {len(positions)} odom messages")
    print(f"Duration: {timestamps[-1]:.2f} seconds")
    print(f"X range: [{positions[:,0].min():.3f}, {positions[:,0].max():.3f}]")
    print(f"Y range: [{positions[:,1].min():.3f}, {positions[:,1].max():.3f}]")
    print(f"Yaw range: [{orientations.min():.3f}, {orientations.max():.3f}] rad")

    # Calculate total distance and rotation
    dx = np.diff(positions[:,0])
    dy = np.diff(positions[:,1])
    distances = np.sqrt(dx**2 + dy**2)
    total_distance = np.sum(distances)

    dyaw = np.diff(orientations)
    # Handle angle wrapping
    dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))
    total_rotation = np.sum(np.abs(dyaw))

    print(f"Total distance: {total_distance:.3f} m")
    print(f"Total rotation: {total_rotation:.3f} rad ({np.degrees(total_rotation):.1f} deg)")

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # 1. XY trajectory
    ax1 = axes[0, 0]
    scatter = ax1.scatter(positions[:,0], positions[:,1], c=timestamps, cmap='viridis', s=5)
    ax1.plot(positions[0,0], positions[0,1], 'go', markersize=10, label='Start')
    ax1.plot(positions[-1,0], positions[-1,1], 'ro', markersize=10, label='End')

    # Draw orientation arrows every N points
    arrow_interval = max(1, len(positions) // 30)
    for i in range(0, len(positions), arrow_interval):
        x, y = positions[i]
        yaw = orientations[i]
        arrow_dx = 0.05 * np.cos(yaw)
        arrow_dy = 0.05 * np.sin(yaw)
        ax1.arrow(x, y, arrow_dx, arrow_dy, head_width=0.02, head_length=0.01, fc='red', ec='red', alpha=0.5)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Odometry XY Trajectory')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True)
    plt.colorbar(scatter, ax=ax1, label='Time (s)')

    # 2. X, Y vs time
    ax2 = axes[0, 1]
    ax2.plot(timestamps, positions[:,0], 'b-', label='X')
    ax2.plot(timestamps, positions[:,1], 'r-', label='Y')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('X, Y Position vs Time')
    ax2.legend()
    ax2.grid(True)

    # 3. Yaw vs time
    ax3 = axes[1, 0]
    ax3.plot(timestamps, orientations, 'g-')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Yaw (rad)')
    ax3.set_title('Yaw Orientation vs Time')
    ax3.grid(True)

    # Add degree scale on right
    ax3_deg = ax3.twinx()
    ax3_deg.set_ylim(np.degrees(ax3.get_ylim()[0]), np.degrees(ax3.get_ylim()[1]))
    ax3_deg.set_ylabel('Yaw (deg)')

    # 4. Velocity (distance per timestep)
    ax4 = axes[1, 1]
    dt = np.diff(timestamps)
    dt[dt == 0] = 0.001  # Avoid division by zero
    velocity = distances / dt
    angular_velocity = np.abs(dyaw) / dt

    ax4.plot(timestamps[1:], velocity, 'b-', label='Linear (m/s)', alpha=0.7)
    ax4.plot(timestamps[1:], angular_velocity, 'r-', label='Angular (rad/s)', alpha=0.7)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Velocity')
    ax4.set_title('Linear and Angular Velocity')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f"\nSaved trajectory plot to {args.output}")

    # Print statistics
    print("\n=== Movement Statistics ===")
    print(f"Start position: ({positions[0,0]:.4f}, {positions[0,1]:.4f})")
    print(f"End position: ({positions[-1,0]:.4f}, {positions[-1,1]:.4f})")
    print(f"Net displacement: {np.sqrt((positions[-1,0]-positions[0,0])**2 + (positions[-1,1]-positions[0,1])**2):.4f} m")
    print(f"Start yaw: {orientations[0]:.4f} rad ({np.degrees(orientations[0]):.1f} deg)")
    print(f"End yaw: {orientations[-1]:.4f} rad ({np.degrees(orientations[-1]):.1f} deg)")

    # Check for any significant movement
    print("\n=== Movement Analysis ===")
    significant_moves = np.sum(distances > 0.01)
    significant_rotations = np.sum(np.abs(dyaw) > 0.01)
    print(f"Movements > 1cm: {significant_moves} / {len(distances)}")
    print(f"Rotations > 0.01 rad: {significant_rotations} / {len(dyaw)}")

    # Print first 20 odom positions
    print("\n=== First 20 Odom Positions ===")
    for i in range(min(20, len(positions))):
        print(f"  [{i}] x={positions[i,0]:.6f}, y={positions[i,1]:.6f}, yaw={orientations[i]:.4f}")

    # Print cumulative distance every ~10%
    print("\n=== Cumulative Distance ===")
    cumulative = np.cumsum(distances)
    for pct in [10, 25, 50, 75, 90, 100]:
        idx = int(len(cumulative) * pct / 100) - 1
        if idx >= 0:
            print(f"  {pct}%: {cumulative[idx]:.3f} m at t={timestamps[idx+1]:.1f}s")


if __name__ == '__main__':
    main()
