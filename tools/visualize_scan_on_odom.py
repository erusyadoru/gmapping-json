#!/usr/bin/env python3
"""Visualize scan data overlaid on odom trajectory."""

import math
import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def transform_scan_to_world(ranges, angle_min, angle_increment, robot_x, robot_y, robot_yaw, range_max):
    """Transform scan points from robot frame to world frame."""
    points = []
    for i, r in enumerate(ranges):
        if r < 0.1 or r > range_max or math.isnan(r) or math.isinf(r):
            continue
        # Angle in robot frame
        angle = angle_min + i * angle_increment
        # Point in robot frame
        px = r * math.cos(angle)
        py = r * math.sin(angle)
        # Transform to world frame
        wx = robot_x + px * math.cos(robot_yaw) - py * math.sin(robot_yaw)
        wy = robot_y + px * math.sin(robot_yaw) + py * math.cos(robot_yaw)
        points.append((wx, wy))
    return points

def main():
    bag_file = '/home/industryalpha/gmapping-json/2025-11-18-08-13-15_merged.bag'
    typestore = get_typestore(Stores.ROS1_NOETIC)

    # Read odom data
    print("Reading odom data...")
    odom_data = []  # (timestamp, x, y, yaw)
    with Reader(bag_file) as reader:
        connections = [c for c in reader.connections if c.topic == '/odom']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            odom_data.append((timestamp / 1e9, x, y, yaw))

    print(f"Loaded {len(odom_data)} odom messages")

    # Read scan data and find closest odom for each
    print("Reading scan data...")
    all_scan_points = []
    scan_count = 0

    with Reader(bag_file) as reader:
        connections = [c for c in reader.connections if c.topic == '/scan_merged']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            scan_time = timestamp / 1e9

            # Find closest odom
            closest_odom = min(odom_data, key=lambda o: abs(o[0] - scan_time))
            _, robot_x, robot_y, robot_yaw = closest_odom

            # Transform scan points
            points = transform_scan_to_world(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                robot_x, robot_y, robot_yaw,
                msg.range_max
            )
            all_scan_points.extend(points)
            scan_count += 1

            if scan_count % 50 == 0:
                print(f"  Processed {scan_count} scans...")

    print(f"Total scan points: {len(all_scan_points)}")

    # Create occupancy grid at 0.01m resolution
    resolution = 0.01

    # Determine bounds
    odom_array = np.array([(o[1], o[2]) for o in odom_data])
    scan_array = np.array(all_scan_points) if all_scan_points else np.array([[0, 0]])

    all_points = np.vstack([odom_array, scan_array])
    x_min, y_min = all_points.min(axis=0) - 1.0
    x_max, y_max = all_points.max(axis=0) + 1.0

    print(f"Bounds: x=[{x_min:.2f}, {x_max:.2f}], y=[{y_min:.2f}, {y_max:.2f}]")

    # Create grid
    grid_width = int((x_max - x_min) / resolution) + 1
    grid_height = int((y_max - y_min) / resolution) + 1
    print(f"Grid size: {grid_width} x {grid_height}")

    # Initialize grid (0 = unknown)
    grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Mark scan points as obstacles (black = 0 in image, but we'll use 255 for obstacles in our representation)
    print("Populating grid with scan points...")
    for px, py in all_scan_points:
        gx = int((px - x_min) / resolution)
        gy = int((py - y_min) / resolution)
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            grid[gy, gx] = 255  # Obstacle

    # Create figure
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # Left: Raw point cloud view
    ax1 = axes[0]
    if all_scan_points:
        scan_array = np.array(all_scan_points)
        ax1.scatter(scan_array[:, 0], scan_array[:, 1], c='black', s=0.1, alpha=0.5, label='Scan points')
    ax1.plot(odom_array[:, 0], odom_array[:, 1], 'b-', linewidth=1, alpha=0.7, label='Odom trajectory')
    ax1.plot(odom_array[0, 0], odom_array[0, 1], 'go', markersize=10, label='Start')
    ax1.plot(odom_array[-1, 0], odom_array[-1, 1], 'ro', markersize=10, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Scan Points + Odom Trajectory')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    # Right: Grid map view (like gmapping output)
    ax2 = axes[1]
    # Flip grid for proper orientation (y increases upward)
    grid_display = np.flipud(grid)
    # Create image: white background, black obstacles
    img = np.ones((grid_height, grid_width, 3), dtype=np.uint8) * 200  # Gray background
    img[grid_display == 255] = [0, 0, 0]  # Black obstacles

    # Draw trajectory on grid
    for i in range(len(odom_array)):
        gx = int((odom_array[i, 0] - x_min) / resolution)
        gy = grid_height - 1 - int((odom_array[i, 1] - y_min) / resolution)
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            img[max(0,gy-1):min(grid_height,gy+2), max(0,gx-1):min(grid_width,gx+2)] = [0, 0, 255]  # Blue trajectory

    extent = [x_min, x_max, y_min, y_max]
    ax2.imshow(img, extent=extent, aspect='equal', origin='lower')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title(f'Grid Map (resolution={resolution}m)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('scan_on_odom.png', dpi=150)
    print(f"\nSaved to scan_on_odom.png")

    # Also save as PGM for comparison with gmapping output
    print("Saving grid as PGM...")
    with open('scan_overlay.pgm', 'wb') as f:
        f.write(f"P5\n{grid_width} {grid_height}\n255\n".encode())
        # Flip and invert for standard map format (white=free, black=occupied, gray=unknown)
        pgm_grid = np.flipud(grid)
        pgm_out = np.where(pgm_grid == 255, 0, 200).astype(np.uint8)  # obstacles=black, rest=gray
        f.write(pgm_out.tobytes())

    print(f"Saved grid to scan_overlay.pgm ({grid_width}x{grid_height})")

if __name__ == '__main__':
    main()
