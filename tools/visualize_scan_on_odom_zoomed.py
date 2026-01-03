#!/usr/bin/env python3
"""Visualize scan data overlaid on odom trajectory - zoomed view."""

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
        # Filter out max range hits and invalid readings
        if r < 0.1 or r > range_max * 0.95 or math.isnan(r) or math.isinf(r):
            continue
        angle = angle_min + i * angle_increment
        px = r * math.cos(angle)
        py = r * math.sin(angle)
        wx = robot_x + px * math.cos(robot_yaw) - py * math.sin(robot_yaw)
        wy = robot_y + px * math.sin(robot_yaw) + py * math.cos(robot_yaw)
        points.append((wx, wy))
    return points

def main():
    bag_file = '/home/industryalpha/gmapping-json/2025-11-18-08-13-15_merged.bag'
    typestore = get_typestore(Stores.ROS1_NOETIC)

    # Read odom data
    print("Reading odom data...")
    odom_data = []
    with Reader(bag_file) as reader:
        connections = [c for c in reader.connections if c.topic == '/odom']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            odom_data.append((timestamp / 1e9, x, y, yaw))

    print(f"Loaded {len(odom_data)} odom messages")

    # Read scan data
    print("Reading scan data...")
    all_scan_points = []
    scan_count = 0

    with Reader(bag_file) as reader:
        connections = [c for c in reader.connections if c.topic == '/scan_merged']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            scan_time = timestamp / 1e9

            closest_odom = min(odom_data, key=lambda o: abs(o[0] - scan_time))
            _, robot_x, robot_y, robot_yaw = closest_odom

            points = transform_scan_to_world(
                msg.ranges, msg.angle_min, msg.angle_increment,
                robot_x, robot_y, robot_yaw, msg.range_max
            )
            all_scan_points.extend(points)
            scan_count += 1

    print(f"Total scan points (filtered): {len(all_scan_points)}")

    # Convert to arrays
    odom_array = np.array([(o[1], o[2]) for o in odom_data])
    scan_array = np.array(all_scan_points)

    # Calculate bounds based on odom trajectory + margin
    odom_center_x = (odom_array[:, 0].min() + odom_array[:, 0].max()) / 2
    odom_center_y = (odom_array[:, 1].min() + odom_array[:, 1].max()) / 2

    # Use a reasonable view window (e.g., 10m x 10m centered on robot)
    view_size = 5.0  # 5m radius
    x_min, x_max = odom_center_x - view_size, odom_center_x + view_size
    y_min, y_max = odom_center_y - view_size, odom_center_y + view_size

    print(f"View bounds: x=[{x_min:.2f}, {x_max:.2f}], y=[{y_min:.2f}, {y_max:.2f}]")

    # Filter scan points within view
    mask = (scan_array[:, 0] >= x_min) & (scan_array[:, 0] <= x_max) & \
           (scan_array[:, 1] >= y_min) & (scan_array[:, 1] <= y_max)
    scan_in_view = scan_array[mask]
    print(f"Scan points in view: {len(scan_in_view)}")

    # Create grid at 0.01m resolution
    resolution = 0.01
    grid_width = int((x_max - x_min) / resolution) + 1
    grid_height = int((y_max - y_min) / resolution) + 1
    print(f"Grid size: {grid_width} x {grid_height}")

    grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

    for px, py in scan_in_view:
        gx = int((px - x_min) / resolution)
        gy = int((py - y_min) / resolution)
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            grid[gy, gx] = 255

    # Create figure
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # Left: Scatter plot
    ax1 = axes[0]
    ax1.scatter(scan_in_view[:, 0], scan_in_view[:, 1], c='black', s=0.5, alpha=0.5)
    ax1.plot(odom_array[:, 0], odom_array[:, 1], 'b-', linewidth=2, alpha=0.8, label='Odom')
    ax1.plot(odom_array[0, 0], odom_array[0, 1], 'go', markersize=12, label='Start', zorder=5)
    ax1.plot(odom_array[-1, 0], odom_array[-1, 1], 'ro', markersize=12, label='End', zorder=5)

    # Draw orientation arrows
    arrow_interval = max(1, len(odom_data) // 20)
    for i in range(0, len(odom_data), arrow_interval):
        x, y = odom_array[i]
        yaw = odom_data[i][3]
        dx = 0.1 * np.cos(yaw)
        dy = 0.1 * np.sin(yaw)
        ax1.arrow(x, y, dx, dy, head_width=0.03, head_length=0.02, fc='red', ec='red', alpha=0.7)

    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Scan Points + Odom Trajectory (Zoomed)')
    ax1.legend(loc='upper right')
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    # Right: Grid map
    ax2 = axes[1]
    grid_display = np.flipud(grid)
    img = np.ones((grid_height, grid_width, 3), dtype=np.uint8) * 255  # White background
    img[grid_display == 255] = [0, 0, 0]  # Black obstacles

    # Draw trajectory on grid
    for i in range(len(odom_array)):
        gx = int((odom_array[i, 0] - x_min) / resolution)
        gy = grid_height - 1 - int((odom_array[i, 1] - y_min) / resolution)
        if 0 <= gx < grid_width and 0 <= gy < grid_height:
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if 0 <= gy+dy < grid_height and 0 <= gx+dx < grid_width:
                        img[gy+dy, gx+dx] = [0, 0, 255]  # Blue trajectory

    ax2.imshow(img, extent=[x_min, x_max, y_min, y_max], aspect='equal', origin='lower')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title(f'Grid Map (resolution={resolution}m, {grid_width}x{grid_height})')

    plt.tight_layout()
    plt.savefig('scan_on_odom_zoomed.png', dpi=150)
    print(f"\nSaved to scan_on_odom_zoomed.png")

    # Save as PGM
    with open('scan_overlay_zoomed.pgm', 'wb') as f:
        f.write(f"P5\n{grid_width} {grid_height}\n255\n".encode())
        pgm_grid = np.flipud(grid)
        pgm_out = np.where(pgm_grid == 255, 0, 255).astype(np.uint8)
        f.write(pgm_out.tobytes())

    # Save YAML
    with open('scan_overlay_zoomed.yaml', 'w') as f:
        f.write(f"image: scan_overlay_zoomed.pgm\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{x_min}, {y_min}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print(f"Saved grid to scan_overlay_zoomed.pgm")

if __name__ == '__main__':
    main()
