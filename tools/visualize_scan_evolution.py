#!/usr/bin/env python3
"""Visualize scan evolution over time to identify alignment issues."""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def transform_scan_to_world(ranges, angle_min, angle_increment, robot_x, robot_y, robot_yaw, range_max):
    points = []
    for i, r in enumerate(ranges):
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

    # Read scan data with timestamps
    print("Reading scan data...")
    scans_with_time = []  # (relative_time, points, robot_pose)

    start_time = None
    with Reader(bag_file) as reader:
        connections = [c for c in reader.connections if c.topic == '/scan_merged']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            scan_time = timestamp / 1e9

            if start_time is None:
                start_time = scan_time

            closest_odom = min(odom_data, key=lambda o: abs(o[0] - scan_time))
            _, robot_x, robot_y, robot_yaw = closest_odom

            points = transform_scan_to_world(
                msg.ranges, msg.angle_min, msg.angle_increment,
                robot_x, robot_y, robot_yaw, msg.range_max
            )

            rel_time = scan_time - start_time
            scans_with_time.append((rel_time, points, (robot_x, robot_y, robot_yaw)))

    print(f"Loaded {len(scans_with_time)} scans")

    # Create time-colored visualization
    fig, axes = plt.subplots(2, 2, figsize=(16, 16))

    # View bounds
    view_size = 5.0
    odom_array = np.array([(o[1], o[2]) for o in odom_data])
    center_x = odom_array[:, 0].mean()
    center_y = odom_array[:, 1].mean()
    x_min, x_max = center_x - view_size, center_x + view_size
    y_min, y_max = center_y - view_size, center_y + view_size

    # 1. All scans colored by time
    ax1 = axes[0, 0]
    max_time = scans_with_time[-1][0]
    cmap = plt.cm.viridis

    for rel_time, points, pose in scans_with_time:
        if points:
            pts = np.array(points)
            color = cmap(rel_time / max_time)
            ax1.scatter(pts[:, 0], pts[:, 1], c=[color], s=0.1, alpha=0.3)

    ax1.plot(odom_array[:, 0], odom_array[:, 1], 'r-', linewidth=2, alpha=0.8, label='Odom')
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_title('All Scans (colored by time: purple=early, yellow=late)')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    # 2. First 20 scans only
    ax2 = axes[0, 1]
    for i, (rel_time, points, pose) in enumerate(scans_with_time[:20]):
        if points:
            pts = np.array(points)
            ax2.scatter(pts[:, 0], pts[:, 1], s=0.5, alpha=0.5, label=f't={rel_time:.1f}s' if i < 5 else None)

    ax2.plot(odom_array[:50, 0], odom_array[:50, 1], 'r-', linewidth=2)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.set_title('First 20 Scans')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8)

    # 3. Time slices - first quarter vs last quarter
    ax3 = axes[1, 0]
    n_scans = len(scans_with_time)
    quarter = n_scans // 4

    # First quarter (blue)
    for rel_time, points, pose in scans_with_time[:quarter]:
        if points:
            pts = np.array(points)
            ax3.scatter(pts[:, 0], pts[:, 1], c='blue', s=0.1, alpha=0.3)

    # Last quarter (red)
    for rel_time, points, pose in scans_with_time[-quarter:]:
        if points:
            pts = np.array(points)
            ax3.scatter(pts[:, 0], pts[:, 1], c='red', s=0.1, alpha=0.3)

    ax3.set_xlim(x_min, x_max)
    ax3.set_ylim(y_min, y_max)
    ax3.set_title('First quarter (blue) vs Last quarter (red)')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.axis('equal')
    ax3.grid(True, alpha=0.3)

    # 4. Single scan comparison - show individual scans at different times
    ax4 = axes[1, 1]
    indices_to_show = [0, n_scans//4, n_scans//2, 3*n_scans//4, n_scans-1]
    colors = ['blue', 'green', 'orange', 'red', 'purple']

    for idx, color in zip(indices_to_show, colors):
        rel_time, points, (rx, ry, ryaw) = scans_with_time[idx]
        if points:
            pts = np.array(points)
            ax4.scatter(pts[:, 0], pts[:, 1], c=color, s=1, alpha=0.7,
                       label=f't={rel_time:.1f}s, yaw={math.degrees(ryaw):.0f}°')
            # Mark robot position
            ax4.plot(rx, ry, 'o', color=color, markersize=8)
            # Draw heading arrow
            ax4.arrow(rx, ry, 0.2*math.cos(ryaw), 0.2*math.sin(ryaw),
                     head_width=0.05, color=color)

    ax4.set_xlim(x_min, x_max)
    ax4.set_ylim(y_min, y_max)
    ax4.set_title('Individual Scans at Different Times')
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.axis('equal')
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='upper right', fontsize=8)

    plt.tight_layout()
    plt.savefig('scan_evolution.png', dpi=150)
    print("\nSaved to scan_evolution.png")

    # Print odom pose at those times
    print("\n=== Robot poses at selected scans ===")
    for idx in indices_to_show:
        rel_time, points, (rx, ry, ryaw) = scans_with_time[idx]
        print(f"t={rel_time:.1f}s: pos=({rx:.3f}, {ry:.3f}), yaw={math.degrees(ryaw):.1f}°")

if __name__ == '__main__':
    main()
