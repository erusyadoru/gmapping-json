#!/usr/bin/env python3
"""Visualize the mapping process step by step, saving frames."""

import math
import numpy as np
import matplotlib.pyplot as plt
import os
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
    output_dir = '/home/industryalpha/gmapping-json/mapping_frames'
    os.makedirs(output_dir, exist_ok=True)

    typestore = get_typestore(Stores.ROS1_NOETIC)

    # Parameters
    resolution = 0.01  # 1cm per pixel
    view_size = 5.0    # 5m radius view
    x_min, x_max = -view_size, view_size
    y_min, y_max = -view_size, view_size
    grid_width = int((x_max - x_min) / resolution) + 1
    grid_height = int((y_max - y_min) / resolution) + 1

    print(f"Grid size: {grid_width} x {grid_height}")
    print(f"Output directory: {output_dir}")

    # Initialize cumulative grid
    cumulative_grid = np.zeros((grid_height, grid_width), dtype=np.float32)

    # Read odom data first
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

    # Trajectory for plotting
    trajectory = []

    # Process scans
    print("Processing scans...")
    scan_count = 0
    start_time = None

    with Reader(bag_file) as reader:
        connections = [c for c in reader.connections if c.topic == '/scan_merged']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            scan_time = timestamp / 1e9

            if start_time is None:
                start_time = scan_time

            rel_time = scan_time - start_time

            # Find closest odom
            closest_odom = min(odom_data, key=lambda o: abs(o[0] - scan_time))
            _, robot_x, robot_y, robot_yaw = closest_odom

            # Add to trajectory
            trajectory.append((robot_x, robot_y, robot_yaw))

            # Transform scan points
            points = transform_scan_to_world(
                msg.ranges, msg.angle_min, msg.angle_increment,
                robot_x, robot_y, robot_yaw, msg.range_max
            )

            # Update cumulative grid
            for px, py in points:
                gx = int((px - x_min) / resolution)
                gy = int((py - y_min) / resolution)
                if 0 <= gx < grid_width and 0 <= gy < grid_height:
                    cumulative_grid[gy, gx] = min(cumulative_grid[gy, gx] + 0.5, 10.0)

            # Save frame every 5 scans
            if scan_count % 5 == 0:
                fig, ax = plt.subplots(1, 1, figsize=(10, 10))

                # Draw cumulative map (obstacles as black on white background)
                # origin='lower' means row 0 is at bottom (y_min)
                img = np.ones((grid_height, grid_width, 3), dtype=np.float32)
                normalized = np.clip(cumulative_grid / 5.0, 0, 1)
                for c in range(3):
                    img[:, :, c] = 1.0 - normalized

                ax.imshow(img, extent=[x_min, x_max, y_min, y_max], origin='lower')

                # Draw current scan points in red
                if points:
                    pts = np.array(points)
                    ax.scatter(pts[:, 0], pts[:, 1], c='red', s=1, alpha=0.8, label='Current scan')

                # Draw trajectory
                if len(trajectory) > 1:
                    traj = np.array([(t[0], t[1]) for t in trajectory])
                    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1.5, alpha=0.7, label='Trajectory')

                # Draw robot position and heading
                ax.plot(robot_x, robot_y, 'go', markersize=12, label='Robot')
                arrow_len = 0.3
                ax.arrow(robot_x, robot_y,
                        arrow_len * math.cos(robot_yaw),
                        arrow_len * math.sin(robot_yaw),
                        head_width=0.1, head_length=0.05, fc='green', ec='green')

                ax.set_xlim(x_min, x_max)
                ax.set_ylim(y_min, y_max)
                ax.set_xlabel('X (m)')
                ax.set_ylabel('Y (m)')
                ax.set_title(f'Mapping Process - Scan {scan_count}, t={rel_time:.1f}s\n'
                            f'Robot: ({robot_x:.2f}, {robot_y:.2f}), yaw={math.degrees(robot_yaw):.1f}°')
                ax.legend(loc='upper right')
                ax.set_aspect('equal')
                ax.grid(True, alpha=0.3)

                frame_file = os.path.join(output_dir, f'frame_{scan_count:04d}.png')
                plt.savefig(frame_file, dpi=100, bbox_inches='tight')
                plt.close()

                print(f"  Saved frame {scan_count}: t={rel_time:.1f}s, pos=({robot_x:.3f}, {robot_y:.3f}), yaw={math.degrees(robot_yaw):.1f}°")

            scan_count += 1

    print(f"\nProcessed {scan_count} scans")
    print(f"Frames saved to {output_dir}")

    # Create final summary image
    print("\nCreating final summary...")
    fig, ax = plt.subplots(1, 1, figsize=(12, 12))

    img = np.ones((grid_height, grid_width, 3), dtype=np.float32)
    normalized = np.clip(cumulative_grid / 5.0, 0, 1)
    for c in range(3):
        img[:, :, c] = 1.0 - normalized

    ax.imshow(img, extent=[x_min, x_max, y_min, y_max], origin='lower')

    # Full trajectory
    traj = np.array([(t[0], t[1]) for t in trajectory])
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, alpha=0.8, label='Trajectory')
    ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=15, label='Start')
    ax.plot(traj[-1, 0], traj[-1, 1], 'ro', markersize=15, label='End')

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Final Map - {scan_count} scans processed')
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.savefig(os.path.join(output_dir, 'final_map.png'), dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Final map saved to {output_dir}/final_map.png")

if __name__ == '__main__':
    main()
