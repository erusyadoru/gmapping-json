#!/usr/bin/env python3
"""Plot each scan step on the final map for verification."""

import json
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def load_scan_log(json_path):
    """Load scan log from JSON file."""
    with open(json_path, 'r') as f:
        return json.load(f)


def load_map(pgm_path):
    """Load map from PGM file."""
    img = Image.open(pgm_path)
    return np.array(img)


def world_to_pixel(x, y, map_info):
    """Convert world coordinates to pixel coordinates."""
    px = (x - map_info['origin_x']) / map_info['resolution']
    # Flip Y for image coordinates (image origin is top-left)
    py = map_info['height'] - (y - map_info['origin_y']) / map_info['resolution']
    return px, py


def draw_scan_on_map(ax, map_img, step, map_info):
    """Draw a single scan on the map."""
    # Draw map
    ax.imshow(map_img, cmap='gray', origin='upper')

    # Get pose and scan data
    pose = step['pose']
    robot_x, robot_y, robot_theta = pose['x'], pose['y'], pose['theta']

    angle_min = step['angle_min']
    angle_max = step['angle_max']
    ranges = step['ranges']
    num_beams = len(ranges)

    if num_beams == 0:
        return

    angle_increment = (angle_max - angle_min) / (num_beams - 1) if num_beams > 1 else 0

    # Convert robot pose to pixel coordinates
    robot_px, robot_py = world_to_pixel(robot_x, robot_y, map_info)

    # Draw scan points
    scan_px = []
    scan_py = []

    for i, r in enumerate(ranges):
        if r <= 0.01 or r > 30.0 or np.isnan(r) or np.isinf(r):
            continue

        angle = angle_min + i * angle_increment
        # Transform to world coordinates
        world_angle = robot_theta + angle
        hit_x = robot_x + r * np.cos(world_angle)
        hit_y = robot_y + r * np.sin(world_angle)

        px, py = world_to_pixel(hit_x, hit_y, map_info)
        scan_px.append(px)
        scan_py.append(py)

    # Plot scan points
    ax.scatter(scan_px, scan_py, c='red', s=1, alpha=0.8, label='Scan')

    # Plot robot position
    ax.scatter(robot_px, robot_py, c='blue', s=50, marker='o', zorder=10, label='Robot')

    # Draw robot orientation arrow
    arrow_len = 20  # pixels
    ax.arrow(robot_px, robot_py,
             arrow_len * np.cos(-robot_theta),  # Negative because image Y is flipped
             arrow_len * np.sin(-robot_theta),
             head_width=5, head_length=3, fc='blue', ec='blue', zorder=10)

    # Also show odometry position
    odom = step['odom']
    odom_px, odom_py = world_to_pixel(odom['x'], odom['y'], map_info)
    ax.scatter(odom_px, odom_py, c='green', s=50, marker='x', zorder=10, label='Odom')

    ax.set_title(f"Step {step['step']}: pose=({robot_x:.3f}, {robot_y:.3f}, {robot_theta:.3f})")


def main():
    if len(sys.argv) < 2:
        print("Usage: plot_scan_on_map.py <scan_log.json> [map.pgm] [output_dir]")
        print("\nThis will generate one image per scan step showing the scan on the final map.")
        sys.exit(1)

    json_path = sys.argv[1]

    # Derive map path from json path if not specified
    if len(sys.argv) >= 3:
        map_path = sys.argv[2]
    else:
        map_path = os.path.join(os.path.dirname(json_path), 'map.pgm')
        if not os.path.exists(map_path):
            map_path = 'map.pgm'

    # Output directory
    if len(sys.argv) >= 4:
        output_dir = sys.argv[3]
    else:
        output_dir = '/tmp/scan_steps'

    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading scan log from {json_path}...")
    data = load_scan_log(json_path)

    print(f"Loading map from {map_path}...")
    map_img = load_map(map_path)

    steps = data['steps']
    map_info = data['map']

    print(f"Found {len(steps)} steps, map size {map_info['width']}x{map_info['height']}")
    print(f"Map origin: ({map_info['origin_x']}, {map_info['origin_y']}), resolution: {map_info['resolution']}")
    print(f"Saving images to {output_dir}/")

    # Generate image for each step
    for i, step in enumerate(steps):
        fig, ax = plt.subplots(figsize=(12, 12))
        draw_scan_on_map(ax, map_img, step, map_info)
        ax.legend(loc='upper right')
        ax.axis('equal')

        output_path = os.path.join(output_dir, f"step_{step['step']:04d}.png")
        plt.savefig(output_path, dpi=100, bbox_inches='tight')
        plt.close(fig)

        if (i + 1) % 10 == 0 or i == 0:
            print(f"  Generated {i + 1}/{len(steps)} images...")

    print(f"\nDone! Generated {len(steps)} images in {output_dir}/")

    # Also create a combined trajectory plot
    print("\nGenerating combined trajectory plot...")
    fig, ax = plt.subplots(figsize=(14, 14))
    ax.imshow(map_img, cmap='gray', origin='upper')

    # Draw trajectory
    poses_px = []
    poses_py = []
    odoms_px = []
    odoms_py = []

    for step in steps:
        pose = step['pose']
        px, py = world_to_pixel(pose['x'], pose['y'], map_info)
        poses_px.append(px)
        poses_py.append(py)

        odom = step['odom']
        ox, oy = world_to_pixel(odom['x'], odom['y'], map_info)
        odoms_px.append(ox)
        odoms_py.append(oy)

    ax.plot(poses_px, poses_py, 'b-', linewidth=1, label='Estimated trajectory')
    ax.plot(odoms_px, odoms_py, 'g--', linewidth=1, alpha=0.5, label='Odometry trajectory')
    ax.scatter(poses_px[0], poses_py[0], c='green', s=100, marker='o', zorder=10, label='Start')
    ax.scatter(poses_px[-1], poses_py[-1], c='red', s=100, marker='x', zorder=10, label='End')

    ax.legend(loc='upper right')
    ax.set_title(f'Trajectory ({len(steps)} steps)')
    ax.axis('equal')

    trajectory_path = os.path.join(output_dir, 'trajectory.png')
    plt.savefig(trajectory_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved trajectory plot to {trajectory_path}")


if __name__ == '__main__':
    main()
