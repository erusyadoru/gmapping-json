#!/usr/bin/env python3
"""Plot each scan step on the final map for verification - zoomed version."""

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


def find_mapped_region(map_img):
    """Find the bounding box of the mapped (non-gray) region."""
    # Gray (unknown) is typically 128
    non_gray = (map_img != 128)
    if not np.any(non_gray):
        return None

    rows = np.any(non_gray, axis=1)
    cols = np.any(non_gray, axis=0)
    ymin, ymax = np.where(rows)[0][[0, -1]]
    xmin, xmax = np.where(cols)[0][[0, -1]]

    # Add padding
    padding = 50
    ymin = max(0, ymin - padding)
    ymax = min(map_img.shape[0], ymax + padding)
    xmin = max(0, xmin - padding)
    xmax = min(map_img.shape[1], xmax + padding)

    return xmin, xmax, ymin, ymax


def draw_scan_on_map(ax, map_img, step, map_info, zoom_region=None):
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
    ax.scatter(scan_px, scan_py, c='red', s=3, alpha=0.8, label='Scan')

    # Plot robot position
    ax.scatter(robot_px, robot_py, c='blue', s=100, marker='o', zorder=10, label='Robot')

    # Draw robot orientation arrow
    arrow_len = 30  # pixels
    ax.arrow(robot_px, robot_py,
             arrow_len * np.cos(-robot_theta),  # Negative because image Y is flipped
             arrow_len * np.sin(-robot_theta),
             head_width=8, head_length=5, fc='blue', ec='blue', zorder=10)

    # Also show odometry position
    odom = step['odom']
    odom_px, odom_py = world_to_pixel(odom['x'], odom['y'], map_info)
    ax.scatter(odom_px, odom_py, c='lime', s=100, marker='x', zorder=10, label='Odom', linewidths=3)

    # Draw line between estimated and odom position
    ax.plot([robot_px, odom_px], [robot_py, odom_py], 'g--', linewidth=2, alpha=0.7)

    # Set zoom region
    if zoom_region:
        xmin, xmax, ymin, ymax = zoom_region
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymax, ymin)  # Reversed because image Y is flipped

    # Calculate distance between estimated pose and odom
    dist = np.sqrt((robot_x - odom['x'])**2 + (robot_y - odom['y'])**2)

    ax.set_title(f"Step {step['step']}: pose=({robot_x:.2f}, {robot_y:.2f})\n"
                 f"odom=({odom['x']:.2f}, {odom['y']:.2f}), diff={dist:.3f}m")


def main():
    if len(sys.argv) < 2:
        print("Usage: plot_scan_on_map_zoomed.py <scan_log.json> [map.pgm] [output_dir]")
        print("\nThis will generate one zoomed image per scan step.")
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
        output_dir = '/tmp/scan_steps_zoomed'

    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading scan log from {json_path}...")
    data = load_scan_log(json_path)

    print(f"Loading map from {map_path}...")
    map_img = load_map(map_path)

    steps = data['steps']
    map_info = data['map']

    print(f"Found {len(steps)} steps, map size {map_info['width']}x{map_info['height']}")
    print(f"Map origin: ({map_info['origin_x']}, {map_info['origin_y']}), resolution: {map_info['resolution']}")

    # Find mapped region for zooming
    zoom_region = find_mapped_region(map_img)
    if zoom_region:
        xmin, xmax, ymin, ymax = zoom_region
        print(f"Mapped region: x=[{xmin}, {xmax}], y=[{ymin}, {ymax}]")
    else:
        print("Could not find mapped region, using full map")

    print(f"Saving zoomed images to {output_dir}/")

    # Generate image for each step
    for i, step in enumerate(steps):
        fig, ax = plt.subplots(figsize=(12, 10))
        draw_scan_on_map(ax, map_img, step, map_info, zoom_region)
        ax.legend(loc='upper right')

        output_path = os.path.join(output_dir, f"step_{step['step']:04d}.png")
        plt.savefig(output_path, dpi=120, bbox_inches='tight')
        plt.close(fig)

        if (i + 1) % 10 == 0 or i == 0:
            print(f"  Generated {i + 1}/{len(steps)} images...")

    print(f"\nDone! Generated {len(steps)} images in {output_dir}/")

    # Print summary statistics
    print("\n=== Pose vs Odom Difference Summary ===")
    diffs = []
    for step in steps:
        pose = step['pose']
        odom = step['odom']
        diff = np.sqrt((pose['x'] - odom['x'])**2 + (pose['y'] - odom['y'])**2)
        diffs.append(diff)
        print(f"Step {step['step']:3d}: diff = {diff:.3f} m")

    print(f"\nMean difference: {np.mean(diffs):.3f} m")
    print(f"Max difference: {np.max(diffs):.3f} m")
    print(f"Min difference: {np.min(diffs):.3f} m")


if __name__ == '__main__':
    main()
