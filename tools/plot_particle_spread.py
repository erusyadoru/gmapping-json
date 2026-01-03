#!/usr/bin/env python3
"""Plot particle spread over time from GMapping log."""

import re
import sys
import matplotlib.pyplot as plt
import numpy as np

def parse_log(filename):
    """Parse particle spread data from log file."""
    data = {
        'mean_x': [], 'mean_y': [],
        'std_x': [], 'std_y': [],
        'range_x': [], 'range_y': []
    }

    pattern = r'\[ParticleSpread\] mean=\(([^,]+),([^)]+)\) std=\(([^,]+),([^)]+)\) range_x=([^ ]+) range_y=([^ ]+)'

    with open(filename, 'r') as f:
        for line in f:
            match = re.search(pattern, line)
            if match:
                data['mean_x'].append(float(match.group(1)))
                data['mean_y'].append(float(match.group(2)))
                data['std_x'].append(float(match.group(3)))
                data['std_y'].append(float(match.group(4)))
                data['range_x'].append(float(match.group(5)))
                data['range_y'].append(float(match.group(6)))

    return data

def main():
    if len(sys.argv) < 2:
        log_file = '/tmp/server_log.txt'
    else:
        log_file = sys.argv[1]

    print(f"Parsing {log_file}...")
    data = parse_log(log_file)

    if not data['mean_x']:
        print("No particle spread data found!")
        return

    print(f"Found {len(data['mean_x'])} data points")

    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    steps = range(len(data['mean_x']))

    # Plot 1: Mean position (trajectory)
    ax1 = axes[0]
    ax1.plot(data['mean_x'], data['mean_y'], 'b-', linewidth=1, label='Mean trajectory')
    ax1.scatter(data['mean_x'][0], data['mean_y'][0], c='g', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(data['mean_x'][-1], data['mean_y'][-1], c='r', s=100, marker='x', label='End', zorder=5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Mean Particle Position (Trajectory)')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # Plot 2: Standard deviation over time
    ax2 = axes[1]
    ax2.plot(steps, data['std_x'], 'r-', label='Std X', linewidth=1.5)
    ax2.plot(steps, data['std_y'], 'b-', label='Std Y', linewidth=1.5)
    ax2.set_xlabel('Step')
    ax2.set_ylabel('Standard Deviation (m)')
    ax2.set_title('Particle Spread (Standard Deviation) Over Time')
    ax2.legend()
    ax2.grid(True)

    # Plot 3: Range (max - min) over time
    ax3 = axes[2]
    ax3.plot(steps, data['range_x'], 'r-', label='Range X', linewidth=1.5)
    ax3.plot(steps, data['range_y'], 'b-', label='Range Y', linewidth=1.5)
    ax3.set_xlabel('Step')
    ax3.set_ylabel('Range (m)')
    ax3.set_title('Particle Spread (Range: max - min) Over Time')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()

    # Save figure
    output_file = '/tmp/particle_spread.png'
    plt.savefig(output_file, dpi=150)
    print(f"Saved plot to {output_file}")

    # Print statistics
    print("\n=== Statistics ===")
    print(f"Mean Std X: {np.mean(data['std_x']):.4f} m")
    print(f"Mean Std Y: {np.mean(data['std_y']):.4f} m")
    print(f"Max Std X: {np.max(data['std_x']):.4f} m")
    print(f"Max Std Y: {np.max(data['std_y']):.4f} m")
    print(f"Mean Range X: {np.mean(data['range_x']):.4f} m")
    print(f"Mean Range Y: {np.mean(data['range_y']):.4f} m")
    print(f"Max Range X: {np.max(data['range_x']):.4f} m")
    print(f"Max Range Y: {np.max(data['range_y']):.4f} m")

if __name__ == '__main__':
    main()
