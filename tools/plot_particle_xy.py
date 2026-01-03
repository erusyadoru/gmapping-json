#!/usr/bin/env python3
"""Plot particle positions on X-Y plane for each step."""

import re
import sys
import matplotlib.pyplot as plt
import numpy as np

def parse_particles(filename):
    """Parse all particle positions from log file."""
    steps = []

    pattern = r'P(\d+)\(([^,]+),([^,]+),([^)]+)\)'
    best_pattern = r'\*P(\d+)'

    with open(filename, 'r') as f:
        for line in f:
            if '[Particles]' in line:
                particles = []
                best_idx = -1

                # Find best particle
                best_match = re.search(best_pattern, line)
                if best_match:
                    best_idx = int(best_match.group(1))

                # Find all particles
                for match in re.finditer(pattern, line):
                    idx = int(match.group(1))
                    x = float(match.group(2))
                    y = float(match.group(3))
                    theta = float(match.group(4))
                    particles.append({'idx': idx, 'x': x, 'y': y, 'theta': theta, 'best': idx == best_idx})

                if particles:
                    steps.append(particles)

    return steps

def main():
    if len(sys.argv) < 2:
        log_file = '/tmp/server_log.txt'
    else:
        log_file = sys.argv[1]

    print(f"Parsing {log_file}...")
    steps = parse_particles(log_file)

    if not steps:
        print("No particle data found!")
        return

    print(f"Found {len(steps)} steps")

    # Select steps to plot (evenly spaced)
    num_plots = min(12, len(steps))
    indices = np.linspace(0, len(steps)-1, num_plots, dtype=int)

    # Create figure with subplots
    cols = 4
    rows = (num_plots + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(16, 4*rows))
    axes = axes.flatten()

    for i, step_idx in enumerate(indices):
        ax = axes[i]
        particles = steps[step_idx]

        xs = [p['x'] for p in particles]
        ys = [p['y'] for p in particles]
        best_x = [p['x'] for p in particles if p['best']]
        best_y = [p['y'] for p in particles if p['best']]

        # Plot all particles
        ax.scatter(xs, ys, c='orange', s=30, alpha=0.7, label='Particles')

        # Highlight best particle
        if best_x:
            ax.scatter(best_x, best_y, c='cyan', s=100, marker='*', label='Best', zorder=5)

        # Calculate spread
        std_x = np.std(xs)
        std_y = np.std(ys)
        range_x = max(xs) - min(xs)
        range_y = max(ys) - min(ys)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Step {step_idx}\nstd=({std_x:.3f}, {std_y:.3f})\nrange=({range_x:.3f}, {range_y:.3f})')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        # Set axis limits based on data spread + margin
        margin = max(0.05, max(range_x, range_y) * 0.3)
        mean_x, mean_y = np.mean(xs), np.mean(ys)
        ax.set_xlim(mean_x - margin - range_x/2, mean_x + margin + range_x/2)
        ax.set_ylim(mean_y - margin - range_y/2, mean_y + margin + range_y/2)

    # Hide unused subplots
    for i in range(num_plots, len(axes)):
        axes[i].set_visible(False)

    plt.tight_layout()

    # Save figure
    output_file = '/tmp/particle_xy.png'
    plt.savefig(output_file, dpi=150)
    print(f"Saved plot to {output_file}")

    # Also create a single plot with trajectory and particle clouds
    fig2, ax2 = plt.subplots(figsize=(12, 10))

    # Plot particle clouds at selected steps with different colors
    colors = plt.cm.viridis(np.linspace(0, 1, len(indices)))

    for i, step_idx in enumerate(indices):
        particles = steps[step_idx]
        xs = [p['x'] for p in particles]
        ys = [p['y'] for p in particles]
        ax2.scatter(xs, ys, c=[colors[i]], s=20, alpha=0.5, label=f'Step {step_idx}')

    # Plot mean trajectory
    mean_xs = [np.mean([p['x'] for p in s]) for s in steps]
    mean_ys = [np.mean([p['y'] for p in s]) for s in steps]
    ax2.plot(mean_xs, mean_ys, 'k-', linewidth=2, label='Mean trajectory')
    ax2.scatter(mean_xs[0], mean_ys[0], c='green', s=200, marker='o', zorder=10, label='Start')
    ax2.scatter(mean_xs[-1], mean_ys[-1], c='red', s=200, marker='x', zorder=10, label='End')

    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Particle Distribution Over Trajectory')
    ax2.legend(loc='upper left', fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    output_file2 = '/tmp/particle_xy_trajectory.png'
    plt.savefig(output_file2, dpi=150)
    print(f"Saved trajectory plot to {output_file2}")

if __name__ == '__main__':
    main()
