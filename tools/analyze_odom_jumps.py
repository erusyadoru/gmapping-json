#!/usr/bin/env python3
"""Analyze odom data for sudden jumps."""

import math
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

bag_file = '/home/industryalpha/gmapping-json/2025-11-18-08-13-15_merged.bag'
typestore = get_typestore(Stores.ROS1_NOETIC)

positions = []
orientations = []
timestamps = []

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

        positions.append((x, y))
        orientations.append(yaw)
        timestamps.append(timestamp / 1e9)

positions = np.array(positions)
orientations = np.array(orientations)
timestamps = np.array(timestamps)
timestamps = timestamps - timestamps[0]

# Find jumps
print("=== Odom Jump Analysis ===\n")

# Linear jumps
dx = np.diff(positions[:,0])
dy = np.diff(positions[:,1])
distances = np.sqrt(dx**2 + dy**2)
dt = np.diff(timestamps)

# Angular jumps
dyaw = np.diff(orientations)
dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))  # Normalize to [-pi, pi]

# Find significant jumps
print("Large position jumps (>5cm between consecutive samples):")
large_pos_jumps = np.where(distances > 0.05)[0]
for idx in large_pos_jumps[:20]:
    print(f"  t={timestamps[idx]:.2f}s: ({positions[idx,0]:.4f}, {positions[idx,1]:.4f}) -> "
          f"({positions[idx+1,0]:.4f}, {positions[idx+1,1]:.4f}) = {distances[idx]:.4f}m in {dt[idx]*1000:.1f}ms")

print(f"\nTotal large position jumps: {len(large_pos_jumps)}")

print("\n\nLarge yaw jumps (>0.1 rad = ~5.7 deg between consecutive samples):")
large_yaw_jumps = np.where(np.abs(dyaw) > 0.1)[0]
for idx in large_yaw_jumps[:30]:
    print(f"  t={timestamps[idx]:.2f}s: yaw {orientations[idx]:.4f} -> {orientations[idx+1]:.4f} "
          f"(delta={dyaw[idx]:.4f} rad = {np.degrees(dyaw[idx]):.1f} deg) in {dt[idx]*1000:.1f}ms")

print(f"\nTotal large yaw jumps: {len(large_yaw_jumps)}")

# Check timestamp intervals
print("\n\n=== Timestamp Analysis ===")
print(f"Mean interval: {np.mean(dt)*1000:.1f} ms")
print(f"Min interval: {np.min(dt)*1000:.1f} ms")
print(f"Max interval: {np.max(dt)*1000:.1f} ms")
print(f"Std interval: {np.std(dt)*1000:.1f} ms")

# Find timestamp gaps
large_gaps = np.where(dt > 0.1)[0]  # > 100ms
print(f"\nTimestamp gaps > 100ms:")
for idx in large_gaps[:10]:
    print(f"  t={timestamps[idx]:.2f}s: gap of {dt[idx]*1000:.1f}ms")

# Correlation between jumps and timestamp gaps
print("\n\n=== Jump at Timestamp Gaps ===")
for idx in large_yaw_jumps[:10]:
    if dt[idx] > 0.06:  # > 60ms gap
        print(f"  t={timestamps[idx]:.2f}s: yaw jump {np.degrees(dyaw[idx]):.1f} deg, gap {dt[idx]*1000:.1f}ms")
