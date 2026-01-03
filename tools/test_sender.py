#!/usr/bin/env python3
"""
Test sender for gmapping_json_server
Sends simulated scan and odometry data via UDP in rosbridge format.
"""

import socket
import json
import time
import math
import argparse


def create_laser_scan(timestamp, angle_min=-math.pi/2, angle_max=math.pi/2,
                      num_beams=360, range_min=0.1, range_max=30.0,
                      simulated_ranges=None):
    """Create a sensor_msgs/LaserScan message in rosbridge format."""

    angle_increment = (angle_max - angle_min) / (num_beams - 1)

    if simulated_ranges is None:
        # Generate simple simulated ranges (e.g., room-like environment)
        simulated_ranges = []
        for i in range(num_beams):
            angle = angle_min + i * angle_increment
            # Simple box room simulation
            dist = 5.0 + 2.0 * math.sin(angle * 3)  # Some variation
            if dist < range_min:
                dist = range_min
            if dist > range_max:
                dist = range_max
            simulated_ranges.append(dist)

    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)

    return {
        "op": "publish",
        "topic": "/scan",
        "msg": {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": "laser"
            },
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "time_increment": 0.0,
            "scan_time": 0.1,
            "range_min": range_min,
            "range_max": range_max,
            "ranges": simulated_ranges,
            "intensities": []
        }
    }


def create_odometry(timestamp, x, y, theta, vx=0.0, vy=0.0, vtheta=0.0):
    """Create a nav_msgs/Odometry message in rosbridge format."""

    # Convert theta to quaternion (only yaw rotation)
    qz = math.sin(theta / 2)
    qw = math.cos(theta / 2)

    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)

    return {
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": "odom"
            },
            "child_frame_id": "base_link",
            "pose": {
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}
                },
                "covariance": [0.0] * 36
            },
            "twist": {
                "twist": {
                    "linear": {"x": vx, "y": vy, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": vtheta}
                },
                "covariance": [0.0] * 36
            }
        }
    }


def simulate_room_scan(robot_x, robot_y, robot_theta, room_width=10.0, room_height=10.0,
                       angle_min=-math.pi/2, angle_max=math.pi/2, num_beams=360,
                       range_max=30.0):
    """Simulate laser scan in a rectangular room."""

    ranges = []
    angle_increment = (angle_max - angle_min) / (num_beams - 1)

    # Room walls
    walls = [
        (-room_width/2, -room_height/2, room_width/2, -room_height/2),  # bottom
        (room_width/2, -room_height/2, room_width/2, room_height/2),    # right
        (room_width/2, room_height/2, -room_width/2, room_height/2),    # top
        (-room_width/2, room_height/2, -room_width/2, -room_height/2),  # left
    ]

    for i in range(num_beams):
        angle = robot_theta + angle_min + i * angle_increment
        dx = math.cos(angle)
        dy = math.sin(angle)

        min_dist = range_max

        for x1, y1, x2, y2 in walls:
            # Ray-line segment intersection
            denom = dx * (y1 - y2) - dy * (x1 - x2)
            if abs(denom) < 1e-10:
                continue

            t = ((x1 - robot_x) * (y1 - y2) - (y1 - robot_y) * (x1 - x2)) / denom
            u = -((robot_x - x1) * dy - (robot_y - y1) * dx) / denom

            if t > 0 and 0 <= u <= 1:
                dist = t
                if dist < min_dist:
                    min_dist = dist

        ranges.append(min_dist)

    return ranges


def main():
    parser = argparse.ArgumentParser(description='Test sender for gmapping_json_server')
    parser.add_argument('--host', default='127.0.0.1', help='Target host')
    parser.add_argument('--port', type=int, default=9090, help='Target port')
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz')
    parser.add_argument('--duration', type=float, default=30.0, help='Duration in seconds')
    parser.add_argument('--room-width', type=float, default=10.0, help='Room width in meters')
    parser.add_argument('--room-height', type=float, default=10.0, help='Room height in meters')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Sending to {args.host}:{args.port}")
    print(f"Rate: {args.rate} Hz, Duration: {args.duration} s")
    print(f"Room size: {args.room_width} x {args.room_height} m")
    print()

    start_time = time.time()

    # Simple circular motion path
    linear_speed = 0.2  # m/s
    angular_speed = 0.1  # rad/s

    x, y, theta = 0.0, 0.0, 0.0

    dt = 1.0 / args.rate

    try:
        while time.time() - start_time < args.duration:
            current_time = time.time()
            timestamp = current_time - start_time

            # Update robot pose (simple motion model)
            x += linear_speed * math.cos(theta) * dt
            y += linear_speed * math.sin(theta) * dt
            theta += angular_speed * dt

            # Keep robot inside room
            margin = 1.0
            x = max(-args.room_width/2 + margin, min(args.room_width/2 - margin, x))
            y = max(-args.room_height/2 + margin, min(args.room_height/2 - margin, y))

            # Simulate laser scan
            ranges = simulate_room_scan(x, y, theta, args.room_width, args.room_height)

            # Create and send scan message
            scan_msg = create_laser_scan(timestamp, simulated_ranges=ranges)
            scan_json = json.dumps(scan_msg)
            sock.sendto(scan_json.encode(), (args.host, args.port))

            # Create and send odom message
            odom_msg = create_odometry(timestamp, x, y, theta,
                                       vx=linear_speed * math.cos(theta),
                                       vy=linear_speed * math.sin(theta),
                                       vtheta=angular_speed)
            odom_json = json.dumps(odom_msg)
            sock.sendto(odom_json.encode(), (args.host, args.port))

            print(f"\rTime: {timestamp:.1f}s, Pose: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)", end="")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nInterrupted")

    print(f"\nDone. Sent {int(args.duration * args.rate)} scan/odom pairs")
    sock.close()


if __name__ == '__main__':
    main()
