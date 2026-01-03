#!/usr/bin/env python3
"""
ROS1 bag file to UDP JSON publisher
Reads LaserScan and Odometry from bag and sends as rosbridge JSON via UDP
"""

import sys
import json
import socket
import time
import argparse
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def sanitize_float(value: float, default: float = 0.0) -> float:
    """Replace inf/nan with default value for JSON compatibility"""
    import math
    if math.isnan(value) or math.isinf(value):
        return default
    return float(value)


def create_laser_scan_json(topic: str, msg, timestamp: int) -> dict:
    """Convert LaserScan message to rosbridge JSON format"""
    sec = timestamp // 1_000_000_000
    nanosec = timestamp % 1_000_000_000

    range_max = float(msg.range_max)

    return {
        "op": "publish",
        "topic": topic,
        "msg": {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": msg.header.frame_id
            },
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(msg.angle_increment),
            "time_increment": float(msg.time_increment),
            "scan_time": float(msg.scan_time),
            "range_min": float(msg.range_min),
            "range_max": range_max,
            "ranges": [sanitize_float(r, range_max) for r in msg.ranges],
            "intensities": [sanitize_float(i, 0.0) for i in msg.intensities] if len(msg.intensities) > 0 else []
        }
    }


def create_odometry_json(topic: str, msg, timestamp: int) -> dict:
    """Convert Odometry message to rosbridge JSON format"""
    sec = timestamp // 1_000_000_000
    nanosec = timestamp % 1_000_000_000

    return {
        "op": "publish",
        "topic": topic,
        "msg": {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": msg.header.frame_id
            },
            "child_frame_id": msg.child_frame_id,
            "pose": {
                "pose": {
                    "position": {
                        "x": float(msg.pose.pose.position.x),
                        "y": float(msg.pose.pose.position.y),
                        "z": float(msg.pose.pose.position.z)
                    },
                    "orientation": {
                        "x": float(msg.pose.pose.orientation.x),
                        "y": float(msg.pose.pose.orientation.y),
                        "z": float(msg.pose.pose.orientation.z),
                        "w": float(msg.pose.pose.orientation.w)
                    }
                },
                "covariance": [float(c) for c in msg.pose.covariance]
            },
            "twist": {
                "twist": {
                    "linear": {
                        "x": float(msg.twist.twist.linear.x),
                        "y": float(msg.twist.twist.linear.y),
                        "z": float(msg.twist.twist.linear.z)
                    },
                    "angular": {
                        "x": float(msg.twist.twist.angular.x),
                        "y": float(msg.twist.twist.angular.y),
                        "z": float(msg.twist.twist.angular.z)
                    }
                },
                "covariance": [float(c) for c in msg.twist.covariance]
            }
        }
    }


def main():
    parser = argparse.ArgumentParser(description='Publish ROS bag data as JSON over UDP')
    parser.add_argument('bag_file', help='Path to ROS bag file')
    parser.add_argument('--host', default='127.0.0.1', help='UDP host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9090, help='UDP port (default: 9090)')
    parser.add_argument('--scan-topic', default='/wr_scan_front_right',
                        help='Scan topic to use (default: /wr_scan_front_right)')
    parser.add_argument('--odom-topic', default='/odom',
                        help='Odometry topic to use (default: /odom)')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Playback rate multiplier (default: 1.0)')
    parser.add_argument('--start', type=float, default=0.0,
                        help='Start time offset in seconds (default: 0.0)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Duration to play in seconds (0 = all, default: 0)')
    parser.add_argument('--list-topics', action='store_true',
                        help='List available topics and exit')
    args = parser.parse_args()

    bag_path = Path(args.bag_file)
    if not bag_path.exists():
        print(f"Error: Bag file not found: {bag_path}")
        sys.exit(1)

    # Create type store for ROS1
    typestore = get_typestore(Stores.ROS1_NOETIC)

    with Reader(bag_path) as reader:
        # List topics mode
        if args.list_topics:
            print("Available topics:")
            for conn in reader.connections:
                print(f"  {conn.topic}: {conn.msgtype} ({conn.msgcount} msgs)")
            return

        # Find connections for our topics
        scan_conn = None
        odom_conn = None

        for conn in reader.connections:
            if conn.topic == args.scan_topic:
                scan_conn = conn
            elif conn.topic == args.odom_topic:
                odom_conn = conn

        if not scan_conn:
            print(f"Error: Scan topic '{args.scan_topic}' not found in bag")
            print("Available scan topics:")
            for conn in reader.connections:
                if 'LaserScan' in conn.msgtype:
                    print(f"  {conn.topic}")
            sys.exit(1)

        if not odom_conn:
            print(f"Error: Odom topic '{args.odom_topic}' not found in bag")
            sys.exit(1)

        print(f"=== Bag to UDP Publisher ===")
        print(f"Bag: {bag_path}")
        print(f"Target: {args.host}:{args.port}")
        print(f"Scan topic: {args.scan_topic} ({scan_conn.msgcount} msgs)")
        print(f"Odom topic: {args.odom_topic} ({odom_conn.msgcount} msgs)")
        print(f"Playback rate: {args.rate}x")
        print()

        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Collect and sort messages by timestamp
        messages = []
        connections = {scan_conn.id: scan_conn, odom_conn.id: odom_conn}

        print("Loading messages from bag...")
        for conn, timestamp, rawdata in reader.messages(connections=list(connections.values())):
            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            messages.append((timestamp, conn.topic, conn.msgtype, msg))

        messages.sort(key=lambda x: x[0])
        print(f"Loaded {len(messages)} messages")

        if len(messages) == 0:
            print("No messages to publish")
            return

        # Calculate time bounds
        start_timestamp = messages[0][0]
        start_offset_ns = int(args.start * 1e9)
        duration_ns = int(args.duration * 1e9) if args.duration > 0 else float('inf')

        # Filter messages by time
        filtered_messages = []
        for timestamp, topic, msgtype, msg in messages:
            rel_time = timestamp - start_timestamp
            if rel_time >= start_offset_ns:
                if rel_time - start_offset_ns <= duration_ns:
                    filtered_messages.append((timestamp, topic, msgtype, msg))

        if len(filtered_messages) == 0:
            print("No messages in specified time range")
            return

        print(f"Publishing {len(filtered_messages)} messages...")
        print("Press Ctrl+C to stop")
        print()

        # Publish messages
        base_timestamp = filtered_messages[0][0]
        base_time = time.time()

        scan_count = 0
        odom_count = 0

        try:
            for i, (timestamp, topic, msgtype, msg) in enumerate(filtered_messages):
                # Calculate delay
                rel_time = (timestamp - base_timestamp) / 1e9  # to seconds
                target_time = base_time + rel_time / args.rate

                # Wait until it's time to send
                now = time.time()
                if target_time > now:
                    time.sleep(target_time - now)

                # Create JSON message
                if 'LaserScan' in msgtype:
                    json_msg = create_laser_scan_json('/scan', msg, timestamp)
                    scan_count += 1
                elif 'Odometry' in msgtype:
                    json_msg = create_odometry_json('/odom', msg, timestamp)
                    odom_count += 1
                else:
                    continue

                # Send via UDP
                data = json.dumps(json_msg).encode('utf-8')
                sock.sendto(data, (args.host, args.port))

                # Progress update
                if (i + 1) % 100 == 0 or i == len(filtered_messages) - 1:
                    elapsed = time.time() - base_time
                    print(f"\rProgress: {i+1}/{len(filtered_messages)} "
                          f"(scan: {scan_count}, odom: {odom_count}) "
                          f"elapsed: {elapsed:.1f}s", end='', flush=True)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

        print(f"\n\nDone! Sent {scan_count} scans and {odom_count} odom messages")
        sock.close()


if __name__ == '__main__':
    main()
