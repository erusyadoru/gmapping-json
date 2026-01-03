#!/usr/bin/env python3
"""
Rosbag to UDP sender for gmapping_json_server

Reads scan and odom data from a rosbag file and sends it to gmapping_json_server
via UDP in rosbridge JSON format.
"""

import socket
import json
import time
import math
import argparse
from pathlib import Path

from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.msg import get_types_from_msg


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_tf_static_transforms(bag_path):
    """Extract static transforms from bag file."""
    transforms = {}

    # Determine bag format
    bag_path = Path(bag_path)

    try:
        reader = Reader1(bag_path)
        is_ros1 = True
    except:
        reader = Reader2(bag_path)
        is_ros1 = False

    typestore = get_typestore(Stores.ROS1_NOETIC if is_ros1 else Stores.ROS2_HUMBLE)

    # Register tf2_msgs types
    TF2_MSG = 'geometry_msgs/TransformStamped[] transforms'
    try:
        add_types = get_types_from_msg(TF2_MSG, 'tf2_msgs/msg/TFMessage')
        typestore.register(add_types)
    except:
        pass

    with reader:
        for connection, timestamp, rawdata in reader.messages():
            if '/tf_static' in connection.topic:
                try:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype) if not is_ros1 else typestore.deserialize_ros1(rawdata, connection.msgtype)

                    for transform in msg.transforms:
                        child_frame = transform.child_frame_id
                        parent_frame = transform.header.frame_id
                        trans = transform.transform

                        transforms[child_frame] = {
                            'parent': parent_frame,
                            'x': trans.translation.x,
                            'y': trans.translation.y,
                            'z': trans.translation.z,
                            'qx': trans.rotation.x,
                            'qy': trans.rotation.y,
                            'qz': trans.rotation.z,
                            'qw': trans.rotation.w,
                        }
                except Exception as e:
                    pass

    return transforms


def create_scan_msg(scan_data, timestamp, frame_id="laser"):
    """Create rosbridge format LaserScan message."""
    sec = int(timestamp / 1e9)
    nanosec = int(timestamp % 1e9)

    # Replace inf/nan with range_max for JSON compatibility
    range_max = float(scan_data.range_max)
    ranges = []
    for r in scan_data.ranges:
        r_float = float(r)
        if math.isnan(r_float) or math.isinf(r_float) or r_float < 0:
            ranges.append(range_max)
        else:
            ranges.append(r_float)

    return {
        "op": "publish",
        "topic": "/scan",
        "msg": {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": frame_id
            },
            "angle_min": float(scan_data.angle_min),
            "angle_max": float(scan_data.angle_max),
            "angle_increment": float(scan_data.angle_increment),
            "time_increment": float(scan_data.time_increment),
            "scan_time": float(scan_data.scan_time),
            "range_min": float(scan_data.range_min),
            "range_max": range_max,
            "ranges": ranges,
            "intensities": []  # Skip intensities to avoid inf issues
        }
    }


def create_odom_msg(odom_data, timestamp):
    """Create rosbridge format Odometry message."""
    sec = int(timestamp / 1e9)
    nanosec = int(timestamp % 1e9)

    return {
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": odom_data.header.frame_id
            },
            "child_frame_id": odom_data.child_frame_id if hasattr(odom_data, 'child_frame_id') else "base_link",
            "pose": {
                "pose": {
                    "position": {
                        "x": float(odom_data.pose.pose.position.x),
                        "y": float(odom_data.pose.pose.position.y),
                        "z": float(odom_data.pose.pose.position.z)
                    },
                    "orientation": {
                        "x": float(odom_data.pose.pose.orientation.x),
                        "y": float(odom_data.pose.pose.orientation.y),
                        "z": float(odom_data.pose.pose.orientation.z),
                        "w": float(odom_data.pose.pose.orientation.w)
                    }
                },
                "covariance": list(odom_data.pose.covariance) if hasattr(odom_data.pose, 'covariance') else [0.0] * 36
            },
            "twist": {
                "twist": {
                    "linear": {
                        "x": float(odom_data.twist.twist.linear.x),
                        "y": float(odom_data.twist.twist.linear.y),
                        "z": float(odom_data.twist.twist.linear.z)
                    },
                    "angular": {
                        "x": float(odom_data.twist.twist.angular.x),
                        "y": float(odom_data.twist.twist.angular.y),
                        "z": float(odom_data.twist.twist.angular.z)
                    }
                },
                "covariance": list(odom_data.twist.covariance) if hasattr(odom_data.twist, 'covariance') else [0.0] * 36
            }
        }
    }


def main():
    parser = argparse.ArgumentParser(description='Send rosbag data to gmapping_json_server')
    parser.add_argument('bagfile', help='Path to rosbag file')
    parser.add_argument('--host', default='127.0.0.1', help='Target host')
    parser.add_argument('--port', type=int, default=9090, help='Target port')
    parser.add_argument('--scan-topic', default='/wr_scan_rear', help='LaserScan topic')
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic')
    parser.add_argument('--rate-multiplier', type=float, default=1.0, help='Playback rate multiplier')
    parser.add_argument('--start', type=float, default=0, help='Start time offset (seconds)')
    parser.add_argument('--duration', type=float, default=-1, help='Duration to play (seconds, -1 for all)')
    parser.add_argument('--show-tf', action='store_true', help='Show tf_static transforms and exit')
    args = parser.parse_args()

    bag_path = Path(args.bagfile)
    if not bag_path.exists():
        print(f"Error: Bag file not found: {bag_path}")
        return

    # Show tf_static if requested
    if args.show_tf:
        print("Reading tf_static transforms...")
        transforms = get_tf_static_transforms(bag_path)
        if transforms:
            for frame_id, tf in transforms.items():
                yaw = quaternion_to_yaw(tf['qx'], tf['qy'], tf['qz'], tf['qw'])
                print(f"  {tf['parent']} -> {frame_id}:")
                print(f"    translation: ({tf['x']:.4f}, {tf['y']:.4f}, {tf['z']:.4f})")
                print(f"    rotation: quat=({tf['qx']:.4f}, {tf['qy']:.4f}, {tf['qz']:.4f}, {tf['qw']:.4f}), yaw={math.degrees(yaw):.2f}°")
        else:
            print("  No tf_static transforms found")
        return

    # Determine bag format
    try:
        reader = Reader1(bag_path)
        is_ros1 = True
        print("Detected ROS1 bag format")
    except:
        reader = Reader2(bag_path)
        is_ros1 = False
        print("Detected ROS2 bag format")

    typestore = get_typestore(Stores.ROS1_NOETIC if is_ros1 else Stores.ROS2_HUMBLE)

    # Register tf2_msgs types
    TF2_MSG = 'geometry_msgs/TransformStamped[] transforms'
    try:
        add_types = get_types_from_msg(TF2_MSG, 'tf2_msgs/msg/TFMessage')
        typestore.register(add_types)
    except:
        pass

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Sending to {args.host}:{args.port}")
    print(f"Scan topic: {args.scan_topic}")
    print(f"Odom topic: {args.odom_topic}")
    print()

    # Collect all messages first
    messages = []
    scan_count = 0
    odom_count = 0

    with reader:
        # Get bag info
        print(f"Bag topics:")
        for conn in reader.connections:
            print(f"  {conn.topic}: {conn.msgtype} ({conn.msgcount} msgs)")
        print()

        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic

            if topic == args.scan_topic or topic == args.odom_topic:
                try:
                    if is_ros1:
                        msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
                    else:
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                    messages.append((timestamp, topic, msg))

                    if topic == args.scan_topic:
                        scan_count += 1
                    else:
                        odom_count += 1
                except Exception as e:
                    print(f"Error deserializing {topic}: {e}")

    if not messages:
        print("No matching messages found!")
        return

    # Sort by timestamp
    messages.sort(key=lambda x: x[0])

    print(f"Found {scan_count} scans and {odom_count} odoms")

    start_time = messages[0][0]
    end_time = messages[-1][0]
    duration = (end_time - start_time) / 1e9
    print(f"Duration: {duration:.2f} seconds")
    print()

    # Apply start offset
    start_offset_ns = int(args.start * 1e9)

    # Playback
    print("Starting playback... (Ctrl+C to stop)")

    last_timestamp = None
    sent_scans = 0
    sent_odoms = 0
    latest_odom = None

    try:
        for timestamp, topic, msg in messages:
            if timestamp < start_time + start_offset_ns:
                continue

            if args.duration > 0:
                if (timestamp - start_time) / 1e9 > args.start + args.duration:
                    break

            # Timing
            if last_timestamp is not None:
                sleep_time = (timestamp - last_timestamp) / 1e9 / args.rate_multiplier
                if sleep_time > 0 and sleep_time < 1.0:
                    time.sleep(sleep_time)

            last_timestamp = timestamp

            # Send message
            if topic == args.scan_topic:
                json_msg = create_scan_msg(msg, timestamp)
                sock.sendto(json.dumps(json_msg).encode(), (args.host, args.port))
                sent_scans += 1

                # Also send latest odom with scan
                if latest_odom is not None:
                    odom_json = create_odom_msg(latest_odom[1], timestamp)
                    sock.sendto(json.dumps(odom_json).encode(), (args.host, args.port))

                elapsed = (timestamp - start_time) / 1e9
                print(f"\rTime: {elapsed:.1f}s, Scans: {sent_scans}, Odoms: {sent_odoms}", end="")

            elif topic == args.odom_topic:
                json_msg = create_odom_msg(msg, timestamp)
                sock.sendto(json.dumps(json_msg).encode(), (args.host, args.port))
                sent_odoms += 1
                latest_odom = (timestamp, msg)

    except KeyboardInterrupt:
        print("\nPlayback interrupted")

    print(f"\nDone. Sent {sent_scans} scans and {sent_odoms} odoms")
    sock.close()


if __name__ == '__main__':
    main()
