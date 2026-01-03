#!/usr/bin/env python3
import rosbag
import socket
import json
import time
import math
import sys
import argparse

def play_bag(bag_path, speed=2.0, scan_topic=None, odom_topic=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Opening bag: {bag_path}")
    bag = rosbag.Bag(bag_path)

    # トピック情報取得
    topics = bag.get_type_and_topic_info().topics

    # トピック自動検出または指定されたトピックを使用
    if scan_topic:
        if scan_topic not in topics:
            print(f"Error: Specified scan topic '{scan_topic}' not found in bag")
            print(f"Available topics: {list(topics.keys())}")
            return False
        print(f"Using specified scan topic: {scan_topic}")
    else:
        for topic in topics:
            if 'scan' in topic.lower():
                scan_topic = topic
                print(f"Scan topic detected: {topic}")
                break

    if odom_topic:
        if odom_topic not in topics:
            print(f"Error: Specified odom topic '{odom_topic}' not found in bag")
            print(f"Available topics: {list(topics.keys())}")
            return False
        print(f"Using specified odom topic: {odom_topic}")
    else:
        for topic in topics:
            if 'odom' in topic.lower():
                odom_topic = topic
                print(f"Odom topic detected: {topic}")
                break

    if not scan_topic or not odom_topic:
        print(f"Error: Required topics not found. scan={scan_topic}, odom={odom_topic}")
        print(f"Available topics: {list(topics.keys())}")
        return False

    messages = []
    for topic, msg, t in bag.read_messages(topics=[scan_topic, odom_topic]):
        messages.append((t.to_sec(), topic, msg))

    messages.sort(key=lambda x: x[0])
    print(f"Loaded {len(messages)} messages")

    start_time = None
    playback_start = time.time()
    scan_count = 0
    odom_count = 0

    for t, topic, msg in messages:
        if start_time is None:
            start_time = t

        elapsed = (t - start_time) / speed
        while time.time() - playback_start < elapsed:
            time.sleep(0.001)

        if topic == odom_topic:
            odom_json = {
                "op": "publish",
                "topic": "/odom",
                "msg": {
                    "header": {
                        "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
                        "frame_id": msg.header.frame_id
                    },
                    "pose": {"pose": {
                        "position": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
                        "orientation": {"x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}
                    }}
                }
            }
            sock.sendto(json.dumps(odom_json).encode(), ("127.0.0.1", 9090))
            odom_count += 1

        elif topic == scan_topic:
            ranges = []
            for r in msg.ranges:
                if math.isnan(r) or math.isinf(r) or r > msg.range_max:
                    ranges.append(float(msg.range_max))
                elif r < msg.range_min:
                    ranges.append(float(msg.range_min))
                else:
                    ranges.append(float(r))

            scan_json = {
                "op": "publish",
                "topic": "/scan",
                "msg": {
                    "header": {
                        "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
                        "frame_id": msg.header.frame_id
                    },
                    "angle_min": float(msg.angle_min),
                    "angle_max": float(msg.angle_max),
                    "angle_increment": float(msg.angle_increment),
                    "time_increment": float(msg.time_increment),
                    "scan_time": float(msg.scan_time),
                    "range_min": float(msg.range_min),
                    "range_max": float(msg.range_max),
                    "ranges": ranges,
                    "intensities": []
                }
            }
            sock.sendto(json.dumps(scan_json).encode(), ("127.0.0.1", 9090))
            scan_count += 1

            if scan_count % 50 == 0:
                print(f"Progress: {scan_count} scans, {odom_count} odoms, elapsed={time.time()-playback_start:.1f}s")

    print(f"Done! Sent {scan_count} scans, {odom_count} odoms")
    bag.close()
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Play ROS bag to UDP')
    parser.add_argument('bag_path', help='Path to bag file')
    parser.add_argument('--speed', type=float, default=2.0, help='Playback speed')
    parser.add_argument('--scan-topic', help='Scan topic name')
    parser.add_argument('--odom-topic', help='Odom topic name')
    args = parser.parse_args()

    success = play_bag(args.bag_path, args.speed, args.scan_topic, args.odom_topic)
    sys.exit(0 if success else 1)
