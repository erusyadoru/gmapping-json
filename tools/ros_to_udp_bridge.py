#!/usr/bin/env python3
"""
ROS to UDP Bridge for gmapping_json_server

This script subscribes to ROS topics (/scan and /odom) and forwards them
to gmapping_json_server via UDP in rosbridge JSON format.

Usage:
    rosrun gmapping_json ros_to_udp_bridge.py --host 192.168.1.100 --port 9090

Or standalone:
    python3 ros_to_udp_bridge.py --host 192.168.1.100 --port 9090
"""

import socket
import json
import argparse
import math

try:
    import rospy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS not available. This script requires ROS to function.")


class RosToUdpBridge:
    def __init__(self, host, port, scan_topic, odom_topic):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.scan_count = 0
        self.odom_count = 0

        # Subscribe to topics
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        rospy.loginfo(f"ROS to UDP Bridge started")
        rospy.loginfo(f"  Target: {host}:{port}")
        rospy.loginfo(f"  Scan topic: {scan_topic}")
        rospy.loginfo(f"  Odom topic: {odom_topic}")

    def scan_callback(self, msg):
        """Convert LaserScan to rosbridge JSON and send via UDP."""

        rosbridge_msg = {
            "op": "publish",
            "topic": "/scan",
            "msg": {
                "header": {
                    "stamp": {
                        "sec": msg.header.stamp.secs,
                        "nanosec": msg.header.stamp.nsecs
                    },
                    "frame_id": msg.header.frame_id
                },
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment,
                "scan_time": msg.scan_time,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": list(msg.ranges),
                "intensities": list(msg.intensities) if msg.intensities else []
            }
        }

        self._send(rosbridge_msg)
        self.scan_count += 1

        if self.scan_count % 100 == 0:
            rospy.loginfo(f"Sent {self.scan_count} scans, {self.odom_count} odoms")

    def odom_callback(self, msg):
        """Convert Odometry to rosbridge JSON and send via UDP."""

        rosbridge_msg = {
            "op": "publish",
            "topic": "/odom",
            "msg": {
                "header": {
                    "stamp": {
                        "sec": msg.header.stamp.secs,
                        "nanosec": msg.header.stamp.nsecs
                    },
                    "frame_id": msg.header.frame_id
                },
                "child_frame_id": msg.child_frame_id,
                "pose": {
                    "pose": {
                        "position": {
                            "x": msg.pose.pose.position.x,
                            "y": msg.pose.pose.position.y,
                            "z": msg.pose.pose.position.z
                        },
                        "orientation": {
                            "x": msg.pose.pose.orientation.x,
                            "y": msg.pose.pose.orientation.y,
                            "z": msg.pose.pose.orientation.z,
                            "w": msg.pose.pose.orientation.w
                        }
                    },
                    "covariance": list(msg.pose.covariance)
                },
                "twist": {
                    "twist": {
                        "linear": {
                            "x": msg.twist.twist.linear.x,
                            "y": msg.twist.twist.linear.y,
                            "z": msg.twist.twist.linear.z
                        },
                        "angular": {
                            "x": msg.twist.twist.angular.x,
                            "y": msg.twist.twist.angular.y,
                            "z": msg.twist.twist.angular.z
                        }
                    },
                    "covariance": list(msg.twist.covariance)
                }
            }
        }

        self._send(rosbridge_msg)
        self.odom_count += 1

    def _send(self, msg):
        """Send JSON message via UDP."""
        try:
            data = json.dumps(msg).encode('utf-8')
            self.sock.sendto(data, (self.host, self.port))
        except Exception as e:
            rospy.logwarn(f"Failed to send: {e}")


def main():
    parser = argparse.ArgumentParser(description='ROS to UDP Bridge for gmapping_json_server')
    parser.add_argument('--host', default='127.0.0.1', help='Target host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9090, help='Target port (default: 9090)')
    parser.add_argument('--scan-topic', default='/scan', help='LaserScan topic (default: /scan)')
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic (default: /odom)')

    # Parse args before rospy.init_node to handle --help
    args, unknown = parser.parse_known_args()

    if not ROS_AVAILABLE:
        print("Error: ROS is not available. Please source your ROS workspace.")
        return

    rospy.init_node('ros_to_udp_bridge', anonymous=True)

    bridge = RosToUdpBridge(args.host, args.port, args.scan_topic, args.odom_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")


if __name__ == '__main__':
    main()
