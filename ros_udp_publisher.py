#!/usr/bin/env python3
"""
ROS Topic to UDP/JSON Publisher (Rosbridge Format)

Publishes /odom, /wr_scan_front_left, and tf_static to localhost via UDP/JSON
in rosbridge protocol format.
"""

import rospy
import json
import socket
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class RosUdpPublisher:
    def __init__(self, host='127.0.0.1', port=9090):
        self.host = host
        self.port = port

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Store tf_static for laser pose
        self.laser_pose = None

        # ROS subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/wr_scan_front_left', LaserScan, self.scan_callback)
        rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback, queue_size=10)

        rospy.loginfo("UDP Publisher initialized (rosbridge format)")
        rospy.loginfo("  Target: {}:{}".format(host, port))
        rospy.loginfo("  Topics: /odom, /wr_scan_front_left, /tf_static")

    def send_udp(self, data):
        """Send JSON data via UDP."""
        try:
            json_str = json.dumps(data)
            self.sock.sendto(json_str.encode('utf-8'), (self.host, self.port))
        except Exception as e:
            rospy.logwarn("UDP send error: {}".format(e))

    def odom_callback(self, msg):
        """Handle odometry messages - convert to rosbridge format."""
        data = {
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

        self.send_udp(data)

    def scan_callback(self, msg):
        """Handle laser scan messages - convert to rosbridge format."""
        # Convert ranges to list, handling inf/nan
        ranges = []
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                ranges.append(msg.range_max)
            else:
                ranges.append(r)

        data = {
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
                "ranges": ranges,
                "intensities": list(msg.intensities) if msg.intensities else []
            }
        }

        self.send_udp(data)

    def tf_static_callback(self, msg):
        """Handle tf_static messages - find and store laser pose."""
        for tf in msg.transforms:
            # Look for laser frame relative to base_link
            if tf.header.frame_id == "base_link" and "front_left" in tf.child_frame_id:
                self.laser_pose = {
                    "x": tf.transform.translation.x,
                    "y": tf.transform.translation.y,
                    "theta": self.quaternion_to_yaw(tf.transform.rotation)
                }
                rospy.loginfo("Laser pose found: x={:.3f}, y={:.3f}, theta={:.3f}".format(
                    self.laser_pose["x"], self.laser_pose["y"], self.laser_pose["theta"]))

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_laser_pose(self):
        """Return laser pose for gmapping configuration."""
        return self.laser_pose

    def run(self):
        """Run the publisher."""
        rospy.spin()


def main():
    rospy.init_node('ros_udp_publisher', anonymous=True)

    # Parameters
    host = rospy.get_param('~host', '127.0.0.1')
    port = rospy.get_param('~port', 9090)

    publisher = RosUdpPublisher(host, port)

    # Wait for tf_static to get laser pose
    rospy.sleep(1.0)
    laser_pose = publisher.get_laser_pose()
    if laser_pose:
        rospy.loginfo("Use these gmapping parameters:")
        rospy.loginfo("  --laser-x {:.4f} --laser-y {:.4f} --laser-theta {:.4f}".format(
            laser_pose["x"], laser_pose["y"], laser_pose["theta"]))

    publisher.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
