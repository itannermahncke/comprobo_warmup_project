"""
Avoid obstacles.
"""

import rclpy
from rclpy.node import Node

import math
from math import pi

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ObstacleAvoidNode(Node):
    """
    Node for commanding the Neato to maintain a close distance from a scanned
    person/moving object.
    """

    def __init__(self):
        """
        Initialize an instance of the ObstacleAvoidNode.
        """
        super().__init__("obstacle_avoider")

        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # velocity publisher
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # marker publisher
        self.marker_publisher = self.create_publisher(Marker, "marker", 10)

        # scan subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan, "stable_scan", self.on_scan, 10
        )

        # useful attributes
        self.fov = 90.0  # defined field of view
        self.closest_r = 10.0  # to avoid collisions
        self.left_scans = [[], []]
        self.right_scans = [[], []]

    def timer_callback(self):
        """
        Callback function for timer. Based on scan counts and values on the
        left and rightside of the Neato, adjust angular velocity while driving
        constantly forward. If the ranges are close enough, stop driving and
        only turn.
        """
        # create message
        vel_msg = Twist()

        # don't turn if too close
        if self.closest_r > 0.3:
            vel_msg.linear.x = 0.1
        else:
            print(f"Too close to drive forward: {self.closest_r}.")

        # only go forward if ranges have been taken
        if len(self.left_scans[1]) > 0 and len(self.right_scans[1]) > 0:
            # flip shortest range (so small values create big coefficients, and vice versa)
            left_k = 1.0 / min(self.left_scans[1]) * -1
            right_k = 1.0 / min(self.right_scans[1])
            vel_msg.angular.z = 0.1 * (left_k + right_k)
        else:
            print("WARN: NO DATA TO TURN")

        # publish
        print(f"Linear: {vel_msg.linear.x}; Angular: {vel_msg.angular.z}")
        self.cmd_publisher.publish(vel_msg)

    def on_scan(self, msg: LaserScan):
        """
        Callback for receiving scans. Log scans within a field of view and sort
        them depending on whether they are on the left or right side of the Neato.
        """
        # arrays
        self.left_scans = [[], []]
        self.right_scans = [[], []]

        avg_r = []
        i = 0

        # sort ranges
        for angle, r in enumerate(msg.ranges):
            i += 1
            # only examine good quality ranges
            if self.determine_range_quality(r):
                # if scan is on the left side
                if abs(angle - 360) < self.fov:
                    print(f"Left angle: {angle}")
                    self.left_scans[0].append(angle)
                    self.left_scans[1].append(r)
                    avg_r.append(r)
                    self.mark(r, angle, i, "left")
                # if scan is on the right side
                elif 0.0 < angle < self.fov:
                    print(f"Right angle: {angle}")
                    self.right_scans[0].append(angle)
                    self.right_scans[1].append(r)
                    avg_r.append(r)
                    self.mark(r, angle, i, "right")

        # compute closest range
        if len(avg_r) > 0:
            self.closest_r = min(avg_r)
        # if impossible, set it far away to keep driving forward
        else:
            self.closest_r = 10.0

    def mark(self, r, theta, i, dot_type):
        """
        Mark a point given its polar coordinates. If right, color green
        instead of blue. If robot, mark red.
        """
        # convert to cartesian
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        # color average value differently
        rgb = [0.0, 0.0, 0.0]
        if dot_type == "right":
            rgb[1] = 1.0
        elif dot_type == "left":
            rgb[2] = 1.0
        else:
            rgb[0] = 1.0

        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]

        self.marker_publisher.publish(marker)

    def determine_range_quality(self, r):
        """
        Return True if range is a usable number; false if not.
        """
        if r != 0.0 and not math.isinf(r) and r < 1.0:
            return True
        return False

    def degree_to_rad(self, d):
        """
        Convert degree values to radians.
        """
        return d * pi / 180


def main(args=None):
    """
    Spin an instance of the DriveSquareNode, then clean up.
    """
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
