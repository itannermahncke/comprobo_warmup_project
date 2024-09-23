"""
Drive parallel to the wall.
"""

import math
from math import pi

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowNode(Node):
    """
    Node that commands the Neato to search for a wall, find it, and drive
    parallel to it.
    """

    def __init__(self):
        """
        Initialize an instance of the wall follower node.
        """
        super().__init__("wall_follow_node")

        # for callback functions
        self.timer = self.create_timer(0.1, self.timer_callback)

        # get laser scans
        self.laser_subscriber = self.create_subscription(
            LaserScan, "scan", self.on_scan, qos_profile=10
        )

        # send velocity commands
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # mark points in rviz
        self.marker_publisher = self.create_publisher(Marker, "marker", 10)

        # track state (1 = approaching wall; 2 = aligning and following wall)
        self.current_state = 1

        # heading
        self.heading = None

        # tracking ranges from wall
        self.range_a = [None, None]  # r, theta
        self.range_b = [None, None]  # r, theta
        self.a_range = [20, 70]
        self.b_range = [90, 140]

        # useful attributes for porportional control
        self.distance_to_wall = None  # current distance
        self.Kp = 0.5  # coefficient for porportional component
        self.target_distance = 1.25  # ideal distance
        self.error = 0.01  # error for distance
        self.Kp_angle = 0.13  # coefficient for angle porportional

    def timer_callback(self):
        """
        Callback for timer. Figures out whether to look for, drive to, or drive
        parallel with a wall.
        """
        msg = Twist()

        # state 1: not currently following a wall
        if self.current_state == 1:
            # if no wall in sight
            if self.distance_to_wall is None:
                msg.linear.x = 0.1
            # if not close enough to a wall
            elif self.distance_to_wall - self.target_distance > self.error:
                # use proportional control to set the velocity
                msg.linear.x = self.Kp * (self.distance_to_wall - self.target_distance)
            # if close enough to a wall
            else:
                # move state forward
                self.current_state = 2

        # state 2: currently following a wall
        elif self.current_state == 2:
            # if range readings exist
            if None not in self.range_a and None not in self.range_b:
                # perform angle correction
                self.heading = self.determine_target_heading()
                if not math.isnan(self.heading):
                    msg.linear.x = 0.1
                    msg.angular.z = -1 * self.heading * self.Kp_angle

        # publish
        self.cmd_publisher.publish(msg)

    def on_scan(self, msg: LaserScan):
        """
        Callback when a scan is received.
        """
        # on first state, we only care about what is immediately ahead
        if self.current_state == 1:
            # look for a good range value
            for i in range(0, 10):
                if self.determine_range_quality(msg.ranges[i]):
                    self.distance_to_wall = msg.ranges[i]
                    index = i
                    break

            # mark point
            point = self.polar_to_cart(self.distance_to_wall, index)
            self.mark(point[0], point[1], False)

        # second state, we need two ranges for trig
        elif self.current_state == 2:
            # look for a good range value for the first range
            for i in range(self.a_range[0], self.a_range[-1]):
                if self.determine_range_quality(msg.ranges[i]):
                    self.range_a = [msg.ranges[i], i]
                    break

            # repeat for second range
            for i in range(self.b_range[0], self.b_range[-1]):
                if self.determine_range_quality(msg.ranges[i]):
                    self.range_b = [msg.ranges[i], i]
                    break

            # mark a
            point = self.polar_to_cart(self.range_a[0], self.range_a[1])
            self.mark(point[0], point[1], False)

            # mark b
            point = self.polar_to_cart(self.range_b[0], self.range_b[1])
            self.mark(point[0], point[1], False)

    def determine_target_heading(self):
        """
        Use trigonometry to determine the target side distance.
        """
        print(f"ra: {self.range_a} and rb: {self.range_b}")
        # Calculate the angle between neato heading and wall direction
        num = self.range_a[0] * math.cos(math.radians(self.range_a[1])) - self.range_b[
            0
        ] * math.cos(math.radians(self.range_b[1]))
        den = self.range_a[0] * math.sin(math.radians(self.range_a[1])) - self.range_b[
            0
        ] * math.sin(math.radians(self.range_b[1]))

        # return target heading
        return math.atan(num / den)

    def determine_range_quality(self, r):
        """
        Return True if range is a usable number; false if not.
        """
        if r != 0.0 and not math.isinf(r):
            return True
        return False

    def degree_to_rad(self, d):
        """
        Convert degree values to radians.
        """
        return d * pi / 180

    def polar_to_cart(self, r, theta):
        """
        Convert polar coords to cartesian.
        """
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return [x, y]

    def mark(self, x, y, average):
        """
        Mark a point given its cartesian coordinates. If average, color green
        instead of blue.
        """
        rgb = [0.0, 0.0, 0.0]
        if average:
            rgb[1] = 1.0
        else:
            rgb[2] = 1.0

        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
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


def main(args=None):
    """
    Main function to initialize ROS node.
    """
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
