"""
Follow a person.
"""

import rclpy
from rclpy.node import Node

import math
from math import pi

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class PersonFollowerNode(Node):
    """
    Node for commanding the Neato to maintain a close distance from a scanned
    person/moving object.
    """

    def __init__(self):
        """
        Initialize an instance of the PersonFollowerNode.
        """
        super().__init__("person_follower")

        # subscriber for laser scans
        self.scan_subscriber = self.create_subscription(
            LaserScan, "stable_scan", self.on_scan, 10
        )
        self.latest_scan = None
        self.fov = 90.0

        # timer
        self.timestep = 0.5  # 10 Hz
        self.timer = self.create_timer(self.timestep, self.timer_callback)

        # publisher that sends commands to the Neato
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # publisher for marker
        self.marker_publisher = self.create_publisher(Marker, "marker", 10)

        # attributes for PID control (linear only)
        self.process_variable = [None, None]  # current guess for person's location
        self.set_point = 1.0  # ideal difference in distance
        self.current_error = None  # current error from this timestep
        self.previous_error = 0.0  # error from last timestep
        self.integral = 0.0  # maintain prior integral value
        self.k_vals = [0.5, 0.0, 0.0]  # Kp, Ki, Kd

        # attributes for porportional control (angular)
        self.kp_angle = 0.8

    def timer_callback(self):
        """
        Callback function for timer. Examines the current guess for the target
        person's location and determines how the Neato should move to get there.
        """
        # message to publish
        vel_msg = Twist()

        # only act if process variable is available
        if None not in self.process_variable:
            vel_msg.linear.x = self.pid_control()
            # turn at a porportional rate
            vel_msg.angular.z = self.kp_angle * self.process_variable[1]

        # publish
        print(f"Publish v = {vel_msg.linear.x}, w = {vel_msg.angular.z}")
        self.cmd_publisher.publish(vel_msg)

    def on_scan(self, msg: LaserScan):
        """
        Respond to scan messages. Filter out any bad points, then find an
        average point that represents the target person's rough location.
        """
        # pack ranges and angles
        rt = []  # array of r/theta points

        # filter out bad r values
        for angle, r in enumerate(msg.ranges):
            if self.determine_range_quality(r):
                if angle < self.fov or abs(angle - 360) < self.fov:
                    rt.append([r, angle])

        # convert points to cartesian
        x_avg = []
        y_avg = []
        for i, point in enumerate(rt):
            x = point[0] * math.cos(self.degree_to_rad(point[1]))
            y = point[0] * math.sin(self.degree_to_rad(point[1]))

            x_avg.append(x)
            y_avg.append(y)

            # mark in rviz
            self.mark(x, y, i + 1, average=False)

        if len(x_avg) > 0 and len(y_avg) > 0:
            # find average x and y values
            x_avg = sum(x_avg) / len(x_avg)
            y_avg = sum(y_avg) / len(y_avg)

            # mark in rviz
            self.mark(x_avg, y_avg, 0, average=True)

            # convert averages back to polar
            r_avg = ((x_avg**2) + (y_avg**2)) ** 0.5
            theta_avg = math.atan2(y_avg, x_avg)

            # save person location guess to process variable
            self.process_variable = [r_avg, theta_avg]

    def pid_control(self):
        """
        Runs PID calculations for linear velocity only.
        """
        # find error btwn set point and process variable
        self.current_error = abs(self.set_point - self.process_variable[0])

        # determine P, I, D values
        p_val = self.current_error
        i_val = self.integral + self.current_error * self.timestep
        d_val = (self.current_error - self.previous_error) / self.timestep

        # update old values
        self.integral = i_val
        self.previous_error = self.current_error

        # determine output
        output = float(
            self.k_vals[0] * p_val + self.k_vals[1] * i_val + self.k_vals[2] * d_val
        )
        return output

    def mark(self, x, y, i, average):
        """
        Mark a point given its cartesian coordinates. If average, color green
        instead of blue.
        """
        # color average value differently
        rgb = [0.0, 0.0, 0.0]
        if average:
            rgb[1] = 1.0
        else:
            rgb[2] = 1.0

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
        if r != 0.0 and not math.isinf(r) and r < 1.5:
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
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
