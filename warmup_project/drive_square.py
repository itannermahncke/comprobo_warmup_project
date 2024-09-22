"""
Drive the Neato in a square.
"""

from math import pi

import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DriveSquareNode(Node):
    """
    Node to command the Neato to drive in a 1x1 (meters) square.
    """

    def __init__(self):
        """
        Initialize an instance of the DriveSquareNode.
        """
        super().__init__("drive_square_node")

        # hard code list of state goals; active goal at 0; motion type at linear
        self.state_goals = [
            # x, y, theta (zeroed at startup)
            [1.0, 0.0, 0.0],
            [1.0, 0.0, pi / 2],
            [1.0, 1.0, pi / 2],
            [1.0, 1.0, pi],
            [0.0, 1.0, pi],
            [0.0, 1.0, -1 * pi / 2],
            [0.0, 0.0, -1 * pi / 2],
            [0.0, 0.0, 0.0],
        ]
        self.active_goal = 0
        self.lin_or_ang = True  # linear (True) or angular (False)

        # publisher that periodically shares the Neato's progress
        self.goal_publisher = self.create_publisher(String, "goal_status", 10)

        # timer to periodically give motion commands to the Neato
        timer_period = 0.1  # 10 Hz
        self.cmd_timer = self.create_timer(timer_period, self.timer_callback)

        # publisher that sends commands to the Neato
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # subscriber that receives encoder data from the Neato
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.on_odometry, 10
        )
        self.recent_odom = None  # x, y, and theta from Neato odometry

        # other useful attributes
        self.err = 0.075  # acceptable error for meeting goals
        self.vel = 0.1
        self.odom_count = 0

    def timer_callback(self):
        """
        Callback function for the node timer. Each time the timer goes off,
        determine the motion state of the Neato and behave accordingly.
        """
        # can only behave if there is recent odometry
        if self.recent_odom is not None:
            # first, compare current odometry to current state goal
            goal_met = True
            for i in range(0, 3):
                # if any attribute (x, y, theta) fails, the goal is not met
                goal_met = (
                    goal_met
                    and abs(self.recent_odom[i] - self.state_goals[self.active_goal][i])
                    < self.err
                )

            # if state goal is met, change state and motion behavior
            if goal_met:
                self.active_goal += 1
                self.lin_or_ang = not self.lin_or_ang

                # if the end is reached, stop moving and be done
                if self.active_goal == len(self.state_goals):
                    end_msg = Twist()
                    self.vel_publisher.publish(end_msg)
                    self.cmd_timer.destroy()

            # then iterate on current state (either move forward or turn left)
            if self.lin_or_ang:
                self.drive_forward()
            else:
                self.turn_left()

        # no matter what, publish goal status to topic
        self.publish_goal_status()

    def on_odometry(self, odom_msg):
        """
        Callback function for odometry subscriber. Store current odometry
        values for use in calculations.
        """
        # increase message count
        self.odom_count += 1
        # get the Pose and the orientation (tilt) variables
        odom_pose = odom_msg.pose.pose
        odom_orientation = [
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w,
        ]

        # store x, y, and theta relative to zero
        self.recent_odom = [
            odom_pose.position.x,
            odom_pose.position.y,
            euler_from_quaternion(odom_orientation)[2],
        ]

    def drive_forward(self):
        """
        Commands the Neato to drive forward at a rate of 1 m/s.
        """
        # create a message to drive forward and publish
        fwd_msg = Twist()
        fwd_msg.linear.x = self.vel
        self.vel_publisher.publish(fwd_msg)

    def turn_left(self):
        """
        Commands the Neato to turn left at a rate of 0.1 rad/s.
        """
        # create a message to drive forward and publish
        fwd_msg = Twist()
        fwd_msg.angular.z = self.vel * 2
        self.vel_publisher.publish(fwd_msg)

    def publish_goal_status(self):
        """
        Publishes a String message describing the state of the Neato in relation
        to its goals.
        """
        # if final goal has not been met
        if self.active_goal <= len(self.state_goals) - 1:
            content = (
                f"Msg #: {self.odom_count}"
                + f"\nGoal: {self.active_goal}"
                + f"\nGoal vector: {self.state_goals[self.active_goal]}"
                + f"\nOdom vector: {self.recent_odom}"
            )
        # if final goal has been met
        else:
            content = "Finished driving :)"

        # create message and publish
        str_msg = String(data=content)
        self.goal_publisher.publish(str_msg)


def main(args=None):
    """
    Spin an instance of the DriveSquareNode, then clean up.
    """
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
