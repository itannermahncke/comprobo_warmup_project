import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
import math

class WallFollower(Node):
    '''A class that enables a Neato, placed near a wall, to readjust its 
    direction to be parallel to the wall as it moves forward'''
    def __init__(self):
        super().__init__('wall_follower_node')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_wall is used to communciate laser data to run_loop
        self.distance_to_wall = None
        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.4
        self.vel = 0.1
        # target_distance is the desired distance to the wall in front
        self.target_distance = 1.2

    def handle_estop(self, msg):
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """ 
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)
    
    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def turn_right(self):
        # commands the Neato to ultimately turn right
        if not self.e_stop.is_set():
            self.drive(linear=0.0, angular=6.1)
            sleep(math.pi / 6.1 / 2)
            self.drive(linear=0.0, angular=0.0)

    def turn_left(self):
        # Commands the Neato to turn left at a rate of 0.1 rad/s ish.
        if not self.e_stop.is_set():
            self.drive(linear=0.0, angular=0.1)
            sleep(math.pi / 0.1 / 2)
            self.drive(linear=0.0, angular=0.0)
        
        #fwd_msg = Twist()
        #fwd_msg.angular.z = self.vel * 2
        #self.vel_publisher.publish(fwd_msg)

    def run_loop(self):
        
        msg = Twist()
        if self.distance_to_wall is None:
            # if we have detected a wall, turn accordingly
            msg.linear.x = 0.1
        else:
            # use proportional control to set the velocity
            msg.linear.x = self.Kp*(self.distance_to_wall - self.target_distance)
        self.vel_pub.publish(msg)
    
    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            # checking for the value 0.0 ensures the data is valid.
            # Your logic here!
            print('scan received', msg.ranges[0])

    def angle_normalize(z):
    # convenience function to map an angle to the range [-pi,pi] 
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(a, b):
        """Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b

       examples:
           angle_diff(.1,.2) -> -.1
           angle_diff(.1, 2*math.pi - .1) -> .2
           angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = angle_normalize(a)
        b = angle_normalize(b)

        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2
        
def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
