#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class TurtleBot3Drive(Node):
    def __init__(self):
        super().__init__('turtlebot3_drive')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.target_x = 1.0  # Target x coordinate (1 meter ahead)
        self.target_y = 0.0  # Target y coordinate (straight ahead)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def timer_callback(self):
        cmd = Twist()

        # Calculate the distance to the target
        distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        
        # Proportional control for linear velocity
        linear_speed = 0.2 * distance

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angle_diff = angle_to_goal - self.current_yaw

        # Normalize the angle difference to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Proportional control for angular velocity
        angular_speed = 0.5 * angle_diff

        # Stop the robot if it is close enough to the target
        if distance < 0.05:
            linear_speed = 0.0
            angular_speed = 0.0
            self.get_logger().info('Reached the target, stopping the robot.')
        
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    turtlebot3_drive = TurtleBot3Drive()
    
    rclpy.spin(turtlebot3_drive)
    
    turtlebot3_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
