#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import sqrt, acos, atan2, pi
import numpy as np
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy

GOAL_POINT = [2.0, 0.0, 0.0]
START_POINT = [0.0, 0.0, 0.0]
thresholdForLine = 0.05
OBJECT_HIT_DIST = 1  # when do we consider a hit-point? in meters
OBJECT_FOLLOW_DIST = 1.2  # How closely do we follow the obstacle? in meters
TARGET_FORWARD_VELOCITY = 0.5  # m/s


class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scan_pub = self.create_publisher(LaserScan, '/scan', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.position = None
        self.orientation = None
        self.laser_ranges = None
        
        self.leave_point = START_POINT
        self.hit_location_reencountered = False

        self.timer = self.create_timer(0.1, self.main_loop)  # 10 Hz

    def odom_callback(self, msg):
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.orientation = msg.pose.pose.orientation

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges

    def set_velocities(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_pub.publish(twist)

    def m_line(self):
        return np.array(START_POINT) - np.array(GOAL_POINT)

    def current_vector_to_goal(self):
        return np.array(self.position) - np.array(GOAL_POINT)

    def is_on_mline(self):
        dproduct = np.dot(self.m_line(), self.current_vector_to_goal())
        angleBTWVectors = (dproduct / (np.linalg.norm(self.m_line()) * np.linalg.norm(self.current_vector_to_goal())))
        trueAngle = np.arccos(np.clip(angleBTWVectors, -1, 1))
        return trueAngle

    def pure_pursuit(self, x_track_error, angle_error):
        ang_vel = -0.05 * x_track_error + 0.1 * angle_error
        self.set_velocities(TARGET_FORWARD_VELOCITY, ang_vel)

    def get_my_location(self):
        return self.position

    def distance_to(self, goal_location):
        return np.linalg.norm(np.array(goal_location) - np.array(self.get_my_location()))

    def distance_to_line(self, start_point, end_point):
        A = np.array(start_point)
        B = np.array(end_point)
        P = np.array(self.get_my_location())
        n = np.cross([0, 0, 1], (B - A))
        distance = np.dot((P - A), n) / np.linalg.norm(n)
        return distance

    def angle_to_object(self):
        index_min = np.argmin(self.laser_ranges)
        angle = (index_min * pi / 180.0) - pi
        return angle

    def heading_difference(self, start_point, end_point):
        line = np.array(end_point) - np.array(start_point)
        robot_heading = np.array([np.cos(self.orientation.z), np.sin(self.orientation.z)])
        angle = acos(np.dot(robot_heading, line) / (np.linalg.norm(line) * np.linalg.norm(robot_heading)))
        cross = np.cross(line, robot_heading)
        if cross[2] > 0.0:
            angle = -angle
        return angle

    def main_loop(self):
        if self.position is None or self.laser_ranges is None:
            return

        while self.distance_to(GOAL_POINT) > OBJECT_HIT_DIST and not any(r < OBJECT_HIT_DIST for r in self.laser_ranges):
            x_track_error = self.distance_to_line(self.leave_point, GOAL_POINT)
            heading_error = self.heading_difference(self.leave_point, GOAL_POINT)
            self.pure_pursuit(x_track_error, heading_error)
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.distance_to(GOAL_POINT) < OBJECT_HIT_DIST:
            self.get_logger().info('Goal reached')
            rclpy.shutdown()

        hit_location = self.get_my_location()
        returning_to_hit = False

        while self.distance_to(GOAL_POINT) > OBJECT_HIT_DIST and not self.hit_location_reencountered:
            angle = self.angle_to_object()
            dist = min(self.laser_ranges)
            self.pure_pursuit(OBJECT_FOLLOW_DIST - dist, angle - pi / 2)

            current_goal_distance = self.distance_to(GOAL_POINT)
            if self.distance_to(self.leave_point) < OBJECT_HIT_DIST:
                self.leave_point = self.get_my_location()

            if not returning_to_hit and self.distance_to(hit_location) > OBJECT_HIT_DIST:
                returning_to_hit = True

            if self.is_on_mline() <= thresholdForLine and returning_to_hit:
                self.hit_location_reencountered = True
                self.get_logger().info("On M-line")

        while self.distance_to(self.leave_point) > OBJECT_HIT_DIST:
            angle = self.angle_to_object()
            dist = min(self.laser_ranges)
            self.pure_pursuit(OBJECT_FOLLOW_DIST - dist, angle - pi / 2)

def main(args=None):
    rclpy.init(args=args)
    controller = Bug2Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()