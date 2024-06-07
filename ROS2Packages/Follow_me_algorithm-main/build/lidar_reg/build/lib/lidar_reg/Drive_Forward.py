import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from math import cos, sin, pi, sqrt
import math

class Odometry:
    def __init__(self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius  # Radius of the robot's wheels (in meters)
        self.wheel_base = wheel_base      # Distance between the robot's wheels (in meters)
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0
        self.x = 0.0    # Robot's x-coordinate (in meters)
        self.y = 0.0    # Robot's y-coordinate (in meters)
        self.theta = 0.0  # Robot's orientation (in radians)

    def update(self, left_encoder, right_encoder):
        delta_left = (left_encoder - self.prev_left_encoder) * self.wheel_radius
        delta_right = (right_encoder - self.prev_right_encoder) * self.wheel_radius
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder
        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base
        self.x += delta_distance * cos(self.theta + delta_theta / 2.0)
        self.y += delta_distance * sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta = (self.theta + pi) % (2 * pi) - pi

    def get_pose(self):
        return self.x, self.y, self.theta

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.odometry = Odometry(wheel_radius=0.033, wheel_base=0.160)  # TurtleBot3 Burger specs
        self.drive_speed = 0.2  # Linear velocity in m/s

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.publisher = self.create_publisher(Float32MultiArray, 'targetCoordinates', 1)
        self.timer = self.create_timer(0.1, self.update_odometry_and_drive)  # Update odometry and drive forward

    def lidar_callback(self, msg):
    # Process Lidar data here
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        time_increment = msg.time_increment
        
        # Print the data
        print('Ranges: ', ranges)
        print('Minimum angle: ', angle_min)
        print('Maximum angle: ', angle_max)
        print('Angle increment: ', angle_increment)
        print('Time increment: ', time_increment)
        pass

    def update_odometry_and_drive(self):
        # Simulate wheel encoder readings (replace with actual encoder values)
        left_encoder, right_encoder = self.read_encoders()

        # Update odometry based on wheel encoders
        self.odometry.update(left_encoder, right_encoder)

        # Check the front-facing Lidar distance
        current_distance = self.get_front_lidar_distance()

        if current_distance <= 0.2:
            self.get_logger().info(f"Stopping - Reached desired distance: {current_distance} meters")
            # Stop the robot (e.g., by sending zero velocity commands)
            # Implement stopping logic here
        else:
            self.get_logger().info(f"Driving forward - Current distance: {current_distance} meters")
            # Drive forward at a fixed speed
            # Implement forward driving logic here

    def read_encoders(self):
        # Simulated function to read encoder values (replace with actual code)
        left_encoder = 100  # Example left wheel encoder reading (ticks or units)
        right_encoder = 110  # Example right wheel encoder reading (ticks or units)
        return left_encoder, right_encoder

    def get_front_lidar_distance(self):
        # Simulated function to get front-facing Lidar distance (example)
        # In a real implementation, this would read from actual Lidar data
        return 0.3  # Example: returns a fixed distance of 0.3 meters

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()