import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import math

#------------CLASSES----------------
class LIDARsubscriber(Node):
    def __init__(self):
        super().__init__("LIDAR_subscriber")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription
    
    def listener_callback(self,msg:LaserScan):
        self.dist = msg.ranges
        for i in range(len(self.dist)):
            if self.dist[i] > 0.3:
                self.dist[i] = 0.0
            if self.dist[i] < 0.1:
                self.dist[i] = 0.0

class WaypointPublisher(Node):
    def __init__(self,coordinates):
        self.coordinates = coordinates
        super().__init__('targetCoordinate_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'targetCoordinates', 1)
        timer_period = 0.33  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)

    def publish_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.coordinates[0],self.coordinates[1]]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: ({msg.data})")

#-----------FUNCTIONS---------------
def main(args=None):
    rclpy.init(args=args)

    #Make subscriber and lists for x and y coordinates of point cloud
    lidar_subscriber = LIDARsubscriber()
    coord_publisher = WaypointPublisher(coordinates=[0.0,0.0])

    while(1):
        # Fill lidar data array
        rclpy.spin_once(lidar_subscriber)
        points = np.asarray(lidar_subscriber.dist)
        
        # Set coordinates for publish x1 & y1
        coord_publisher.coordinates[0] = 0.5755
        coord_publisher.coordinates[1] = y1
                
        # Publish
        rclpy.spin_once(coord_publisher)

        # Reset coordinates
        coord_publisher.coordinates[0] = 0.0
        coord_publisher.coordinates[1] = 0.0
        
    coord_publisher.destroy_node()
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()