import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class XYZSubscriber(Node):
    def __init__(self):
        super().__init__('xyz_coordinates_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'xyz_coordinates',
            self.listener_callback,
            10)
        self.coordinates_list = None
        self.shutdown_flag = False

    def listener_callback(self, msg):
        self.get_logger().info('Received coordinates: "%s"' % str(msg.data))
        self.coordinates_list = np.array(msg.data).reshape(-1, 3)
        self.get_logger().info('Processed coordinates list: "%s"' % str(self.coordinates_list))
        # Set the shutdown flag and shutdown
        self.shutdown_flag = True
        self.get_logger().info('Coordinates received, shutting down.')

def main(args=None):
    rclpy.init(args=args)
    xyz_subscriber = XYZSubscriber()

    while not xyz_subscriber.shutdown_flag:
        rclpy.spin_once(xyz_subscriber)

    # After shutdown, access the coordinates list
    coordinates_list = xyz_subscriber.coordinates_list

    if coordinates_list is not None:
        print("Received coordinates list in main:")
        print(coordinates_list)
    else:
        print("No data received.")

    xyz_subscriber.destroy_node()

    print("HEJSA BABY")


    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
