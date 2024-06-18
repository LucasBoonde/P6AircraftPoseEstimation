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
        self.subscription  # prevent unused variable warning
        self.received_data = None

    def listener_callback(self, msg):
        self.get_logger().info('Received coordinates: "%s"' % str(msg.data))
        self.received_data = msg.data
        # Add your processing code here
        self.process_coordinates(self.received_data)

    def process_coordinates(self, data):
        # Example processing code
        coordinates_list = np.array(data).reshape(-1, 3)
        print("Processed coordinates list:")
        print(coordinates_list)
        # If you only want to receive the data once and then shut down
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    xyz_subscriber = XYZSubscriber()

    try:
        rclpy.spin(xyz_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        xyz_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

