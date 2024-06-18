import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import action_msgs.msg

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(PoseStamped, 'coordinates_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_coordinates)
        self.counter = 0

    def publish_coordinates(self):
        msg = PoseStamped()
        # Modify these coordinates as per your requirement
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Publishing coordinates: %f, %f' % (msg.pose.position.x, msg.pose.position.y))

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
