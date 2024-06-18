import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import NavigateToPose
from geometry_msgs.msg import PoseStamped
import action_msgs.msg

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            'coordinates_topic',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.nav_client = self.create_client(NavigateToPose, 'navigate_to_pose')
        while not self.nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Navigation service not available, waiting again...')
        self.request = NavigateToPose.Request()

    def callback(self, msg):
        self.get_logger().info('Received coordinates: %f, %f' % (msg.pose.position.x, msg.pose.position.y))
        self.request.pose = msg
        future = self.nav_client.call_async(self.request)
        future.add_done_callback(self.navigation_callback)

    def navigation_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                if response.result == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info('Navigation succeeded!')
                else:
                    self.get_logger().warn('Navigation failed with result: %d' % response.result)
            else:
                self.get_logger().warn('Failed to get response from navigation action server')
        except Exception as e:
            self.get_logger().error('Navigation action failed: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
