#LISTENER
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class CmdVelListener(Node):

    def __init__(self):
        super().__init__('cmd_vel_listener')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)


        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        print('Received Twist Message: ')
        print('Linear Components: [%.2f, %.2f, %.2f]' % (msg.linear.x, msg.linear.y, msg.linear.z))
        print('Angular Components: [%.2f, %.2f, %.2f]' % (msg.angular.x, msg.angular.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_listener = CmdVelListener()
    rclpy.spin(cmd_vel_listener)
    cmd_vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()