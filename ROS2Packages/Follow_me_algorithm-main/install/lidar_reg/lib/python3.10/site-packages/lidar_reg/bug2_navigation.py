import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TurtlebotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        
        # Publisher and message
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.velocity = Twist()

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Goal position
        self.goal_x = 1.0
        self.goal_y = 0.0

        # Current position and orientation (to be filled by odom_callback)
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_z = msg.pose.pose.orientation.z  # assumes 2D (planar) motion
        
        # Call the controller method
        self.move_robot()

    def move_robot(self):
        distance_to_goal = ((self.goal_x-self.position_x) **2 + (self.goal_y-self.position_y)**2)**0.5
        if distance_to_goal > 0.1:
            self.velocity.linear.x = 0.5
        else:
            self.velocity.linear.x = 0.0
        self.publisher_.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()