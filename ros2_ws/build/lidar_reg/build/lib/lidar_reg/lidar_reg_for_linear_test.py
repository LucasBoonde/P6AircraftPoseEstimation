import os
import select
import sys
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from math import pi
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from std_msgs.msg import String
#from rclpy.qos import QosProfile, QosReliabilityPolicy
from rclpy.qos import ReliabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from collections import deque
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy


#Firstly choosing operating system: 
import termios
import tty

#-----------------Control Constants----------------
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84/2

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.02

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
message = """The robot will drive now"""
error = """AN ERROR HAS ACCURED"""

# Deque size limit
queueLimit = 10
goalDist = 0.1
#object_detection_threshold = 0.2
#qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)


#--------------FUNCTIONS------------------
def make_simple_profile(output, input, step):
    if input > output:
        output = min(input, output + step)
    elif input < output:
        output = max(input, output - step)
    else:
        output = input
    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

def quaternion_to_radians(w, x, y, z):
    # Calculate the yaw angle
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return yaw

def transform_to_local_frame(x_g, y_g, robot_x, robot_y, rotation_z):
    # Calculate the inverse transformation
    delta_x = x_g - robot_x
    delta_y = y_g - robot_y
    cos_theta = np.cos(rotation_z)
    sin_theta = np.sin(rotation_z)

    x_l = (delta_x * cos_theta + delta_y * sin_theta)
    y_l = (-delta_x * sin_theta + delta_y * cos_theta)

    # Return the transformed point in the local frame
    return [x_l, y_l]

def transform_to_global_frame(x_l, y_l, robot_x, robot_y, rotation_z):
    # Apply the transformation
    x_g = robot_x + (x_l * np.cos(rotation_z)) - (y_l * np.sin(rotation_z))
    y_g = robot_y + (x_l * np.sin(rotation_z)) + (y_l * np.cos(rotation_z))

    # Return the transformed point in the global frame
    return [x_g, y_g]

def reg_ang(target_x, target_y, orientation, robotx, roboty, angvel): 
    k       = 1                       # Regulation constant 
    pD      = 100/np.deg2rad(180)     # Percentage of degrees - Indicated that when the reference angle is 180, the robot will turn at full speed
    avP     = 0.025                   # One percent of max angle velocity in rads

    #Translate global coordinate into local coordinates to get angle for regulation:
    localWP = transform_to_local_frame(target_x,target_y,robotx,roboty,orientation)

    # Find the angle from the origin to the waypoint in rads
    ang_global = math.atan2(localWP[1], localWP[0])
    
    err = ang_global 

    print(f"Error reg_ang: {err}")
        #u       = k*e           #Upscaled degree difference
        #pF      = u*pD          #Percentage boost
        #angVelRes    = avP*pF       #Change in angle velocity 

    angVelRes = k*err*pD*avP
    print(f"Prev Angle: {angVelRes}")
                
    return constrain(angVelRes,-BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL) 

def reg_vel(target_x, target_y, robotx, roboty): # FIX FIX FIX
    #This is made if the distance is 0.5 - 2 meters where the ideal length is 1.25.
    #For the none derived version check maple document. 
    standard_vel = 0.11 

    #dist = np.linalg.norm([target_x-robotx,target_y-roboty])
    dist = 1
    error = -0.25 + dist
    stepchange = 0.1476851852*error+standard_vel

    return constrain(stepchange,-BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

#-----------------CLASSES-------------
class Imu_Sub(Node):
    def __init__(self):
        super().__init__('Imu_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription 

    def listener_callback(self, msg: Imu):#We have our message AKA data 
        self.orientation = quaternion_to_radians(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        self.angvel = msg.angular_velocity.z

class Point_Sub(Node): #Subscriber that extracts global position data of the TB3
    def __init__(self):
        super().__init__('Point_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Odometry):#We have our message AKA data 
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
 
class LIDAR_Sub(Node): #Subscriber that extracts lidar range data
    def __init__(self):
        super().__init__('LIDAR_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg: LaserScan):
        self.lidar_ranges = msg.ranges
        self.front_ranges = msg.ranges [-45:45]
        #self.front_dist = msg.ranges[0]
        #print(f"FRONT DISTANCE READING: {self.front_dist}")
        self.lidar_angle_increment = msg.angle_increment
        self.lidar_min_angle = msg.angle_min
        self.lidar_max_angle = msg.angle_max
        

def main():
    # Initialize rclpy and create the twist data-class publisher
    
    rclpy.init()
    node = rclpy.create_node('drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # Instantiate all relevant nodes
    Imu_node = Imu_Sub()
    pose_node = Point_Sub()
    lidar_node = LIDAR_Sub() 

    #Set all values to 0.0 in setup
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    # Make a double ended queue for waypoints
    queue = deque()
    
    # Instantiate the twist data-class (Which is published with new movement data to make the robot move)
    twist = Twist()

    #queue.append([1.74,-0.27])
    #queue.append([2.34,-0.83])
    queue.append([1.0,0.0])

    queue.append([0.0,0.0])

    #------------------

    print("Entering First Loop")
    while True: 
        # Fill all subscriber nodes with live data
        rclpy.spin_once(pose_node)
        rclpy.spin_once(Imu_node)
        rclpy.spin_once(lidar_node)

        if len(queue) != 0:
            currentWP = queue.popleft()
            distToWP = 1
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            pub.publish(twist)
            print("Queue Empty, stopped driving.")

        #Break loop when the robot is 20 cm from the current waypoint:
        #def get_avoidance_turn(lidar_node):
        #    left_mean = np.mean(lidar_node.lidar_ranges[len(lidar_node.lidar_ranges)//2:])
        #    right_mean = np.mean(lidar_node.lidar_ranges[:len(lidar_node.lidar_ranges)//2])
        #    if left_mean < right_mean:
        #        return 'right'  
        #    else:
        #        return 'left' 

        while distToWP > goalDist:
            distToWP = np.linalg.norm([currentWP[0]-pose_node.x, currentWP[1]-pose_node.y])

            rclpy.spin_once(pose_node)
            rclpy.spin_once(Imu_node)
            rclpy.spin_once(lidar_node) 

            if distToWP <= goalDist:
                print("Waypoint reached. Pausing for 2 seconds.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
                time.sleep(1)
                if len(queue) > 0:
                    currentWP = queue.popleft()
                    distToWP = 1
                continue

            if len(queue) == 0:
                lastWP = currentWP
            else:
                lastWP = queue.popleft() 
                queue.appendleft(lastWP)

            print(f"Distance to WP: {distToWP}")
            print(f"Current Waypoint: {currentWP[0]}, {currentWP[1]}")
            print(f"Current Position: {pose_node.x}, {pose_node.y}")
            print(f"Current Orientation: {Imu_node.orientation}")


            control_linear_velocity = reg_vel(lastWP[0], lastWP[1], pose_node.x, pose_node.y)
            control_angular_velocity = reg_ang(currentWP[0], currentWP[1], Imu_node.orientation, pose_node.x, pose_node.y, Imu_node.angvel)

            #for value in lidar_node.front_ranges:
             #   if (value < 0.30) & (value != 0.0):
              #      print("Object detected")
               #     control_angular_velocity += 2.0
                #    control_linear_velocity = 0.0
    

            # Create a list of non-zero distances
            #non_zero_values = [value for value in lidar_node.front_ranges if 0.0 < value <= 1.5]
            #mean_distance = 0.0 
            # If we have at least one non-zero distance, calculate the mean
            #if non_zero_values:
             #   mean_distance = sum(non_zero_values) / len(non_zero_values)
              #  if mean_distance < 0.50:
               #     print("Object detected")
                #    control_angular_velocity += 2.0
                 #   control_linear_velocity = 0.0        

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity/2

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.z = math.atan2(math.sin(control_angular_velocity), math.cos(control_angular_velocity))
            
            #print(f"Mean_Distance: {mean_distance}")
            print(f"Control_Angular_velocity: {control_angular_velocity}")
            pub.publish(twist)
            print(f"Publishing Twist: {twist}")
            print(pub.qos_profile)        
    target_node.destroy_node()
    Imu_node.destroy_node()
    pose_node.destroy_node()
    rclpy.shutdown()      

if __name__ == '__main__':
    main()