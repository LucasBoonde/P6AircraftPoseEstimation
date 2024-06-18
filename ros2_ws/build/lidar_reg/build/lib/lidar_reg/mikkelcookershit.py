import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from collections import deque
import numpy as np
import math
import time
import paho.mqtt.client as mqtt
import json

# Global variables
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84 / 2
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.02
goalDist = 0.05

# Deque for waypoints
waypoint_queue = deque()

# Callback function for MQTT message
def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        latitude = payload['latitude']
        longitude = payload['longitude']
        waypoint_queue.append([latitude, longitude])
        print(f"Received and added waypoint: {latitude}, {longitude}")
    except Exception as e:
        print(f"Error processing message: {e}")

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
    return input_vel

def quaternion_to_radians(w, x, y, z):
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    return yaw

def transform_to_local_frame(x_g, y_g, robot_x, robot_y, rotation_z):
    delta_x = x_g - robot_x
    delta_y = y_g - robot_y
    cos_theta = np.cos(rotation_z)
    sin_theta = np.sin(rotation_z)
    x_l = (delta_x * cos_theta + delta_y * sin_theta)
    y_l = (-delta_x * sin_theta + delta_y * cos_theta)
    return [x_l, y_l]

def reg_ang(target_x, target_y, orientation, robotx, roboty, angvel):
    k = 1
    pD = 100 / np.deg2rad(180)
    avP = 0.025
    localWP = transform_to_local_frame(target_x, target_y, robotx, roboty, orientation)
    ang_global = math.atan2(localWP[1], localWP[0])
    err = ang_global
    angVelRes = k * err * pD * avP
    return constrain(angVelRes, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

def reg_vel(target_x, target_y, robotx, roboty):
    standard_vel = 0.11
    dist = np.linalg.norm([target_x - robotx, target_y - roboty])
    error = -0.25 + dist
    stepchange = 0.1476851852 * error + standard_vel
    return constrain(stepchange, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

class Imu_Sub(Node):
    def __init__(self):
        super().__init__('Imu_Sub')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.orientation = 0.0
        self.angvel = 0.0

    def listener_callback(self, msg: Imu):
        self.orientation = quaternion_to_radians(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        self.angvel = msg.angular_velocity.z

class Point_Sub(Node):
    def __init__(self):
        super().__init__('Point_Sub')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.x = 0.0
        self.y = 0.0

    def listener_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

def main():
    rclpy.init()
    node = rclpy.create_node('drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    Imu_node = Imu_Sub()
    pose_node = Point_Sub()

    twist = Twist()

    # MQTT subscriber setup
    mqtt_client = mqtt.Client()
    mqtt_client.on_message = on_message
    mqtt_client.connect("broker.hivemq.com", 1883, 60)
    mqtt_client.subscribe("coordinates/topic")
    mqtt_client.loop_start()


    print("Waiting for initial waypoints...")
    while not waypoint_queue:
        time.sleep(0.1)

    print("Waiting for initial sensor data...")
    for _ in range(10):
        rclpy.spin_once(pose_node)
        rclpy.spin_once(Imu_node)
        time.sleep(0.1)

    print("Starting main control loop...")
    while rclpy.ok():
        rclpy.spin_once(pose_node)
        rclpy.spin_once(Imu_node)

        if len(waypoint_queue) != 0:
            currentWP = waypoint_queue.popleft()
            distToWP = 1
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            pub.publish(twist)

        while distToWP > goalDist:
            distToWP = np.linalg.norm([currentWP[0] - pose_node.x, currentWP[1] - pose_node.y])

            rclpy.spin_once(pose_node)
            rclpy.spin_once(Imu_node)

            if distToWP <= goalDist:
                print("Waypoint reached. Pausing for 2 seconds.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
                time.sleep(2)
                if len(waypoint_queue) > 0:
                    currentWP = waypoint_queue.popleft()
                    distToWP = 1
                continue

            if len(waypoint_queue) == 0:
                lastWP = currentWP
            else:
                lastWP = waypoint_queue.popleft() 
                waypoint_queue.appendleft(lastWP)

            print(f"Distance to WP: {distToWP}")
            print(f"Current Waypoint: {currentWP[0]}, {currentWP[1]}")
            print(f"Current Position: {pose_node.x}, {pose_node.y}")
            print(f"Current Orientation: {Imu_node.orientation}")

            control_linear_velocity = reg_vel(lastWP[0], lastWP[1], pose_node.x, pose_node.y)
            control_angular_velocity = reg_ang(currentWP[0], currentWP[1], Imu_node.orientation, pose_node.x, pose_node.y, Imu_node.angvel)
                    
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                0.0,  # target_linear_velocity
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity / 2

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                0.0,  # target_angular_velocity
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.z = math.atan2(math.sin(control_angular_velocity), math.cos(control_angular_velocity))

            print(f"Control_Angular_velocity: {control_angular_velocity}")
            pub.publish(twist)
            print(f"Publishing Twist: {twist}")

    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    Imu_node.destroy_node()
    pose_node.destroy_node()
    rclpy.shutdown()       

if __name__ == '__main__':
    main()
