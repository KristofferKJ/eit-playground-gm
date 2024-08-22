#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped

from mavros.base import SENSOR_QOS
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Quaternion
import math

def quaternion_multiply( q1, q2):
        # Perform quaternion multiplication
        q = Quaternion()
        q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return q


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)

    Parameters:
    x, y, z, w -- the four components of the quaternion

    Returns:
    roll, pitch, yaw -- the euler angles in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class RelayNode(Node):

    def __init__(self):
        super().__init__('relay')

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=0
        )
        self.sensor_qos = SENSOR_QOS
        self.sensor_qos.depth = 0
        self.subscription = self.create_subscription(PoseStamped,'/vrpn_mocap/Drone_JK/pose',self.listener_callback, self.sensor_qos)#self.qos_profile)
        self.subscription  # prevent unused variable warning

        # self.publisher = self.create_publisher(PoseStamped, '/mavros/mocap/pose', SENSOR_QOS)
        self.publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 0)#lf.qos_profile)#SENSOR_QOS)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def listener_callback(self, msg):
        msg.header.frame_id = "map"
        #self.publisher.publish(msg)
        # Create a new PoseStamped message
#'''
        corrected_message = PoseStamped()

        # Populate the new message with data from the received message
        '''corrected_message.header = msg.header
        corrected_message.pose.position.x = msg.pose.position.z
        corrected_message.pose.position.y = msg.pose.position.x
        corrected_message.pose.position.z = msg.pose.position.y

        corrected_message.pose.orientation.x = msg.pose.orientation.z
        corrected_message.pose.orientation.y = msg.pose.orientation.x
        corrected_message.pose.orientation.z = msg.pose.orientation.y
        corrected_message.pose.orientation.w = msg.pose.orientation.w'''

        corrected_message = msg

        x = corrected_message.pose.orientation.x
        y = corrected_message.pose.orientation.y
        z = corrected_message.pose.orientation.z
        w = corrected_message.pose.orientation.w

        roll,pitch,yaw=quaternion_to_euler(x, y, z, w)

        print("x: "+str(corrected_message.pose.position.x)+" y: "+str(corrected_message.pose.position.y)+" z: "+str(corrected_message.pose.position.z))

        time.sleep(0.05)
        self.publisher.publish(corrected_message)
#'''



def main(args=None):
    rclpy.init(args=args)

    relay_node = RelayNode()

    rclpy.spin(relay_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
