#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offboard_ctrl.py: Controlling the setpoints

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
import math

###############################################
# ROS Imports                                 #
###############################################
import rclpy
from rclpy.node import Node

###############################################
# ROS Topic messages                          #
###############################################
from std_msgs.msg import String
from std_msgs.msg import Float32

from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State, RCOut


###############################################
# ROS Service messages                        #
###############################################


###############################################
# Offboad Control class                       #
###############################################
class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_ctrl')
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg = PoseStamped()
        self.msg.header.frame_id = "map"

    def timer_callback(self):

        #x = 1 * math.cos(self.i*2*math.pi/360)
        #y = 1 * math.sin(self.i*2*math.pi/360)
        x = 0.0
        y = 0.0
        z = 2.0

        self.msg.pose.position.x = x
        self.msg.pose.position.y = y
        self.msg.pose.position.z = z


        self.i = self.i + 1

        # Reset counter
        if(self.i > 360):
            self.i = 0
        
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s"' % self.msg)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    offb_ctrl = OffboardControl()

    rclpy.spin(offb_ctrl)
    offb_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
