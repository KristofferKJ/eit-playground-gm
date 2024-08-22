 
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

from mavros_msgs.msg import State, RCOut, SetMode, CommandBool


###############################################
# ROS Service messages                        #
###############################################


###############################################
# Offboad Control class                       #
###############################################
class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_ctrl')

        # Publisher for local setpoints
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)



        # Subscriber to MavRos state
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback)
        self.state = State()
        self.state_sub

        # Arming client
        self.arming_client = self.create_client(CommandBool,'/mavros/command/arming')
        # Set mode client
        self.setMode_client = self.create_client(SetMode, '/mavros/set_mode')


        # Start of program
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "map"
        self.pose_msg.position.x = 0
        self.pose_msg.position.y = 0
        self.pose_msg.position.z = 2

        self.setMode_offboard = SetMode()
        self.setMode_offboard.request.custom_mode = "OFFBOARD"

        self.setMode_land = SetMode()
        self.setMode_land.request.custom_node ="AUTO.LAND"

        self.arm_message = CommandBool()
        self.arm_message.request.value = "true"



    def state_callback(self, msg):
        self.state = msg.mode

def main(args=None):
    rclpy.init(args=args)

    offb_ctrl = OffboardControl()

    rclpy.spin(offb_ctrl)
    offb_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
