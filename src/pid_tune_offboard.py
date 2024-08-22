#################################################################
# Ros related libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros.base import SENSOR_QOS
#################################################################
# General python libraries
import numpy as np
import math
import time
import signal
import threading
import quaternion
#################################################################
# Extras
from simple_pid import PID
from xbox360controller import Xbox360Controller



class PositionController(Node):
    #############################################################
    # Initializer for ROS2 node
    #############################################################
    def __init__(self):
        super().__init__('pos_controller')
        #########################################################
        # Initialize PID controller
        #########################################################
        kp=0.6 #0.6, 0.05, 0.9 seems very good for small frame drone
        ki=0.05 #
        kd=0.9
        x_output_limit = 1.0
        y_output_limit = 1.0
        z_output_limit = 0.5
        yaw_output_limit = 1.0

        self.pid_x = PID(kp, ki, kd, setpoint=0.0,output_limits=(-x_output_limit,x_output_limit))
        self.pid_y = PID(kp, ki, kd, setpoint=0.0,output_limits=(-y_output_limit,y_output_limit))
        self.pid_z = PID(0.3, 0.0, 0.1, setpoint=0.0,output_limits=(-z_output_limit,z_output_limit))
        self.pid_yaw = PID(kp, ki, kd, setpoint=0.0, output_limits=(-yaw_output_limit,yaw_output_limit))


        #########################################################
        # Initialize subscriber to vision topic
        #########################################################
        self.sub_pose = self.create_subscription(
            PoseStamped,
           '/mavros/local_position/pose',
            self.pose_callback,
           SENSOR_QOS
        )
        #'/vrpn_mocap/Drone_JK/pose'
        #########################################################
        # Initialize publisher for mavros velocity commands
        #########################################################
        self.pub_vel = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            0
        )

        #########################################################
        # Create a timer for the publishser. PX4 expects 10 Hz
        #########################################################
        self.pub_timer = self.create_timer(0.05, self.compute_control)

        #########################################################
        # Initialize variables
        #########################################################
        self.current_pose = PoseStamped()
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 2.0
        self.desired_yaw = 0.0
        self.joystick_dict = {'button_a': False, 'button_b': False, 'button_x': False, 'button_y': False, 'axis_r': (0.0,0.0), 'axis_l': (0.0,0.0), 'hat':(0.0,0.0)}
        self.prev_time = time.time()

        self.angle_increment_time = 0.10
        self.circle_angle = 0.0
        self.circle_radius = 2
        self.angle_step = np.pi / 80

        self.square_length = 1.0



        #########################################################
        # Initialize threads
        #########################################################
        self.joystick_thread = threading.Thread(target=self.joystick_thread_function)
        self.joystick_thread.start()

        self.angle_thread = threading.Thread(target=self.circle_angle_thread)
        self.angle_thread.start()


    #############################################################
    # Save the pose received from OptiTrack as self.current_pose
    #############################################################
    def pose_callback(self, msg):
        self.current_pose = msg


    #############################################################
    # Control functions -> Currently PID
    #############################################################
    def compute_control(self):
        #########################################################
        # Compute the desired positions - Square
        #########################################################
        # # For this example the directional keys can be used to change point in a square
        if self.joystick_dict['hat'] == (0,1):
            self.desired_x = self.square_length
            self.desired_y = 0
            self.desired_z = 2.0
            self.desired_yaw = math.atan2(self.desired_y-0.5, self.desired_x-0.5) + math.pi
        elif self.joystick_dict['hat'] == (-1,0):
            self.desired_x = self.square_length
            self.desired_y = self.square_length
            self.desired_z = 2.0
            self.desired_yaw = math.atan2(self.desired_y-0.5, self.desired_x-0.5) + math.pi
        elif self.joystick_dict['hat'] == (0,-1):
            self.desired_x = 0
            self.desired_y = self.square_length
            self.desired_z = 2.0
            self.desired_yaw = math.atan2(self.desired_y-0.5, self.desired_x-0.5) + math.pi
        elif self.joystick_dict['hat'] == (1,0):
            self.desired_x = 0
            self.desired_y = 0
            self.desired_z = 2.0
            self.desired_yaw = math.atan2(self.desired_y-0.5, self.desired_x-0.5) + math.pi

        #########################################################
        # Compute the desired positions - Circle
        #########################################################
        '''self.desired_x = self.circle_radius * math.cos(self.circle_angle)
        self.desired_y=self.circle_radius * math.sin(self.circle_angle)
        self.desired_z=2.0
        self.desired_yaw = math.atan2(self.desired_y - 1, self.desired_x - 1) + math.pi
        self.desired_yaw = 0.0'''

        #########################################################
        # Get the Quaternion from OptiTrack
        #########################################################
        q = np.quaternion(1,0,0,0)
        q.x = self.current_pose.pose.orientation.x
        q.y = self.current_pose.pose.orientation.y
        q.z = self.current_pose.pose.orientation.z
        q.w = self.current_pose.pose.orientation.w

        #########################################################
        # Get rotation vector from the Quaternion
        #########################################################
        rotation_vector = quaternion.as_rotation_vector(q)

        #########################################################
        # Compute the PID errors
        #########################################################
        error_x = self.desired_x - self.current_pose.pose.position.x
        error_y = self.desired_y - self.current_pose.pose.position.y
        error_z = self.desired_z - self.current_pose.pose.position.z
        error_yaw = self.desired_yaw - rotation_vector[2]


        #########################################################
        # Correct the yaw error
        #########################################################
        if (error_yaw < -math.pi):
            error_yaw += 2 * math.pi
        if (error_yaw > math.pi):
            error_yaw -= 2 * math.pi

        #########################################################
        # Compute PID outputs
        #########################################################
        vel_x = -self.pid_x(error_x)
        vel_y = -self.pid_y(error_y)
        vel_z = -self.pid_z(error_z)
        vel_yaw = -self.pid_yaw(error_yaw)

        #########################################################
        # Publish velocity commands as mavros velocity setpoints
        #########################################################
        cmd_vel_msg = TwistStamped()
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.twist.linear.x = vel_x
        cmd_vel_msg.twist.linear.y = vel_y
        cmd_vel_msg.twist.linear.z = vel_z
        cmd_vel_msg.twist.angular.z = vel_yaw
        self.pub_vel.publish(cmd_vel_msg)

        print('x: ', self.desired_x, ' y: ', self.desired_y, ' z: ', self.desired_z, "yaw:", self.desired_yaw)
        #print(self.joystick_dict)



    #############################################################
    # Thread handling joystick inputs
    #############################################################
    def joystick_thread_function(self):
        while True:
            with Xbox360Controller(0, axis_threshold=0.2) as controller:
                #################################################
                # Add the button functions to all buttons
                #################################################
                for b in controller.buttons:
                    b.when_pressed = self.on_button_pressed
                    b.when_released = self.on_button_released
                for a in controller.axes:
                    a.when_moved = self.on_axis_moved

                signal.pause()

    #############################################################
    # Helper functions for Joystick
    #############################################################
    def on_button_pressed(self, button):
        self.joystick_dict[button.name] = True

    def on_button_released(self, button):
        self.joystick_dict[button.name] = False

    def on_axis_moved(self, axis):
        try:
            self.joystick_dict[axis.name] = axis.value
        except:
            self.joystick_dict[axis.name] = (axis.x, axis.y)


    #############################################################
    # Angle thread for circle movement
    #############################################################
    def circle_angle_thread(self):
        while True:
            if time.time() - self.prev_time > self.angle_increment_time:
                self.circle_angle=self.circle_angle + self.angle_step
                self.prev_time = time.time()


#################################################################
# Main functions -> Start ROS node
#################################################################
def main(args=None):
    # Start rclpy
    rclpy.init(args=args)
    # Create positions controller node
    pos_controller = PositionController()
    rclpy.spin(pos_controller)
    # Clean up
    pos_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

