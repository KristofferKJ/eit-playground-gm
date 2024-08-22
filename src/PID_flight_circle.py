import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from simple_pid import PID
from mavros.base import SENSOR_QOS
from math import cos,sin,sqrt
import math
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_circle')

        kp=0.19
        ki=0.00025
        kd=0.15
        # Initialize PID controller
        self.pid_x = PID(kp, ki, kd, setpoint=0.0,output_limits=(-0.5,0.5))
        self.pid_y = PID(kp, ki, kd, setpoint=0.0,output_limits=(-0.5,0.5))
        self.pid_z = PID(0.3, 0.0, 0.1, setpoint=0.0,output_limits=(-0.5,0.5))
        # Initialize ROS publishers and subscribers
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.pose_callback,
           SENSOR_QOS
        )
        self.pub_vel = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            0
        )

        # Initialize variables
        self.current_pose = PoseStamped()

        self.height=3
        self.radious=2
        self.wp_n=50
        self.angle_step=2*math.pi/self.wp_n
        self.current_angle=0

        self.initial_pose=[self.radious*cos(self.current_angle),self.radious*sin(self.current_angle),self.height]
        self.moving2initial_pose=True

        print("Initial Pose: "+str(self.initial_pose))



        self.current_wp=0

        self.error_acceptance=0.20
        self.timer=1
        self.goal_time=time.time()+self.timer

        # Calculated required velocity for current number of waypoints and delay

        polygon_side_lenght=2*self.radious*sin(math.pi/self.wp_n)
        expected_speed=polygon_side_lenght/self.timer


        print("Expected veolocity: "+str(expected_speed))

        if expected_speed>sqrt(pow(0.5,2)+pow(0.5,2)):
            print("WARNING: Expected veolocity ("+str(expected_speed)+") grater than maximum linear velocity ("+str( sqrt(pow(self.maxvelx,2)+pow(0.5,2)))+")")




    def deg2rad(self,deg):
        return math.pi*deg/180

    def rad2deg(self,rad):
        return 180*rad/math.pi

    def pose_callback(self, msg):
        self.current_pose = msg

    def compute_control(self):

        if self.moving2initial_pose: # Move to initial circle point
            desired_x=self.initial_pose[0]
            desired_y=self.initial_pose[1]
            desired_z=self.initial_pose[2]

            error_x = desired_x - self.current_pose.pose.position.x
            error_y = desired_y - self.current_pose.pose.position.y
            error_z = desired_z - self.current_pose.pose.position.z

            if abs(error_x)<self.error_acceptance and  abs(error_y)<self.error_acceptance and  abs(error_z)<self.error_acceptance:
                self.moving2initial_pose=False
                print("Initial pose reached")

        else: # Circle movement

            # Compute errors
            desired_x=self.radious*cos(self.current_angle)
            desired_y=self.radious*sin(self.current_angle)
            desired_z=self.height

            error_x = desired_x - self.current_pose.pose.position.x
            error_y = desired_y - self.current_pose.pose.position.y
            error_z = desired_z - self.current_pose.pose.position.z

            # Log circle progress
            if self.goal_time<time.time():
                self.current_angle=self.current_angle+self.angle_step
                print("Current angle: "+str(self.rad2deg(self.current_angle) % 360))
                self.goal_time=time.time()+self.timer



        # Compute PID outputs
        vel_x = -self.pid_x(error_x)
        vel_y = -self.pid_y(error_y)
        vel_z = -self.pid_z(error_z)

        # Publish velocity commands
        cmd_vel_msg = TwistStamped()
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.twist.linear.x = vel_x
        cmd_vel_msg.twist.linear.y = vel_y
        cmd_vel_msg.twist.linear.z = vel_z
        self.pub_vel.publish(cmd_vel_msg)

    def run(self):

        while rclpy.ok():
            self.compute_control()
            rclpy.spin_once(self)



def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    pid_controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
