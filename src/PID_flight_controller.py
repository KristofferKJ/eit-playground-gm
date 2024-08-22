import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from simple_pid import PID
from mavros.base import SENSOR_QOS

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        kp=0.2
        ki=0.00025
        kd=0.15
        # Initialize PID controller
        self.pid_x = PID(kp, ki, kd, setpoint=0.0,output_limits=(-0.5,0.5))
        self.pid_y = PID(kp, ki, kd, setpoint=0.0,output_limits=(-0.5,0.5))
        self.pid_z = PID(0.3, ki, 0.1, setpoint=0.0,output_limits=(-0.5,0.5))
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
            1
        )

        # Initialize variables
        self.current_pose = PoseStamped()


        self.desired_pose=[]
        self.desired_pose.append([2.0,0.0,3.0])
        self.desired_pose.append([2.0,2.0,3.0])
        self.desired_pose.append([-2.0,2.0,3.0])
        self.desired_pose.append([-2.0,0.0,3.0])
        self.desired_pose.append([2.0,0.0,3.0])


        # self.desired_pose.append([1.5,1.5,2.0])
        # self.desired_pose.append([-1.2,2.0,2.0])
        # self.desired_pose.append([-1.36,-1.0,2.0])
        # self.desired_pose.append([1.29,-1.29,2.0])
        # self.desired_pose.append([1.5,1.5,2.0])


        self.n_wp=len(self.desired_pose)
        self.current_wp=0

        self.maxvelx=0.5
        self.maxvely=0.5
        self.maxvelz=0.5

        self.error_acceptance=0.25

    def pose_callback(self, msg):
        self.current_pose = msg

    def compute_control(self):
        # Compute errors
        desired_x=self.desired_pose[self.current_wp][0]
        desired_y=self.desired_pose[self.current_wp][1]
        desired_z=self.desired_pose[self.current_wp][2]

        error_x = desired_x - self.current_pose.pose.position.x
        error_y = desired_y - self.current_pose.pose.position.y
        error_z = desired_z - self.current_pose.pose.position.z

        # check for waipoint reached
        if abs(error_x)<self.error_acceptance and  abs(error_y)<self.error_acceptance and  abs(error_z)<self.error_acceptance:
            if self.current_wp<self.n_wp-1:
                print("Waypoint "+str(self.current_wp)+ " reached")
                self.current_wp=self.current_wp+1
                return
            else:
                print("Path completed")



        # Compute PID outputs
        vel_x = -self.pid_x(error_x)
        vel_y = -self.pid_y(error_y)
        vel_z = -self.pid_z(error_z)

        # saturate
        if vel_x>self.maxvelx:
            vel_x=self.maxvelx
        elif vel_x<-self.maxvelx:
            vel_x=-self.maxvelx

        if vel_y>self.maxvely:
            vel_y=self.maxvely
        elif vel_y<-self.maxvely:
            vel_y=-self.maxvely

        if vel_z>self.maxvelz:
            vel_z=self.maxvelz
        elif vel_z<-self.maxvelz:
            vel_z=-self.maxvelz

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
