import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros.base import SENSOR_QOS
import math

class QuaternionToEulerNode(Node):
    def __init__(self):
        super().__init__('quaternion_to_euler_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            SENSOR_QOS
        )
        self.subscription  # prevent unused variable warning

    def quaternion_to_euler(self, x, y, z, w):
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

    def pose_callback(self, msg):
        """
        Callback function for the PoseStamped subscriber

        Parameters:
        msg -- the received PoseStamped message
        """
        orientation = msg.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)

        print("Roll: "+str(roll*180/math.pi)+" Pitch: "+str(pitch*180/math.pi)+" Yaw: "+str(yaw*180/math.pi))


def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToEulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

