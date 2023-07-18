import rclpy
from rclpy.node import Node
import tf2_geometry_msgs

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from auv_app.utils import *


class ImuCorrector(Node):
    def __init__(self):
        super().__init__("imu_corrector")
        self.imuSub = self.create_subscription(Imu, "/imu", self.imu_sub_cb, 10)
        self.imuCorrectPub = self.create_publisher(Imu, "/imu_correct", 10)

    def imu_sub_cb(self, msg: Imu):
        # rotate by the orientation to assume flat on earth
        msg.linear_acceleration = tf2_geometry_msgs.do_transform_vector3(
            stampVector3(msg.header, msg.linear_acceleration),
            stampTransform(msg.header, transformFromQuaternion(msg.orientation)),
        ).vector
        # remove gravity term
        msg.linear_acceleration.z -= 9.79999999999996
        # pub
        self.imuCorrectPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    imu_corrector = ImuCorrector()

    rclpy.spin(imu_corrector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_corrector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
