import rclpy
from rclpy.node import Node
import tf2_geometry_msgs

from sensor_msgs.msg import Imu

from robot_app.utils import *


class ImuTuner(Node):
    def __init__(self):
        super().__init__("imu_tuner")
        self.imuSub = self.create_subscription(Imu, "imu", self.imu_sub_cb, 10)
        self.imuTunedPub = self.create_publisher(Imu, "imu_tuned", 10)

    def imu_sub_cb(self, msg: Imu):
        # pub imu
        msg.header.stamp = self.get_clock().now().to_msg()

        # rotate gravity by the orientation and remove that from linAcc
        # gravity = Vector3()
        # gravity.z = 9.79999999999996
        # gravity = tf2_geometry_msgs.do_transform_vector3(
        #     stampVector3(Header(), gravity),
        #     stampTransform(Header(), transformFromQuaternion(msg.orientation)),
        # ).vector
        # msg.linear_acceleration = minus(msg.linear_acceleration, gravity)

        # rotate by the inverse of the orientation to assume flat on earth
        # invOrientation = deepcopy(msg.orientation)
        # invOrientation.w *= 1
        # msg.linear_acceleration = tf2_geometry_msgs.do_transform_vector3(
        #     stampVector3(msg.header, msg.linear_acceleration),
        #     stampTransform(msg.header, transformFromQuaternion(invOrientation)),
        # ).vector
        # remove gravity term
        # msg.linear_acceleration.z -= 9.79999999999996
        self.imuTunedPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImuTuner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
