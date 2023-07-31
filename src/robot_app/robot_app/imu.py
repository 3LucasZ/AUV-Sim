import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu as ImuMsg
from robot_app.utils import *


class Imu(Node):
    def __init__(self):
        super().__init__("imu")

        # set transform base_link -> imu_sensor
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_sensor"
        self.tf_static_broadcaster.sendTransform(t)

        # imu_sim sub
        self.imuSimSub = self.create_subscription(
            ImuMsg, "/imu_sim", self.imu_sim_sub_cb, 10
        )

        # imu pub
        self.imuPub = self.create_publisher(ImuMsg, "/imu", 10)

    def imu_sim_sub_cb(self, msg: ImuMsg):
        # correct header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_sensor"
        self.imuPub.publish(msg)


# spin
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Imu())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
