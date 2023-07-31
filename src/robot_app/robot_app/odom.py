import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
from robot_app.utils import *


class Odom(Node):
    def __init__(self):
        super().__init__("odom")

        # set transform world -> odom
        self.tfStaticBroadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "odom"
        self.tfStaticBroadcaster.sendTransform(t)

        # set init transform odom -> base_link so that ekf can initialize
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "odom"
        # t.child_frame_id = "base_link"
        # self.tfStaticBroadcaster.sendTransform(t)

        # odom_sim sub
        self.odomSimSub = self.create_subscription(
            Odometry, "odom_sim", self.odom_sim_sub_cb, 10
        )

        # odom pub
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odomPub = self.create_publisher(Odometry, "odom", 10)

    def odom_sim_sub_cb(self, msg: Odometry):
        # pub odom
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        self.odomPub.publish(msg)

        # # set transform odom -> base_link
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "odom"
        # t.child_frame_id = "base_link"
        # t.transform = transformFromPose(msg.pose.pose)
        # # self.tf_broadcaster.sendTransform(t)


# spin
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Odom())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
