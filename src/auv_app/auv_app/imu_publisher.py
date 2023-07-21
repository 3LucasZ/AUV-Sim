import rclpy
from rclpy.node import Node
import tf2_geometry_msgs

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from auv_app.utils import *

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.rawSimImuSub = self.create_subscription(
            Imu, "/model/auv/raw_sim_imu", self.imu_sub_cb, 10
        )
        self.rawImuPub = self.create_publisher(Imu, "/model/auv/raw_imu", 10)
        self.imuPub = self.create_publisher(Imu, "/model/auv/imu", 10)

        # self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "world"
        # t.child_frame_id = "imu1"
        # self.tf_static_broadcaster.sendTransform(t)
        # t.child_frame_id = "imu2"
        # self.tf_static_broadcaster.sendTransform(t)

    def imu_sub_cb(self, msg: Imu):
        # pub raw imu
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.rawImuPub.publish(msg)

        # pub imu
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        # rotate by the orientation to assume flat on earth
        msg.linear_acceleration = tf2_geometry_msgs.do_transform_vector3(
            stampVector3(msg.header, msg.linear_acceleration),
            stampTransform(msg.header, transformFromQuaternion(msg.orientation)),
        ).vector
        # remove gravity term
        msg.linear_acceleration.z -= 9.79999999999996
        # pub
        self.imuPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    imuPublisher = ImuPublisher()

    rclpy.spin(imuPublisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imuPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
