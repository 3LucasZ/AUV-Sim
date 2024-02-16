import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu as ImuMsg
from custom_msgs.msg import X150Status
from robot_app.utils import *


class X150StatusPub(Node):
    def __init__(self):
        super().__init__("X150_status_pub")

        # set static transform base_link -> X150
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "X150"
        self.tf_static_broadcaster.sendTransform(t)

        # X150 status pub
        self.X150StatusPub = self.create_publisher(X150Status, "/X150_status", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # # X150 -> X110 range bearing pub
        # self.rangeBearingPub = self.create_publisher(ImuMsg, "/imu", 10)

    def timer_callback(self):
        msg = X150Status()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "X150"
        msg.timestamp_sec = 0.0  # dummy
        msg.supply_v = 5.0  # dummy
        msg.temperature_c = 12.0  # dummy
        msg.pressure_mb = 1100  # dummy
        water_density_kg_m3 = 997.0
        gravity_m_s2 = 9.81
        msg.depth_m = 100 * msg.pressure_mb / (water_density_kg_m3 * gravity_m_s2)
        msg.sound_mps = (
            1404.3
            + 4.7 * msg.temperature_c
            - 0.04 * msg.temperature_c * msg.temperature_c
        )
        msg.yaw_deg = 0.0  # dummy
        msg.pitch_deg = 0.0  # dummy
        msg.roll_deg = 0.0  # dummy
        self.X150StatusPub.publish(msg)


# spin
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(X150StatusPub())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
