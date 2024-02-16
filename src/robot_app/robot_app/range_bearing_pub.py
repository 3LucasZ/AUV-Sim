import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from custom_msgs.msg import RangeBearing
from math import *


class RangeBearingPub(Node):
    def __init__(self):
        super().__init__("range_bearing_pub")

        # X150 sim sub
        self.r = 0.0
        self.az = 0.0
        self.el = 0.0
        self.X150_sim_sub = self.create_subscription(
            Vector3, "X150_sim", self.X150_sim_sub_cb, 10
        )

        # range bearing pub
        self.rangeBearingPub = self.create_publisher(RangeBearing, "range_bearing", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def X150_sim_sub_cb(self, msg: Vector3):
        self.r = msg.x
        self.az = msg.y
        self.el = msg.z

    def timer_callback(self):
        msg = RangeBearing()
        msg.orientation_present = True
        msg.yaw_deg = 0.0  # dummy
        msg.roll_deg = 0.0  # dummy
        msg.pitch_deg = 0.0  # dummy
        msg.depth_m = 0.0  # dummy
        msg.range_present = True
        msg.range_m = self.r
        msg.azimuth_deg = self.az
        msg.elevation_deg = self.el
        x: float = self.r * sin(self.el) * cos(self.az)
        y: float = self.r * sin(self.el) * sin(self.az)
        z: float = self.r * cos(self.el)
        msg.remote_east_m = x
        msg.remote_north_m = y
        msg.remote_depth_m = z
        self.rangeBearingPub.publish(msg)


# spin
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RangeBearingPub())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
