import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Float64
from robot_app.utils import *


class Swim(Node):
    def __init__(self):
        super().__init__("swim")
        self.thrusterPubs = []
        for i in range(1,9):
            self.thrusterPubs.append(self.create_publisher(Float64, "/thruster"+(i+1), 10))
       
        self.poseSub = self.create_subscription(
            PoseStamped, "/pose", self.pose_sub_cb, 10 # true pose for sim, est pose for irl
        )

    def pose_sub_cb(self, msg: ImuMsg):
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
