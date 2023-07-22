import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_app.utils import *


class TruePose(Node):
    def __init__(self):
        super().__init__("true_pose")

        # true_pose_sim sub
        self.truePoseSimSub = self.create_subscription(
            PoseStamped, "/true_pose_sim", self.sim_true_pose_sub_cb, 10
        )

        # true_pose pub
        self.truePosePub = self.create_publisher(PoseStamped, "/true_pose", 10)

    def sim_true_pose_sub_cb(self, msg: PoseStamped):
        # correct header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        self.truePosePub.publish(msg)


# spin
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TruePose())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
