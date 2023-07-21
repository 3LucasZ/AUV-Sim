import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf2_msgs.msg import *
from nav_msgs.msg import *
from auv_app.utils import *


class SimPosePublisher(Node):
    def __init__(self):
        super().__init__("sim_pose_publisher")
        self.simPoseSub = self.create_subscription(
            Pose, "/model/auv/raw_sim_pose", self.sim_pose_sub_cb, 2
        )
        self.auvSimPosePub = self.create_publisher(
            PoseStamped, "/model/auv/sim_pose", 2
        )

    def sim_pose_sub_cb(self, msg: Pose):
        ret: PoseStamped = PoseStamped()
        ret.header.stamp = self.get_clock().now().to_msg()
        ret.header.frame_id = "base_link"
        ret.pose = msg
        self.auvSimPosePub.publish(ret)


def main(args=None):
    rclpy.init(args=args)

    sim_pose_publisher = SimPosePublisher()

    rclpy.spin(sim_pose_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
