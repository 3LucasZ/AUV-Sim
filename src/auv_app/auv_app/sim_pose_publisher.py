import rclpy
from rclpy.node import Node

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf2_msgs.msg import *
from nav_msgs.msg import *
from auv_app.utils import *

from tf2_ros import TransformBroadcaster


class SimPosePublisher(Node):
    def __init__(self):
        super().__init__("sim_pose_publisher")
        self.simPoseSub = self.create_subscription(
            Pose, "/model/auv/raw_sim_pose", self.sim_pose_sub_cb, 10
        )
        self.auvSimPosePub = self.create_publisher(
            PoseStamped, "/model/auv/sim_pose", 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def sim_pose_sub_cb(self, msg: Pose):
        # publish dummy_link sim_pose
        ret: PoseStamped = PoseStamped()
        ret.header.stamp = self.get_clock().now().to_msg()
        ret.header.frame_id = "world"
        ret.pose = msg
        self.auvSimPosePub.publish(ret)

        # perform transform world -> sim_pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "sim_pose"
        t.transform = transformFromPose(msg)
        self.tf_broadcaster.sendTransform(t)


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
