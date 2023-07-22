import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from robot_app.utils import *


class PoseEstimator(Node):
    def __init__(self):
        # state
        self.linVelEstimate = Vector3()
        self.poseEstimate = PoseStamped()

        # settings
        self.poseEstimate.pose.position.x = 0.0
        self.poseEstimate.pose.position.y = 0.0
        self.poseEstimate.pose.position.z = 0.25

        # setup
        self.linVelEstimate.x = 0.0
        self.linVelEstimate.y = 0.0
        self.linVelEstimate.z = 0.0
        super().__init__("pose_estimator")
        # sub
        self.imuTunedSub = self.create_subscription(
            Imu, "model/rover/imu_tuned", self.imu_sub_cb, 10
        )
        self.simPoseSub = self.create_subscription(
            PoseStamped, "model/rover/sim_pose", self.sim_pose_sub_cb, 10
        )
        # pub
        self.poseEstimatePub = self.create_publisher(
            PoseStamped, "model/rover/pose_estimate", 10
        )
        self.positionEstimateErrorPub = self.create_publisher(
            Float64, "/model/rover/position_estimate_error", 10
        )
        # time
        self.lastImuMeasTime = self.get_clock().now().nanoseconds

    def imu_sub_cb(self, msg: Imu):
        # upd ti, tf, dt
        curTime = self.get_clock().now().nanoseconds
        dt = (curTime - self.lastImuMeasTime) / 1e9
        self.lastImuMeasTime = curTime

        # upd pose estimate
        dPos = add(
            mult(self.linVelEstimate, dt), mult(msg.linear_acceleration, dt * dt * 0.5)
        )
        self.poseEstimate.pose.position = pointFromVector3(
            add(self.poseEstimate.pose.position, dPos)
        )
        self.poseEstimate.pose.orientation = msg.orientation
        self.linVelEstimate = add(
            self.linVelEstimate, mult(msg.linear_acceleration, dt)
        )

        # pub pose estimate
        self.poseEstimate.header.frame_id = "rover_world"
        self.poseEstimate.header.stamp = self.get_clock().now().to_msg()
        self.poseEstimate.pose.position.z = 0.0
        self.poseEstimatePub.publish(self.poseEstimate)

    def sim_pose_sub_cb(self, msg: PoseStamped):
        # calc pose estimate error (scalar)
        self.positionEstimateError = dist(
            self.poseEstimate.pose.position, msg.pose.position
        )

        # pub pose estimate error
        self.positionEstimateErrorPub.publish(self.positionEstimateError)


def main(args=None):
    rclpy.init(args=args)

    pose_estimation = PoseEstimator()

    rclpy.spin(pose_estimation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
