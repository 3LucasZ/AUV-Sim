import rclpy
from rclpy.node import Node
import tf2_geometry_msgs

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from auv_app.utils import *


class PoseEstimator(Node):
    def __init__(self):
        # state
        self.linVelEstimate = Vector3()
        self.poseEstimate = PoseStamped()
        self.pose = PoseStamped()
        self.positionEstimateError = Float64()

        # settings
        self.imuHz = 100.0
        self.pubPeriod = 0.01
        self.poseEstimate.header.frame_id = "dummy"
        self.poseEstimate.pose.position.x = 0.0
        self.poseEstimate.pose.position.y = 0.0
        self.poseEstimate.pose.position.z = 0.0
        self.pose.header.frame_id = "dummy"

        # setup
        self.linVelEstimate.x = 0.0
        self.linVelEstimate.y = 0.0
        self.linVelEstimate.z = 0.0
        self.imuPeriod = 1.0 / self.imuHz
        super().__init__("pose_estimator")
        self.imuSub = self.create_subscription(Imu, "/imu", self.imu_sub_cb, 10)
        self.odomSub = self.create_subscription(
            Odometry, "/model/auv/odometry", self.odom_sub_cb, 10
        )
        self.poseEstimatePub = self.create_publisher(
            PoseStamped, "/model/auv/pose_estimate", 10
        )
        self.posePub = self.create_publisher(PoseStamped, "/model/auv/pose", 10)
        self.positionEstimateErrorPub = self.create_publisher(
            Float64, "/model/auv/position_estimate_error", 10
        )
        self.timer = self.create_timer(self.pubPeriod, self.pub_cb)

    def imu_sub_cb(self, msg: Imu):
        # access
        header = msg.header
        ori = msg.orientation
        linAcc = msg.linear_acceleration

        # calibrate linAcc
        linAcc = tf2_geometry_msgs.do_transform_vector3(
            stampVector3(header, linAcc),
            stampTransform(header, transformFromQuaternion(ori)),
        ).vector
        linAcc.z -= 9.79999999999996

        # update linVel, pos, ori
        dPos = add(
            mult(self.linVelEstimate, self.imuPeriod),
            mult(linAcc, self.imuPeriod * self.imuPeriod * 0.5),
        )
        self.linVelEstimate = add(self.linVelEstimate, mult(linAcc, self.imuPeriod))
        self.poseEstimate.pose.position = Vector3ToPoint(
            add(self.poseEstimate.pose.position, dPos)
        )
        self.poseEstimate.pose.orientation = ori

    def odom_sub_cb(self, msg: Odometry):
        # calculate scalar for error
        self.positionEstimateError = dist(
            self.poseEstimate.pose.position, msg.pose.pose.position
        )
        # store true pose
        self.pose.pose = msg.pose.pose

    def pub_cb(self):
        self.poseEstimate.header.stamp = self.get_clock().now().to_msg()
        self.poseEstimatePub.publish(self.poseEstimate)
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.posePub.publish(self.pose)
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
