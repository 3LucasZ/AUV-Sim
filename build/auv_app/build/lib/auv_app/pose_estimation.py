import rclpy
from rclpy.node import Node
import tf2_geometry_msgs

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from auv_app.utils import *


class PoseEsimation(Node):
    def __init__(self):
        self.pose = Pose()
        self.linVel = Vector3()
        # settings
        self.imuHz = 100.0
        self.dt = 1.0 / self.imuHz
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0

        # setup
        self.linVel.x = 0.0
        self.linVel.y = 0.0
        self.linVel.z = 0.0
        super().__init__("pose_estimation")
        self.subscription = self.create_subscription(
            Imu, "imu", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        # access
        header = msg.header
        ori = msg.orientation
        angVel = msg.angular_velocity
        linAcc = msg.linear_acceleration

        # calibrate linAcc
        linAcc = tf2_geometry_msgs.do_transform_vector3(
            stampVector3(header, linAcc),
            stampTransfrom(header, transformFromQuaternion(ori)),
        ).vector
        linAcc.z -= 9.79999999999996

        # update linVel, pos
        dPos = add(
            mult(self.linVel, self.dt),
            mult(linAcc, self.dt * self.dt * 0.5),
        )
        self.linVel = add(self.linVel, mult(linAcc, self.dt))
        self.pose.position = Vector3ToPoint(add(self.pose.position, dPos))

        self.get_logger().info("linAcc:" + str(linAcc))
        self.get_logger().info("pose:" + str(self.pose))

        # publish pos estimate
        # publish pos error


def main(args=None):
    rclpy.init(args=args)

    pose_estimation = PoseEsimation()

    rclpy.spin(pose_estimation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
