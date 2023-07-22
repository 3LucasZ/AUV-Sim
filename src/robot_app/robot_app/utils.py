from geometry_msgs.msg import (
    Point,
    PointStamped,
    Vector3,
    Vector3Stamped,
    Pose,
    PoseStamped,
    Quaternion,
    QuaternionStamped,
    Transform,
    TransformStamped,
)
from std_msgs.msg import Float64, Header


# math
def dist(a: Point, b: Point) -> Float64:
    ret = Float64()
    ret.data = pow(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2), 0.5)
    return ret


def floatEq(a, b):
    return abs(a - b) < 0.00000000001


def add(u: Vector3, v: Vector3) -> Vector3:
    ret = Vector3()
    ret.x = u.x + v.x
    ret.y = u.y + v.y
    ret.z = u.z + v.z
    return ret


def minus(u: Vector3, v: Vector3) -> Vector3:
    ret = Vector3()
    ret.x = u.x - v.x
    ret.y = u.y - v.y
    ret.z = u.z - v.z
    return ret


def mult(vector3: Vector3, gain: float) -> Vector3:
    ret = Vector3()
    ret.x = vector3.x * gain
    ret.y = vector3.y * gain
    ret.z = vector3.z * gain
    return ret


# inline stamping
def stampVector3(header: Header, vector3: Vector3):
    ret = Vector3Stamped()
    ret.header = header
    ret.vector = vector3
    return ret


def stampQuaternion(header, quaternion):
    ret = QuaternionStamped()
    ret.header = header
    ret.quaternion = quaternion
    return ret


def stampTransform(header, transform):
    ret = TransformStamped()
    ret.header = header
    ret.transform = transform
    return ret


# partial conversions
def transformFromQuaternion(quaternion):
    ret = Transform()
    ret.rotation = quaternion
    return ret


# full conversions
def pointFromVector3(vector3: Vector3) -> Point:
    ret = Point()
    ret.x = vector3.x
    ret.y = vector3.y
    ret.z = vector3.z
    return ret


def vector3FromPoint(point: Point) -> Vector3:
    ret = Vector3()
    ret.x = point.x
    ret.y = point.y
    ret.z = point.z
    return ret


def transformFromPose(pose: Pose) -> Transform:
    ret = Transform()
    ret.translation = vector3FromPoint(pose.position)
    ret.rotation = pose.orientation
    return ret
