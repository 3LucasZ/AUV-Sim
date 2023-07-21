from geometry_msgs.msg import *
from std_msgs.msg import *


# math
def dist(a: Point, b: Point) -> Float64:
    ret = Float64()
    ret.data = pow(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2), 0.5)
    return ret


def floatEq(a, b):
    return abs(a - b) < 0.00000000001


def add(a: Vector3, b: Vector3) -> Vector3:
    ret = Vector3()
    ret.x = a.x + b.x
    ret.y = a.y + b.y
    ret.z = a.z + b.z
    return ret


def mult(a: Vector3, b: float) -> Vector3:
    ret = Vector3()
    ret.x = a.x * b
    ret.y = a.y * b
    ret.z = a.z * b
    return ret


# inline stamping
def stampVector3(header, vector):
    ret = Vector3Stamped()
    ret.header = header
    ret.vector = vector
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
    point = Point()
    point.x = vector3.x
    point.y = vector3.y
    point.z = vector3.z
    return point


def vector3FromPoint(point: Point) -> Vector3:
    vector3 = Vector3()
    vector3.x = point.x
    vector3.y = point.y
    vector3.z = point.z
    return vector3


def transformFromPose(pose: Pose) -> Transform:
    transform = Transform()
    transform.translation = vector3FromPoint(pose.position)
    transform.rotation = pose.orientation
    return transform
