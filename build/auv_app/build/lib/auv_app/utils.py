from geometry_msgs.msg import *
from std_msgs.msg import *


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


def Vector3ToPoint(a: Vector3) -> Point:
    ret = Point()
    ret.x = a.x
    ret.y = a.y
    ret.z = a.z
    return ret


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


def stampTransfrom(header, transform):
    ret = TransformStamped()
    ret.header = header
    ret.transform = transform
    return ret


def transformFromQuaternion(quaternion):
    ret = Transform()
    ret.rotation = quaternion
    return ret
