from geometry_msgs.msg import Vector3
from utils import *
import copy

print(5)
a = Vector3()
print(a)
a.x = 5.0
print(a)
b = Vector3()
b.z = 10.0
# c = a + b # ADDING NOT ALLOWED
# print(c)

c = add(a, b)
print(c)

d = copy.copy(c)
print(d)
c.x = 10.0
print(c, d)

u = Pose()
print(u)
v = copy.copy(u)
k = copy.deepcopy(u)
u.position.x = 1.0
print("u", u)
print("v", v)
print("k", k)
