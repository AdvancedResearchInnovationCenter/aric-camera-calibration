#!/usr/bin/env python
from time import sleep
import abb

R = abb.Robot(ip='192.168.125.1')
print("start")
[position, quat] = R.get_cartesian()
print(position)
print(quat)

print(R.set_joints([0,0,0,0,0,0]))
# print(R.set_joints([0,0,0,0,0,30]))
# print(R.set_cartesian([[1115.0, -0.04, 1186.5], [0.707, 0.0, 0.707, 0.0]]))
[position, quat] = R.get_cartesian()
print(position)
print(quat)
print("success")
print('ok')
