#!/usr/bin/env python
from time import sleep
import abb

def ABB_HOME():
    R = abb.Robot(ip='192.168.125.1')
    print("start")
    [position, quat] = R.get_cartesian()
    print(position)
    print(quat)
    # print("This will move the robot to Box Position, Proceed? ")

    # print(R.set_joints([-19.65,32.8,-20.3,-180,-77.5,160]))
    print(R.set_joints([-20.08,33.62,-26.49,0,82.87,-20.08]))
    # print(R.set_joints([0,0,0,0,0,30]))
    # print(R.set_cartesian([[-400, -1100, 800], [0, 1, 0, 0.0]]))
    [position, quat] = R.get_cartesian()
    print(position)
    print(quat)
    print("success")
    print('ok')

    return [R.set_joints([-20.08,33.62,-26.49,0,82.87,-20.08])]


ABB_HOME()
