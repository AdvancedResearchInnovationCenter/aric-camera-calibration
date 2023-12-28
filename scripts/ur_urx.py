"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
import urx
import rospy
from robot_base import Robot


class UrRtde(Robot):
    def __init__(self, robot_ip):
        Robot.__init__(self)
        self.robot_ip = robot_ip
        self.robot = urx.Robot(robot_ip, True)

    def move_TCP(self, pose_vec, vel, acc, slow=False):
        self.robot.movel((pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]), vel, acc, wait=True)

    def speed_command(self, twist_vec, acc):
        self.robot.speedl_tool(twist_vec, acc, 1)

    def get_pose(self):
        return self.robot.get_pose()

    def get_vel(self):
        return [0, 0, 0] #TODO
    #    vel, _ = self.compute_rate()
    #    return vel
                
    def get_wrench(self):
        return self.robot.get_force()

    def setio(self, pin, value):
        self.robot.set_digital_out(pin, value)

    def get_analog_input(self):
        return self.robot.get_analog_in(2)

    def compute_rate(self, new_pose):
        raise NotImplementedError