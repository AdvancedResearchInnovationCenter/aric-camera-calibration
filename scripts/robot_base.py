"""
This script includes the interface class for the 
robot controller classes
"""

class Robot:
    def __init__(self):
        pass

    def move_TCP(pose_vec, vel, acc, slow):
        raise NotImplementedError

    def speed_command(self, twist_vec, acc):
        raise NotImplementedError

    def get_pose(self):
        raise NotImplementedError

    def get_vel(self):
        raise NotImplementedError
                
    def get_wrench(self):
        raise NotImplementedError

    def setio(self, pin, value):
        raise NotImplementedError

    def get_analog_input(self):
        raise NotImplementedError
