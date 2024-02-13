"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
from math import degrees
from socket import setdefaulttimeout
from turtle import pos
from mitsubishi import MITSUBISHI
import rospy
from robot_base import Robot
import numpy as np
from scipy.spatial.transform import Rotation as R
from multiprocessing import Process, Array


class MitsubishiRobot(Robot):
    def __init__(self, robot_ip='192.168.0.20'):
        Robot.__init__(self)
        self.robot_ip = robot_ip
        self.robot = MITSUBISHI(ip=self.robot_ip)

        # self.get_pose_process = Process(target=self.get_cartesian_pose, name="getting_pose")

        self.last_position = [0., 0., 0.]
        self.last_rpy = [0., 0., 0.]

        self.robot_in_motion = False

        self.robot.activate_control(True)
        self.robot.activate_servo(True)
        self.robot.start_program()
        
        
    def move_TCP(self, pose_vec, vel, acc, slow=False):
        position = [pose_vec[i] * 1000 for i in range(3)] #convert from m to mm
        rot_vec = pose_vec[3:]

        #Rotate poses by 90 deg to be compatible with base frame of UR10
        mitsubishi_R_ur = R.from_euler('z', 90, degrees=True).as_matrix()

        position = np.matmul(mitsubishi_R_ur, np.array(position)).tolist()
        ur_R_tcp = R.from_rotvec(rot_vec).as_matrix()
        mitsubishi_R_tcp = np.matmul(mitsubishi_R_ur, ur_R_tcp)


        rpy = R.from_matrix(mitsubishi_R_tcp).as_euler('ZYX', degrees=True).tolist()

        if slow:
            self.robot.set_speed(vel/5)
        else:
            self.robot.set_speed(vel)

        pose_values = [position[0], position[1], position[2], rpy[2], rpy[1], rpy[0]]
        pose_string = self.construct_pose_msg(pose_values)

        self.robot.move_robot(pose_string)
        
        self.robot_in_motion = True


        while self.robot_in_motion:
            rospy.sleep(0.1)

        return True
    
    def speed_command(self, twist_vec, acc):
        raise NotImplementedError

    def get_pose(self, force_reading=False):
        try:
            pose_string = self.robot.read('PPOSF')
            if len(pose_string) > 10:

                pose_measurement = self.parse_msg(pose_string)

                position = pose_measurement[:3] 
                rot_rpy = pose_measurement[3:6]

                self.last_position = position
                self.last_rpy = rot_rpy
            else:
                position = self.last_position
                rot_rpy = self.last_rpy
        
        except:
            # rospy.logwarn("Error getting pose")
            position = self.last_position
            rot_rpy = self.last_rpy
        if self.robot_in_motion:
            self.robot_in_motion = self.check_motion_status()
            
        rot_mat = R.from_euler('ZYX', [rot_rpy[2], rot_rpy[1], rot_rpy[0]], degrees=True).as_matrix()
        # print(rot_rpy)
        # print(rot_mat)
        #Rotate poses by -90 deg to be compatible with base frame of UR10
        ur_R_mitsubishi = R.from_euler('z', -90, degrees=True).as_matrix()
        
        updated_rot_mat = np.matmul(ur_R_mitsubishi, rot_mat)
        rotvec = R.from_matrix(updated_rot_mat).as_rotvec().tolist()


        position = [x / 1000 for x in position] #convert from mm to m
        position = np.matmul(ur_R_mitsubishi, np.array(position)).tolist()
            
        pose = position + rotvec
        
      
        return pose

    def get_vel(self):
        return [0, 0, 0, 0, 0, 0] #TODO: implement rate computation
    #    vel, _ = self.compute_rate()
    #    return vel
                
    def get_wrench(self):
        return [0, 0, 0, 0, 0, 0] #TODO: implement rate computation
        # raise NotImplementedError

    def setio(self, pin, value):
        raise NotImplementedError

    def get_analog_input(self):
        raise NotImplementedError

    def compute_rate(self, new_pose):
        raise NotImplementedError
    
    def parse_msg(self, msg_string, variables=['X', 'Y', 'Z', 'A', 'B', 'C']):
        msg_string = (msg_string)
        # print(msg_string)
        xx = msg_string[5:] #Remove 'QoK' parts from message
        x_split = xx.split(';') 
        values = []
        for variable in variables:
            idx = x_split.index(variable)
            values.append(float(x_split[idx+1]))
        return values

    
    def construct_pose_msg(self, values):
        pose_msg_string = '(' 
        N_variables = len(values)
        for i in range(N_variables):
            if i == (N_variables - 1):
                pose_msg_string = pose_msg_string + str(round(values[i],3))
            else:
                pose_msg_string = pose_msg_string + str(round(values[i],3)) + ', '

        pose_msg_string = pose_msg_string + ')'
        return pose_msg_string
    
    def check_motion_status(self):
        status_msg = str(self.robot.read('STATE'))
        status_msg_split = status_msg.split(';')
        movement_status_msg = status_msg_split[4]
        if movement_status_msg == "A1060":
            return False
        else:
            return True