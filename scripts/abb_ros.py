"""
This script includes implementation for robot control 
functions for UR robots using the ur_rtde package
"""
from math import degrees
from socket import setdefaulttimeout
from turtle import pos
import abb
import rospy
from robot_base import Robot
import numpy as np
from scipy.spatial.transform import Rotation as R
from multiprocessing import Process, Array

#NOTE: ABB quaternion format: [w x y z]
#Scipy quaternion format [x y z w]


class AbbRobot(Robot):
    def __init__(self, robot_ip='192.168.125.1'):
        Robot.__init__(self)
        self.robot_ip = robot_ip
        self.robot = abb.Robot(ip=self.robot_ip)
        self.is_active = False

        # self.get_pose_process = Process(target=self.get_cartesian_pose, name="getting_pose")

        self.last_position = [0., 0., 0.]
        self.last_quat = [1., 0., 0., 0.]

        self.move_finish = True
        
    # def get_cartesian_pose(self, in_position, in_quat):
    #     [position, quat] = self.robot.get_cartesian()
    #     in_position[:] = position
    #     in_quat[:] = quat
        
    def move_TCP(self, pose_vec, vel, acc, slow=False):
        position = [pose_vec[i] * 1000 for i in range(3)] #convert from m to mm
        rot_vec = pose_vec[3:]

        #Rotate poses by 90 deg to be compatible with base frame of UR10
        abb_R_ur = R.from_euler('z', 90, degrees=True).as_matrix()

        position = np.matmul(abb_R_ur, np.array(position)).tolist()
        ur_R_tcp = R.from_rotvec(rot_vec).as_matrix()
        abb_R_tcp = np.matmul(abb_R_ur, ur_R_tcp)


        quat = R.from_matrix(abb_R_tcp).as_quat().tolist()
        abb_quat = [quat[3]] + quat[:3]
        if slow:
            self.robot.set_speed([vel*1000/5/2, 50/5, 50, 50])
        else:
            self.robot.set_speed([vel*1000/2, 50, 50, 50])
        self.is_active = True
        self.move_finish = None
        self.move_finish = self.robot.set_cartesian([position, abb_quat])

        while True:
            self.get_pose(force_reading=True)
            delta_pos = np.linalg.norm(np.subtract(self.last_position, position))/1000
            delta_ang = np.linalg.norm(np.subtract( np.abs(self.last_quat), np.abs(abb_quat)))
            delta_pose = delta_pos + delta_ang
            if (delta_pos < 0.0001) and (delta_ang < 0.001):
                break
            rospy.sleep(0.01)
        rospy.sleep(0.1)

        self.is_active = False

        return True
    
    def speed_command(self, twist_vec, acc):
        raise NotImplementedError

    def get_pose(self, force_reading=False):

        #Get the robot pose through a process with timeout to overcome communication freezing problem
        # out_position = Array('d', [0.0, 0.0, 0.0])
        # out_quat = Array('d', [1.0, 0.0, 0.0, 0.0])
        # get_pose_process = Process(target=self.get_cartesian_pose, name="getting_pose", args=(out_position, out_quat))
        # get_pose_process.start()
        # get_pose_process.join(1)
        # if get_pose_process.is_alive():
        #     get_pose_process.terminate()
        # position = out_position[:]
        # quat = out_quat[:]

        #TODO, keep getting pose during movement

        if not self.is_active or force_reading:
            [position, quat] = self.robot.get_cartesian()
            if len(position) == 0 or np.linalg.norm(position) == 0:
                #Catch some errors in communicating with the robot
                position = self.last_position
                quat = self.last_quat
            else:
                self.last_position = position
                self.last_quat = quat
        else:
            position = self.last_position
            quat = self.last_quat
            
        position = [x / 1000 for x in position] #convert from mm to m

        #Rotate poses by -90 deg to be compatible with base frame of UR10
        ur_R_abb = R.from_euler('z', -90, degrees=True).as_matrix()
        
        rot_mat = R.from_quat(quat[1:] + [quat[0]]).as_matrix()

        updated_rot_mat = np.matmul(ur_R_abb, rot_mat)
        rotvec = R.from_matrix(updated_rot_mat).as_rotvec().tolist()

        #Modify the position
        position = np.matmul(ur_R_abb, np.array(position)).tolist()
        # rotvec = R.from_quat(quat[1:] + [quat[0]]).as_rotvec().tolist() #TODO: Remove, doesn't consider base rotation

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