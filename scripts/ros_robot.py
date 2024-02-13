#!/usr/bin/env python3
"""
This script includes a class to methods for the interface between ROS and various robot control packages 
"""

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
import tf2_ros
import datetime
from ros_robot_pkg.srv import moveRobot, desiredTCP, pegHole, setValue, moveRobotRelative
from scipy.spatial.transform import Rotation as R
from kinematics import RobotKinematics
import time
import copy
from ur_rtde import UrRtde
import _thread
from abb_ros import AbbRobot
from mitsubishi_ros import MitsubishiRobot
import sys

#Deburring end effector
# TCP_to_pressure_foot = np.array([   [   -0.7140,         0,   -0.7001,   -0.1042],
#                                     [   -0.7001,   -0.0025,    0.7140,    0.1082],
#                                     [   -0.0018,    1.0000,    0.0018,    0.0922],
#                                     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# g = np.eye(4)
# g[0,3] = 0.0036
# g[1,3] = -0.003
# g[2,3] = 0.00
# TCP_to_pressure_foot = np.matmul(TCP_to_pressure_foot, g)

#davis346 deburring mount on robot
# TCP_to_cam = np.array([ [-0.70417779, -0.02482665, -0.70958951, -0.08241791],
#                         [  -0.70992744,  0.00816375,  0.70422751,  0.0814719 ],
#                         [  -0.01169069,  0.99965844, -0.02337385,  0.03069571],
#                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#Normal tactile sensor mount
# TCP_to_cam = np.array([ [1., 0., 0., -0.0],
#                         [0., 0., 1.,  0.07 ],
#                         [0., -1., 0.,  0.06],
#                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
# TCP_to_cam[1,3] = 0.07
# TCP_to_cam[2,3] = 0.06


# #d435 huang's configuration
# TCP_to_cam = np.array([ [0.71816448,  0.00808576,  0.69582641,  0.03926356],
#                         [-0.69540667, -0.02827723,  0.71805986,  0.08717657],
#                         [0.0254821,  -0.99956742, -0.01468481,  0.09887745],
#                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])


#visuotactile on abb
# TCP_to_cam = np.array([ [0.7071,  0.00808576,  0.7071,  0.0],
#                         [-0.7071, 0.0,  0.7071,  0.0],
#                         [0.0,  -1.0, 0.0,  0.07],
#                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#Halwani's sensor on the ABB
# TCP_to_cam = np.array([[ 0.6949,  -0.0575,  0.7168,  0.0496],
#                         [-0.7178, 0.0050,  0.6962, 0.0091],
#                         [-0.0437, -0.9983, -0.0378,  0.0779],
#                         [ 0.,          0.,          0.,          1.        ]])



#Halwani's sensor on the UR10
# TCP_to_cam = np.array([[ 0.99866872,  0.02011093, -0.04750102 ,-0.10126922],
#                         [ 0.04747108,  0.00196362 , 0.99887068 , 0.07627988],
#                         [ 0.02018149, -0.99979583 , 0.00100631 , 0.11786909],
#                         [ 0. ,         0.    ,      0.   ,       1.        ]])

# cam_to_sensor_ecoflex20 = np.array([[ 0.9999,  -0.0103, 0.0016,    -0.0015],
#                                     [0.0102,  0.9995, 0.0293,   -0.0028],
#                                     [ -0.0019, -0.0292, 0.9996,    0.0866],
#                                     [      0,       0,      0,    1.0000]])

# cam_to_sensor_DragonSkin30 = np.array(          [[ 1.0,  -0.0048, 0.0021,    0.0],
#                                                 [0.0048,  1.0, -0.0046,   -0.0006],
#                                                 [ -0.002, 0.0047, 1.0,    0.0872],
#                                                 [      0,       0,      0,    1.0000]])

# TCP_to_sensor = np.matmul(TCP_to_cam, cam_to_sensor_ecoflex20)

# TCP_to_cam = TCP_to_sensor                        


class RosRobot:
    """
    This is a class for ROS interface with robot controllers
    """
    def __init__(self, robot_controller):
        # self.vel = 0.1
        # self.acc = 0.1
        # self.stop_acc = 0.3
        
        self.vel = 0.15
        self.acc = 0.15
        self.stop_acc = 0.1

        self.cmd_velocity_vector = []
        self.move_vel = False

        self.item_height = 0.11
        
        self.TCP_to_cam_matrix = []
        self.TCP_to_press_ft_matrix = []

        #visual servoing mode parameters
        self.VS_2D_mode = False#True
        self.VS_2D_initialized = False

        # assert isinstance(robot_controller, UrRtde)
        self.robot_controller = robot_controller

        self.current_TCP = 'davis'
        self.set_TCP('davis')

        time.sleep(0.2)

        #ros publishers and subscribers
        self.ros_node = rospy.init_node('ur10_node', anonymous=True)
        self.pose_publisher = rospy.Publisher('tcp_pose', PoseStamped, queue_size=1) #publish pose of robot flange relative to base
        self.tcp_velocity_publisher = rospy.Publisher('/tcp/vel', Twist, queue_size=1) #publish velocity of camera in camera frame
        self.velocity_publisher = rospy.Publisher('/dvs/vel', Twist, queue_size=1) #publish velocity of camera in camera frame
        self.speed_publisher = rospy.Publisher('/dvs/spd', Float64, queue_size=1) #publish speed of camera
        self.cam_pose_publisher = rospy.Publisher('/dvs/pose', PoseStamped, queue_size=1)  #publish pose of camera relative to base
        self.cmd_vel_subs = rospy.Subscriber("ur_cmd_vel", Twist, self.move_robot_callback, queue_size=1) #receives velicty command in the activve TCP frame
        self.cmd_pose_subs = rospy.Subscriber("ur_cmd_pose", Pose, self.move_pose_callback) #publish target pose of active TCP relative to base
        self.arm_force_publisher = rospy.Publisher('/ur_force', Twist, queue_size=1) #publishes the contact forces in the active TCP frame
        self.cmd_adjust_pose_subs = rospy.Subscriber("ur_cmd_adjust_pose", Pose, self.adjust_pose_callback) #adjusts the pose of the active TCP
        self.rotate_ee_cmd = rospy.Subscriber("ur_rotate_ee_x", Float64, self.angle_callback_x)
        self.rotate_ee_cmd = rospy.Subscriber("ur_rotate_ee", Float64, self.angle_callback_z)
        self.pressure_movement_subs = rospy.Subscriber("move_pressure_to_cam", Bool, self.move_PF_to_cam) #moves the pressure_ft to the location of the camera
        self.pickup_service = rospy.Service("ur_pickup", Empty, self.pick_item) 
        self.set_tcp_service = rospy.Service("set_TCP", desiredTCP, self.set_TCP_cb)
        self.move_service = rospy.Service('move_ur', moveRobot, self.moveRobot_cb)
        self.adjust_service = rospy.Service('move_TCP', moveRobot, self.moveTCP_cb)
        self.relative_move_service = rospy.Service('move_ur_relative', moveRobotRelative, self.moveRobotRelative_cb)
        self.move_service = rospy.Service('fire_drill', moveRobot, self.fire_drill_cb)
        self.insert_split_pin = rospy.Service('insert_split_pin', pegHole, self.split_pin_cb)
        self.visual_split_pin = rospy.Service('visual_split_pin', pegHole, self.visual_split_pin_cb)
        self.retract_split_pin = rospy.Service('retract_split_pin', pegHole, self.retract_split_pin_cb)
        self.change_ref_vel = rospy.Service('change_ref_vel', setValue, self.change_ref_vel_cb)
        self.change_ref_acc = rospy.Service('change_ref_acc', setValue, self.change_ref_acc_cb)
        
        self.rate = rospy.Rate(50)
        self.rate_c = rospy.Rate(50)

        self.robot_pose = PoseStamped()
        self.camera_pose = PoseStamped()
        self.prev_camera_pose = PoseStamped()
        self.pressure_ft_pose = PoseStamped()
        self.pre_insertion_split_pin_pose = Pose()
        self.pre_drill_split_pin_pose = Pose()
        self.pre_insertion_davis_pose = Pose()
        self.ur_force = Twist()
        self.cam_vel = Twist()
        self.tcp_vel = Twist()
        self.cam_speed = Float64()
        self.seq = 1
        self.pose = []
        self.initial_pose = []
        self.center_pose = []

        self.kinematics = RobotKinematics()

        # self.setup_tf()

    def setup_tf(self):
        self.kinematics.send_multiple_transform('TCP', ['pressure_ft', 'davis'], transformation_matrices=[TCP_to_pressure_foot, TCP_to_cam], mode='static')

    def change_ref_vel_cb(self, req):
        self.vel = req.value

        return True
    
    def change_ref_acc_cb(self, req):
        self.acc = req.value

        return True

    def moveRobot_cb(self, req, slow=False):
        #Callback to move robot to specific pose
        self.set_TCP(req.frame)

        success = self.move_to_pose(req.target_pose, slow)
        return success

    def moveRobotRelative_cb(self, req, slow=False):
        #Callback to move robot to specific pose
        self.set_TCP(req.frame)

        success = self.move_to_pose(req.target_pose, slow, relative_frame=req.relative_frame)
        return success


    def moveTCP_cb(self, req):
        #Callback to adjust TCP by specifc distance
        self.set_TCP(req.frame)

        success = self.adjust_pose_callback(req.target_pose)
        return success


    def move_robot_callback(self, twist_msg):
        #Callback to set specific velocity command to robot
        velocity_vector = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z]).reshape(3,-1)
        angular_velocity_vector = np.array([twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z]).reshape(3,-1)
        if self.VS_2D_mode:
            if self.VS_2D_initialized == False:

                _, TCP_to_current_TCP_transformation = self.kinematics.receive_transform('ur_base', self.current_TCP) #This takes too much time
                
                self.TCP_to_current_TCP_rotmat = TCP_to_current_TCP_transformation[:3,:3]
                self.VS_2D_initialized = True

            TCP_velocity = np.matmul(self.TCP_to_current_TCP_rotmat, velocity_vector)

            self.cmd_velocity_vector = [TCP_velocity[0][0], TCP_velocity[1][0], TCP_velocity[2][0], 0., 0., 0.]

            if (np.sum(np.abs(self.cmd_velocity_vector))==0 and not self.move_vel):
                    self.move_vel = True

        else:         
            _, TCP_to_current_TCP_transformation = self.kinematics.receive_transform('ur_base', self.current_TCP) #This takes too much time, TODO: find alternative
            
            TCP_velocity = np.matmul(TCP_to_current_TCP_transformation[:3,:3], velocity_vector)
            TCP_angular_velocity = np.matmul(TCP_to_current_TCP_transformation[:3,:3], angular_velocity_vector)

            self.cmd_velocity_vector = [TCP_velocity[0][0], TCP_velocity[1][0], TCP_velocity[2][0], TCP_angular_velocity[0][0], TCP_angular_velocity[1][0], TCP_angular_velocity[2][0]]

            if (np.sum(np.abs(self.cmd_velocity_vector))==0 and not self.move_vel):
                    self.move_vel = True


    def move_pose_callback(self, pose_msg):
        #Callback to move TCP to target pose
        
        # rospy.loginfo("Pose command received:", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w)
        
        return self.move_to_pose(pose_msg)


    def move_to_pose(self, pose_msg, slow=False, relative_frame='ur_base'):
        
        _, base_to_relative = self.kinematics.receive_transform('ur_base', relative_frame)

        relative_to_target = self.kinematics.pose_to_transformation_matrix(pose_msg)

        base_to_target = self.kinematics.add_transformations(base_to_relative, relative_to_target)

        _, desired_to_org_TCP = self.kinematics.receive_transform(self.current_TCP, 'TCP')

        full_transformation_matrix = self.kinematics.add_transformations(base_to_target, desired_to_org_TCP)

        return self.move_TCP(full_transformation_matrix, slow)

    def move_to_pose_list(self, pose_msg_list, slow=False):
        desired_transformation_list = []
        for pose_msg in pose_msg_list:
            transformation_matrix = self.kinematics.pose_to_transformation_matrix(pose_msg)

            _, desired_to_org_TCP = self.kinematics.receive_transform(self.current_TCP, 'TCP')

            full_transformation_matrix = self.kinematics.add_transformations(transformation_matrix, desired_to_org_TCP)

            desired_transformation_list.append(full_transformation_matrix)

        return self.move_TCP_list(desired_transformation_list, slow)
        

    def adjust_pose_callback(self, Pose_msg):
        current_TCP_to_desired_TCP = self.kinematics.pose_to_transformation_matrix(Pose_msg)

        return self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)


    def move_frame(self, frame, transformation_matrix):

        _, base_to_current_TCP = self.kinematics.receive_transform('ur_base', frame)
        
        base_to_desired_TCP = self.kinematics.add_transformations(base_to_current_TCP, transformation_matrix)
        
        _, desired_to_org_TCP = self.kinematics.receive_transform(frame, 'TCP')
        
        full_transformation_matrix = self.kinematics.add_transformations(base_to_desired_TCP, desired_to_org_TCP)
        
        return self.move_TCP(full_transformation_matrix)


    def move_TCP(self, desired_transformation, slow=False):
        command_trans = desired_transformation[:3, 3]
        command_attitude = R.from_matrix(desired_transformation[:3, :3])
        attitude_rot_vec = command_attitude.as_rotvec()
        pose_vec = [command_trans[0], command_trans[1], command_trans[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]]
        self.robot_controller.move_TCP(pose_vec, self.vel, self.acc, slow)
        
        # self.update_poses()

        #TODO: check pose arrival
        return True

    def move_TCP_list(self, desired_transformation_list, slow=False):
        pose_vec_list = []
        for desired_transformation in desired_transformation_list:
            command_trans = desired_transformation[:3, 3]
            command_attitude = R.from_matrix(desired_transformation[:3, :3])
            attitude_rot_vec = command_attitude.as_rotvec()
            pose_vec = [command_trans[0], command_trans[1], command_trans[2], attitude_rot_vec[0], attitude_rot_vec[1], attitude_rot_vec[2]]

            pose_vec_list.append(copy.deepcopy(pose_vec))

        self.robot_controller.move_TCP_compound(pose_vec_list, self.vel, self.acc, blend=0.1, slow=slow)
        
        # self.update_poses()

        #TODO: check pose arrival
        return True

    def angle_callback_z(self, target_angle_msg):
        #rotate end effector around z axis of TCP
        # rospy.loginfo("angle command received:", target_angle_msg)

        current_TCP_to_desired_TCP = np.eye(4)
        current_TCP_to_desired_TCP[:3, :3] = R.from_rotvec([0, 0, target_angle_msg.data]).as_matrix()

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)


    def angle_callback_x(self, target_angle_msg):
        #rotate end effector around x axis of TCP
        current_TCP_to_desired_TCP = np.eye(4)
        current_TCP_to_desired_TCP[:3, :3] = R.from_rotvec([target_angle_msg.data, 0, 0,]).as_matrix()

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)


    def run_node(self):

        self.kinematics.set_transform('ur_base', 'TCP', np.eye(4))
        time.sleep(1)

        while not rospy.is_shutdown():
            self.update_poses()
            self.pose_publisher.publish(self.robot_pose)
            self.cam_pose_publisher.publish(self.camera_pose)
            self.tcp_velocity_publisher.publish(self.tcp_vel)
            self.velocity_publisher.publish(self.cam_vel)
            self.speed_publisher.publish(self.cam_speed)
            self.arm_force_publisher.publish(self.ur_force)
            
            self.rate.sleep()
        
        self.cleanup()

    def run_controller(self):
        while not rospy.is_shutdown():
            
            if (np.sum(np.abs(self.cmd_velocity_vector))!=0 or self.move_vel):
                self.robot_controller.speed_command(self.cmd_velocity_vector, self.acc)
                self.move_vel = False
            
            self.rate_c.sleep()

 
    def update_poses(self):
        self.pose = self.robot_controller.get_pose()

        TCP_transformation_matrix = np.eye(4)
        TCP_transformation_matrix[:3, :3] = R.from_rotvec(self.pose[3:]).as_matrix()
        TCP_transformation_matrix[0, 3] = self.pose[0]
        TCP_transformation_matrix[1, 3] = self.pose[1]
        TCP_transformation_matrix[2, 3] = self.pose[2]

        self.kinematics.set_transform('ur_base', 'TCP', TCP_transformation_matrix)

        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.header.frame_id = 'ur_base'
        self.robot_pose.pose = self.kinematics.transformation_matrix_to_pose(TCP_transformation_matrix)

        self.camera_pose.header.stamp = rospy.Time.now()
        self.camera_pose.header.frame_id = 'ur_base'
        if not len(self.TCP_to_cam_matrix): #check if TCP to cam static transformation has been read before
            _, self.TCP_to_cam_matrix = self.kinematics.receive_transform("TCP", "davis")  #TODO: case where this transformation does not exist
        self.camera_pose.pose =  self.kinematics.transformation_matrix_to_pose(np.matmul(TCP_transformation_matrix, self.TCP_to_cam_matrix))

        self.pressure_ft_pose.header.stamp = rospy.Time.now()
        self.pressure_ft_pose.header.frame_id = 'ur_base'
        if not len(self.TCP_to_press_ft_matrix):#check if TCP to pressure_ft static transformation has been read before
            _, self.TCP_to_press_ft_matrix = self.kinematics.receive_transform('TCP', 'pressure_ft')#TODO: case where this transformation does not exist
        self.pressure_ft_pose.pose = self.kinematics.transformation_matrix_to_pose(np.matmul(TCP_transformation_matrix, self.TCP_to_press_ft_matrix))

        cam_velocity = self.robot_controller.get_vel()
        tcp_velocity_l = self.kinematics.convert_vector_base_frame(np.array(cam_velocity[:3]), 'TCP', 'ur_base')
        tcp_velocity_w = self.kinematics.convert_vector_base_frame(np.array(cam_velocity[3:]), 'TCP', 'ur_base')

        self.tcp_vel.linear.x = tcp_velocity_l[0]
        self.tcp_vel.linear.y = tcp_velocity_l[1]
        self.tcp_vel.linear.z = tcp_velocity_l[2]
        self.tcp_vel.angular.x = tcp_velocity_w[0]
        self.tcp_vel.angular.y = tcp_velocity_w[1]
        self.tcp_vel.angular.z = tcp_velocity_w[2]

        #convert velocity to TCP frame 
        cam_velocity_tcp = self.kinematics.convert_vector_base_frame(np.array(tcp_velocity_l), 'davis', 'TCP')

        self.cam_vel.linear.x = cam_velocity_tcp[0]
        self.cam_vel.linear.y = cam_velocity_tcp[1]
        self.cam_vel.linear.z = cam_velocity_tcp[2]

        self.cam_speed = np.linalg.norm(np.array([self.cam_vel.linear.x, self.cam_vel.linear.y, self.cam_vel.linear.z]))

        wrench = self.robot_controller.get_wrench()
        wrench_pressure_ft = self.kinematics.convert_wrench_base_frame(np.array(wrench), "TCP", "pressure_ft")
        self.ur_force.linear.x = wrench_pressure_ft[0]
        self.ur_force.linear.y = wrench_pressure_ft[1]
        self.ur_force.linear.z = wrench_pressure_ft[2]
        self.ur_force.angular.x = wrench_pressure_ft[3]
        self.ur_force.angular.y = wrench_pressure_ft[4]
        self.ur_force.angular.z = wrench_pressure_ft[5]


    
    def move_PF_to_cam(self, msg):
        return self.move_frameA_to_B('pressure_ft', 'davis')


    def move_frameA_to_B(self, frameA, frameB):
        _, transformation = self.kinematics.receive_transform(frameA, frameB)
        
        return self.move_frame(frameA, transformation) 


    def set_TCP(self, target_TCP):
        self.current_TCP = target_TCP
        self.VS_2D_initialized = False
        # rospy.loginfo("Set current TCP: ", self.current_TCP)


    def set_TCP_cb(self, req):
        self.set_TCP(req.frame)

        return []


    def fire_drill_cb(self, activate_flag):
        if activate_flag.data == True:
            self.pre_drill_split_pin_pose = self.pressure_ft_pose.pose
            self.robot_controller.setio(4, True)
            rospy.sleep(13)
            self.robot_controller.setio(4, True)
        return True, "ok"


    def pick_item(self, req): #TODO: remove
        #perform a top-down grasp
        current_TCP_to_desired_TCP = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., (self.camera_pose.pose.position.z - self.item_height)], [0., 0., 0., 1.]])

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

        self.robot_controller.setio(0, True)
        
        rospy.sleep(0.1)

        current_TCP_to_desired_TCP = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., -(self.camera_pose.pose.position.z - self.item_height)], [0., 0., 0., 1.]])

        self.move_frame(self.current_TCP, current_TCP_to_desired_TCP)

        return []


    def split_pin_cb(self, activate_flag):
        #activate split pin
        start_time = rospy.Time.now().to_nsec()

        self.set_TCP('pressure_ft')

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.z = 0.002

        adjust_pose_msg = Pose()
        adjust_pose_msg.orientation.w = 1
        adjust_pose_msg.position.z = 0.000075
        adjust_pose_msg.position.x = 0.000075
        adjust_pose_msg.position.y = 0.00015

        start_pose = copy.deepcopy(self.pressure_ft_pose.pose)
        rot_mat = R.from_quat([start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]).as_matrix()
        command_pose = Pose()
        command_pose.orientation.w = start_pose.orientation.w
        command_pose.orientation.x = start_pose.orientation.x
        command_pose.orientation.y = start_pose.orientation.y
        command_pose.orientation.z = start_pose.orientation.z
        # adjust_pose_vec = [0.000075, 0.000075, 0.0002]
        adjust_pose_vec = [0.00002, 0.00002, 0.0002]
        command_pose.position
        rospy.sleep(0.1)

        base_force = self.robot_controller.get_force()
        base_force = self.kinematics.convert_wrench_base_frame(base_force, "TCP", "pressure_ft")

        index_i = 0

        while (np.linalg.norm( self.kinematics.convert_wrench_base_frame(self.robot_controller.get_force(), "TCP", "pressure_ft")[2] - base_force[2]) < 115):
            # print(self.robot.get_force() - base_force)
            current_time = rospy.Time.now().to_nsec()
            # cmd_vel_msg.linear.x = 0.0005 * math.sin(1e-9 * 2 * math.pi * 10 * (current_time - start_time))
            # cmd_vel_msg.linear.y = 0.0005 * math.sin(1e-9 * 2 * math.pi * 10 * (current_time - start_time))
            # self.move_robot_callback(cmd_vel_msg)
            # rospy.sleep(0.005)

            # adjust_pose_msg.position.x = - adjust_pose_msg.position.x
            # adjust_pose_msg.position.y = - adjust_pose_msg.position.y
            # self.adjust_pose_callback(adjust_pose_msg)

            adjust_pose_vec[:2] = np.multiply(-1, adjust_pose_vec[:2])
            if index_i==0:
                adjust_pose_vec[2] = adjust_pose_vec[2] + 0.05
                index_i = 1
            else:
                adjust_pose_vec[2] = adjust_pose_vec[2] + 0.0002
            
            command_posistion = [start_pose.position.x, start_pose.position.y, start_pose.position.z] + np.matmul(rot_mat, adjust_pose_vec)
            command_pose.position.x = command_posistion[0]
            command_pose.position.y = command_posistion[1]
            command_pose.position.z = command_posistion[2]
            self.move_to_pose(command_pose)

        # cmd_vel_msg.linear.z = 0.0
        # self.move_robot_callback(cmd_vel_msg)

        return True
        # while self.robot.get_tcp_force

        # if activate_flag.activate == True:
        #     self.robot.set_digital_out(8, True)
        #     # self.perform_drill()
        #     # counter = 0
        #     # while(counter < 7):
        #     #     if (self.robot.get_analog_in(2) < 0.65):
        #     #         counter = counter + 1
        #     #     rospy.sleep(0.05)
        #     rospy.sleep(8)
        #     # rospy.sleep(0.3)
        #     self.robot.set_digital_out(8, False)    
        #     return True        
        # else:
        #     self.robot.set_digital_out(8, False) 
        #     self.robot.get_digital_out

        #     return False

    def retract_split_pin_cb(self, msg):
        self.set_TCP('pressure_ft')

        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.z = -0.01
        # self.move_robot_callback(cmd_vel_msg)
        # rospy.sleep(2.5)


        # cmd_vel_msg.linear.z = 0.00
        # self.move_robot_callback(cmd_vel_msg)

        req = moveRobot()
        req.frame = 'pressure_ft'

        # if self.pre_drill_split_pin_pose.position.z != 0:
        #     req.target_pose = self.pre_drill_split_pin_pose
        #     self.moveUR_cb(req, slow=True)

        #     req.target_pose = self.pre_insertion_split_pin_pose
        #     self.moveUR_cb(req, slow=True)
        # elif self.pre_insertion_split_pin_pose.position.z != 0:
        #     req.target_pose = self.pre_insertion_split_pin_pose
        #     self.moveUR_cb(req, slow=True)

        if False:
            pass
        else:
            start_pose = copy.deepcopy(self.pressure_ft_pose.pose)
            rot_mat = R.from_quat([start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]).as_matrix()
            command_pose = Pose()
            command_pose.orientation.w = start_pose.orientation.w
            command_pose.orientation.x = start_pose.orientation.x
            command_pose.orientation.y = start_pose.orientation.y
            command_pose.orientation.z = start_pose.orientation.z
            # adjust_pose_vec = [0.00005, 0.00005, 0.0002]
            adjust_pose_vec = [0.0, 0.0, 0.0002]
            command_pose.position
            rospy.sleep(0.1)

            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time) < 11:
                adjust_pose_vec[:2] = np.multiply(-1, adjust_pose_vec[:2])
                adjust_pose_vec[2] = adjust_pose_vec[2] - 0.0002
                
                command_posistion = [start_pose.position.x, start_pose.position.y, start_pose.position.z] + np.matmul(rot_mat, adjust_pose_vec)
                command_pose.position.x = command_posistion[0]
                command_pose.position.y = command_posistion[1]
                command_pose.position.z = command_posistion[2]
                self.move_to_pose(command_pose)

        # req.frame = 'davis'
        # req.target_pose = self.pre_insertion_davis_pose
        # self.moveUR_cb(req)
        
        return True


    def visual_split_pin_cb(self, activate_flag):

        self.pre_insertion_davis_pose = self.camera_pose.pose
        self.move_PF_to_cam(True)
        self.update_poses()
        self.pre_insertion_split_pin_pose = self.pressure_ft_pose.pose
        self.split_pin_cb(True)

        return True


    def perform_drill(self):
        #TODO: remove
        # AIR PRESSURE : 90 PSI
        # OIL DAMPER : 9

        # Constants

        [v0, v1] = [9.46, 0.11]
        [d0, d1] = [0.00, 12.70]
        filter_size = 11
        filter_ord = 2

        # Variables

        drill_depth_reached = False
        drill_success = True
        bin_t = []
        bin_d = []
        T0 = rospy.get_time()

        while (True):

            # break

            ti = rospy.get_time() - T0

            print(ti)
            if ti > 20.0:
                drill_success = False
                break

            vi = self.robot_controller.get_analog_input()
            di = (d1 - d0) / (v1 - v0) * (vi - v0) + d0

            bin_t.append(ti)
            bin_d.append(di)

            displacement = Point()
            displacement.x = 0  # Lower Limit
            displacement.y = di
            displacement.z = 0  # Upper Limit

            # Run-out Reached
            n = 10
            if len(bin_d) >= n and drill_depth_reached == False:

                x = bin_t[len(bin_t) - n:len(bin_t)]
                y = bin_d[len(bin_t) - n:len(bin_t)]
                a, b = linear_fit(x, y)

                if a < 0.05 and bin_d[-1] > 2.0:
                    drill_depth_reached = True
                    T1 = rospy.get_time()

            if drill_depth_reached and rospy.get_time() - T1 > 0.5:
                break

            # if len(bin_t) > filter_size:
            #
            #     win_t = np.asarray(bin_t[len(bin_t)-filter_size:len(bin_t)])
            #     win_d = np.asarray(bin_d[len(bin_t)-filter_size:len(bin_t)])
            #
            #     dt = (win_t[-1] - win_t[0])/(filter_size-1)
            #
            #     di = savitzky_golay(win_d, window_size=filter_size, order=filter_ord, deriv=0)[filter_size/2]
            #     fi = np.dot(savitzky_golay(win_d, window_size=filter_size, order=filter_ord, deriv=1), 1.0/dt)[filter_size / 2]
            #
            #     # Publish
            #
            #     feedrate = Point()
            #     feedrate.x = 0  # Lower Limit
            #     feedrate.y = fi
            #     feedrate.z = 0  # Upper Limit
            #     self.pub_feedrate.publish(feedrate)
            #
            #     # Run-out Reached
            #
            #     if not drill_depth_reached and np.abs(fi) < 0.05:
            #         T1 = rospy.get_time()
            #         drill_depth_reached = True
            #
            #     if drill_depth_reached and rospy.get_time() - T1 > 1.00:
            #         break

            rospy.sleep(0.05)

        now = datetime.datetime.now()
        return drill_success   

    
if __name__ == '__main__':
    # robot = UrRtde("192.168.50.110")
    # robot = AbbRobot('192.168.125.1')
    robot = MitsubishiRobot('192.168.0.20')    
    ros_robot = RosRobot(robot)
    _thread.start_new_thread( ros_robot.run_node, () )
    _thread.start_new_thread( ros_robot.run_controller, () )

    while not rospy.is_shutdown():
        pass
    exit()