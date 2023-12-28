#!/usr/bin/python3
import numpy as np
# import urx
import cv2
from cv2 import aruco
import sys
import pickle
import os
import math
import time
import rospy
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import date
from ros_robot import RosRobot
from multiprocessing import Process
import _thread
from ur_rtde import UrRtde
from abb_ros import AbbRobot
import random
import copy
import rospkg
# from mitsubishi_ros import MitsubishiRobot

'''
CALIBRATION WITH CHARUCO BOARD - THINGS THAT NEED TO BE CHANGED - SEMI-AUTO(USE CTRL+F)
---------------------------------------------------------------
1. base_to_marker
2. self.ros_image_topic
3. Charuco properties
   self.charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
   self.CHARUCO_BOARD = aruco.CharucoBoard_create(
           squaresX=11,
           squaresY=8,
           squareLength=0.015,
           markerLength=0.012,
           dictionary=self.charuco_dict)
4. self.dump_file_name
5. self.calibration_data_dir
6. self.robot.set_TCP('davis') --> use your camera frame
7. Calibration specifications
'''

checkerboard_to_center = np.array([0.135, 0.135, 0]).reshape(3, -1) #KU big one
# checkerboard_to_center = np.array([0.025, 0.02, 0]).reshape(3, -1) #KU small one


base_to_marker = np.array([[-1, 0,  0, 0.055], 
                           [ 0, 1,  0, -0.53], 
                           [ 0, 0, -1,     0], 
                           [ 0, 0,  0,     1]])#KU UR config
# base_to_marker = np.array([[-1, 0, 0, -0.095], [0, 1, 0, -0.925], [0,0, -1, 0.2], [0,0,0,1]])#KU ABB config

# W  = '\033[0m'  # white (normal)
# R  = '\033[31m' # red
# G  = '\033[32m' # green
# O  = '\033[33m' # orange
# B  = '\033[34m' # blue
# P  = '\033[35m' # purple

class robot_camera_calibration:

    def __init__(self, robot_ip, chess_size, calibration_file, mode='auto'):
        time.sleep(0.2)
        
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('ros_robot_pkg')
        rospy.loginfo('Package path = %s', self.pkg_path)
        
        self.ur_robot = UrRtde("192.168.50.110")
        # self.mit_robot = MitsubishiRobot('192.168.0.20')    
        self.robot = RosRobot(self.ur_robot)
        # self.abb_robot = AbbRobot('192.168.125.1')
        # self.robot = RosRobot(self.abb_robot)
        
        # self.ros_image_topic = "/debur_cam/image_raw"
        # self.ros_image_topic = "/dvs/image_raw"
        self.ros_image_topic = "/ardu_cam/image_raw"  # ov7521
        self.cv_bridge = CvBridge()
        self.overlayed_charuco_pub = rospy.Publisher('overlayed_image', Image, queue_size=1)

        self.scale_factor = 1.0 #1

        #Checkerboard properties
        self.checkerboard_dim = chess_size
        self.checkerboard_size = 3
        self.object_points = np.zeros((self.checkerboard_dim[0]*self.checkerboard_dim[1],3), np.float32)
        self.object_points[:,:2] = 3*np.mgrid[0:self.checkerboard_dim[0], 0:self.checkerboard_dim[1]].T.reshape(-1,2)

        #Aruco properties
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.aruco_size = 0.0075 #>>>????????
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5#5
        self.aruco_params.cornerRefinementMinAccuracy = 0.01
        self.aruco_params.cornerRefinementMaxIterations =100

        #Charuco properties
        self.charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.CHARUCO_BOARD = aruco.CharucoBoard_create(
                squaresX=8,
                squaresY=11,
                squareLength=0.022,
                markerLength=0.016,
                dictionary=self.charuco_dict)

        # self.charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        # self.CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=8,
        #         squaresY=12,
        #         squareLength=0.0227,
        #         markerLength=0.0198,
        #         dictionary=self.charuco_dict) #KU big one (old)
        # self.CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=8,
        #         squaresY=11,
        #         squareLength=0.005,
        #         markerLength=0.004,
        #         dictionary=self.charuco_dict) #KU small one
        # self.CHARUCO_BOARD = aruco.CharucoBoard_create(
        #         squaresX=8,
        #         squaresY=10,
        #         squareLength=0.01963,
        #         markerLength=0.00986,
        #         dictionary=self.charuco_dict) #STRATA

        self.image_counter = 1
        # self.images_directory = 'new_debur_cam_calibration/'
        # self.images_directory = 'tactile_calibration/'
        self.images_directory = ''

        self.dump_file_name = 'tactile_ov7521_' + str(date.today()) + '.pickle'
        # self.dump_file_name = 'tactile_calibration_data' + str(date.today()) + '.pickle'
        # self.dump_file_name = 'debur_cam_calibration_data' + str(date.today()) + '.pickle'

        self.mtx = []
        self.dist = []

        #Calibration specifications for big KU aruco
        # self.max_angle = 0.1#0.4
        # self.N_cycle = 5#5
        # self.N_pose_per_cycle = 20 #20
        # self.radius = [0.28, 0.32]


        #Calibration specifications for small KU aruco
        self.max_angle = 0.4
        self.N_cycle = 5#5
        self.N_pose_per_cycle = 20 #20
        self.radius = [0.10, 0.15]

        self.dump_data_list = []

        self.calibration_data_dir = '/calibration_data/tactile_ov7521/'
        self.calibration_poses = [] #camera poses relative to aruco board
        self.setup_calibration_poses() 

        if mode=='auto':
            self.load_calibration_files(calibration_file)

    def save_pickle(self):
        pickle.dump(self.dump_data_list, open(self.pkg_path + self.calibration_data_dir + self.dump_file_name, 'wb'))
        rospy.loginfo("Pickle File Saved! %s", self.pkg_path + self.calibration_data_dir + self.dump_file_name)

    def save_image(self, image_name, frame):
        cv2.imwrite(self.pkg_path + self.calibration_data_dir + 'images/' + image_name, frame)
        rospy.loginfo("Image Saved! %s", self.pkg_path + self.calibration_data_dir + 'images/' + image_name)

    def load_calibration_files(self, calibration_file):
        #load camera calibration
        my_file = open(calibration_file, 'rb')
        calibration_results = pickle.load(my_file)

        self.mtx = calibration_results['camera_matrix']
        self.dist = calibration_results['distortion_coefficients']

    def setup_calibration_poses(self):

        #center pose
        transformation_matrix = np.eye(4)
        transformation_matrix[2,3] = -self.radius[0]
        self.calibration_poses.append(transformation_matrix)

        for i in range(self.N_cycle):
            theta = (i+1) * self.max_angle / self.N_cycle
            for j in range(self.N_pose_per_cycle):
                phi = j * 2 * math.pi / self.N_pose_per_cycle

                rx = theta * math.cos(phi)
                ry = theta * math.sin(phi)
                rz = 0.5 * (random.random()-0.5)

                transformation_matrix = np.eye(4)
                transformation_matrix[:3,:3] = R.from_rotvec([rx, ry, rz]).as_matrix().transpose()
                transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3,:3], np.array([0.02 * (random.random() - 0.5), 0.02 * (random.random() - 0.5), -self.radius[0] - random.random() * (self.radius[1] - self.radius[0])])).reshape(3) #for big KU ARUCO
                # transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3,:3], np.array([0.02 * (random.random() - 0.5), 0.02 * (random.random() - 0.5), -self.radius[0] - random.random() * (self.radius[1] - self.radius[0])])).reshape(3) #for big small ARUCO
                self.calibration_poses.append(transformation_matrix)

    def getRosImage(self):

        ros_image = rospy.wait_for_message(self.ros_image_topic, Image)
        # print("Here 2")
        image = self.cv_bridge.imgmsg_to_cv2(ros_image)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        image = cv2.GaussianBlur(image, (11, 11), 1)
        ros_image_blurred = self.cv_bridge.cv2_to_imgmsg(image)
        # image = cv2.resize(image, None, fx=self.scale_factor, fy=self.scale_factor)
            
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(image, (11, 11), 1)

        # gray = copy.deepcopy(image)

        return ros_image_blurred, gray_blurred

    def getArucoPose(self, input_image):

        corners, ids, rejectedImgPoints = aruco.detectMarkers(input_image, self.aruco_dict, parameters=self.aruco_params)

        rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.mtx, self.dist)

        if not np.any(rvecs==None):
            rotation = R.from_rotvec(rvecs[0])
            rotmax = rotation.as_matrix()

            return np.array(tvecs[0]), rotmax
            
        else:
            return None, None

    def getChArucoPose(self, input_image, color_img: Image):
        
        gray_image = input_image
        corners = ids = []
        response = 0    
        
        # while ids is None:
        #     color_img, image = self.getRosImage()
        #     # input_image = cv2.GaussianBlur(input_image, (5, 5), 1)
        #     # cv2.imwrite("/home/noetic/workspace/a.png", input_image)
            
        #     corners, ids, _ = aruco.detectMarkers(
        #                 image=image,
        #                 dictionary=self.charuco_dict,
        #                 parameters=self.aruco_params)
            
        #     if ids is None:
        #         input('No arucos found, update ur pose manually and try again.')
        #         continue
            
        #     print(not(ids is None))
            
        while len(ids) < 4 or ids is None:
            
            corners, ids, _ = aruco.detectMarkers(
                        image=gray_image,
                        dictionary=self.charuco_dict,
                        parameters=self.aruco_params)
            
            if ids is None:
                color_img, gray_image = self.getRosImage()
                corners = ids = []
                continue
            
            # cv2.imwrite("/home/noetic/workspace/a.png", image)
            if len(ids) < 4:
                color_img, gray_image = self.getRosImage()
                corners = ids = []
                input('\033[33mFound {} aruco(s), update ur pose manually and try again.\033[0m'.format(len(ids)))
                # continue

            response, chararuco_corners, chararuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray_image,
                board=self.CHARUCO_BOARD) 

        print('\033[32mFound {} arucos. Moving to next pose.\033[0m'.format(len(ids)))
        # Outline the aruco markers found in our query image
        color_img = self.cv_bridge.imgmsg_to_cv2(color_img)
        print(ids)
        print(f'#: {len(ids)}')
        overlayed_image = aruco.drawDetectedMarkers(
            image=color_img, 
            corners=corners, 
            ids=ids, 
            borderColor=(0,255,0))
        self.overlayed_charuco_pub.publish(self.cv_bridge.cv2_to_imgmsg(overlayed_image, encoding="rgb8")
        )
        return gray_image, overlayed_image
        # print(not(ids is None))
        # response = 0    

        # while response < 4:
            
        #     response, chararuco_corners, chararuco_ids = aruco.interpolateCornersCharuco(
        #         markerCorners=corners,
        #         markerIds=ids,
        #         image=image,
        #         board=self.CHARUCO_BOARD) 
            
        #     # print(response)
            
        #     if response < 4:
        #         color_img, image = self.getRosImage()
        #         input(f'Found {response} aruco(s), update ur pose manually and try again.')
            
        #     color_img, image = self.getRosImage()

        #     corners, ids, _ = aruco.detectMarkers(
        #                 image=image,
        #                 dictionary=self.charuco_dict,
        #                 parameters=self.aruco_params)
            
        #     while ids is None:
                
        #         color_img, image = self.getRosImage()
        #         # image = cv2.GaussianBlur(image, (3, 3), 5)
                
        #         corners, ids, _ = aruco.detectMarkers(
        #                     image=image,
        #                     dictionary=self.charuco_dict,
        #                     parameters=self.aruco_params)

        #     response, chararuco_corners, chararuco_ids = aruco.interpolateCornersCharuco(
        #         markerCorners=corners,
        #         markerIds=ids,
        #         image=image,
        #         board=self.CHARUCO_BOARD)
        

        # print(response)
        
        # rvec = np.empty(shape=(1,))
        # tvec = np.empty(shape=(1,))
        # retval, rvecs, tvecs = aruco.estimatePoseCharucoBoard(chararuco_corners, chararuco_ids, self.CHARUCO_BOARD, self.mtx, self.dist, rvec, tvec)

        # aruco_correctionmatrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # rotation = R.from_rotvec(np.array(rvecs).reshape(3))
        # rotmax = np.matmul(aruco_correctionmatrix, rotation.as_matrix())

        
        # translation = np.array(tvecs).reshape(3, -1) + np.matmul(rotmax, checkerboard_to_center)
        # return translation, rotmax
    
    
    def getCheckerboarPose(self, input_image):

        while not self.check_checkerboard(input_image):
            input('Checkerboard not found, manually update ur pose and try again')
            _, input_image = self.getRosImage()

        #obtaining corners in chessboard
        ret, corners = cv2.findChessboardCorners(input_image, self.checkerboard_dim)

        if ret:
            #refine corner estimation
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(input_image, corners, (5,5), (-1,-1), criteria)

            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv2.solvePnP(self.object_points, corners2, self.mtx, self.dist)


            if not np.any(rvecs==None):
                rotation = R.from_rotvec(np.array(rvecs).reshape(3))
                rotmax = rotation.as_matrix()

                translation = np.array(tvecs/100).reshape(3, -1) + np.matmul(rotmax, checkerboard_to_center)
                return translation, rotmax
                
            else:
                return None, None
        else:
                return None, None

    def check_checkerboard(self, input_image):
        #obtaining corners in chessboard
        ret, corners = cv2.findChessboardCorners(input_image, self.checkerboard_dim)

        return ret
    
    def getEEPose(self):
        robot_pose = self.robot.robot_controller.get_pose()

        tvec = [robot_pose[0], robot_pose[1], robot_pose[2]]

        return np.array(tvec), R.from_rotvec(robot_pose[3:6]).as_matrix()

    def dumpData(self, image_name_list, ee_pose_list, aruco_pose_list):

        return 

    def performAutoCalibRoutine(self):
        rospy.sleep(2)

        #Get initial aruco pose
        _, base_to_camera = self.robot.wait_for_transform('ur_base', 'davis')

        original_img, input_img = self.getRosImage()        

        current_marker_tvec, current_marker_rot = self.getChArucoPose(input_img)
        current_marker_transformation = np.vstack([np.c_[current_marker_rot, current_marker_tvec.reshape(3,-1)], [0, 0, 0, 1]])
        print(current_marker_transformation)

        base_to_marker = self.robot.add_transformations(base_to_camera, current_marker_transformation)
        self.robot.kinematics.set_transform('ur_base', 'aruco', base_to_marker, mode='static')

        input("Check rviz, then proceed ...")

        #start routine 
        for target_pose in self.calibration_poses:
            print('Target pose:', target_pose)

            self.robot.kinematics.set_transform('aruco', 'desired_cam', target_pose, mode='static')
            rospy.sleep(0.2)
            _, base_to_target = self.robot.receive_transform('ur_base', 'desired_cam')
            
            self.robot.set_TCP('davis')
            pose_msg = self.robot.transformation_matrix_to_pose(base_to_target)
            self.robot.move_to_pose(pose_msg)
            rospy.sleep(3)


            original_img, input_img = self.getRosImage()        

            current_marker_tvec, current_marker_rot = self.getChArucoPose(input_img)
            current_marker_transformation = np.vstack([np.c_[current_marker_rot, current_marker_tvec.reshape(3,-1)], [0, 0, 0, 1]])

            current_EE_tvec, current_EE_rot = self.getEEPose()
            current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])

            image_name = self.images_directory + str(self.image_counter) + '.png'
            self.image_counter = self.image_counter + 1

            dump_data = {'ee_pose': current_ee_transformation,
                        'marker_pose': current_marker_transformation, 
                        'image_dir': image_name}

            self.dump_data_list.append(dump_data)
            cv2.imwrite(image_name, original_img)
        
        pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))

    def performManualCalibRoutine(self):
        #manual mode
        while not rospy.is_shutdown():
            user_input = input("Press r to register new point, press d to dump data and close...")
            if user_input == 'r':
                original_img, input_img = self.getRosImage()
                check_state = self.check_checkerboard(input_img)

                if not check_state:
                    print("no marker detected")
                    continue
                else:        
                    current_EE_tvec, current_EE_rot = self.getEEPose()
                    current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])
                    print("current ee transformation:", current_ee_transformation)

                    image_name = self.images_directory + str(self.image_counter) + '.png'
                    print("image_name: ", image_name)
                    self.image_counter = self.image_counter + 1

                    dump_data = {'ee_pose': current_ee_transformation, 
                                'image_dir': image_name}

                    self.dump_data_list.append(dump_data)
                    cv2.imwrite(image_name, original_img)

            elif user_input =='d':            
                pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))
                break

    def performSemiAutoCalibRoutine(self):
        rospy.sleep(1)

        self.robot.kinematics.set_transform('ur_base', 'aruco', base_to_marker, mode='static')
        self.robot.kinematics.set_transform('aruco', 'desired_cam', self.calibration_poses[0], mode='static')
        input("Check rviz, then proceed ...")

        print(len(self.calibration_poses))
        #start routine 
        for target_pose in self.calibration_poses:
            # round
            for i in range(target_pose.shape[0]):
                for j in range(target_pose.shape[1]):
                    target_pose[i,j] = round(target_pose[i,j], 4)

            print('Target pose:', target_pose)

            self.robot.kinematics.set_transform('aruco', 'desired_cam', target_pose, mode='static')
            rospy.sleep(0.2)
            _, base_to_target = self.robot.kinematics.receive_transform('ur_base', 'desired_cam')
            
            self.robot.set_TCP('davis')   # for ids_priscilla
            # self.robot.set_TCP('camera_color_optical_frame')    # for d435
            pose_msg = self.robot.kinematics.transformation_matrix_to_pose(base_to_target)
            self.robot.move_to_pose(pose_msg)
            print("Here 1")
            
            rospy.sleep(1)

            original_img, input_img = self.getRosImage() 

            # while not self.check_checkerboard(input_img):
            #     input('Checkerboard not found, manually update ur pose and try again')
            #     _, input_img = self.getRosImage()   
            # input_img = cv2.GaussianBlur(input_img, (11, 11), 2)

            captured_image, overlayed_image = self.getChArucoPose(input_img, original_img) 
            # original_img, input_img = self.getRosImage() 
            # captured_image = cv2.resize(captured_image , None, fx=self.scale_factor, fy=self.scale_factor)
            # cv2.imwrite("/home/ku-user/calibration_ws/input_img.jpg", input_img)

            current_EE_tvec, current_EE_rot = self.getEEPose()
            current_ee_transformation = np.vstack([np.c_[current_EE_rot, current_EE_tvec.reshape(3,-1)], [0, 0, 0, 1]])

            image_name = self.images_directory + str(self.image_counter) + '.png'
            self.image_counter = self.image_counter + 1

            dump_data = {'ee_pose': current_ee_transformation,
                        'image_dir': image_name}

            self.dump_data_list.append(dump_data)
            self.save_image(image_name, captured_image)
        
        self.save_pickle()
        # pickle.dump(self.dump_data_list, open(self.dump_file_name, 'wb'))



    def cleanup(self):
        self.robot.close()


if __name__ == '__main__':
    # charuco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    # CHARUCO_BOARD = aruco.CharucoBoard_create(
    #             squaresX=2,
    #             squaresY=2,
    #             squareLength=0.023,
    #             markerLength=0.02,
    #             dictionary=charuco_dict)

    # img = CHARUCO_BOARD.draw((2480, 2480))
    # clr_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # black_idx = np.where((clr_img == [0, 0, 0]).all(axis=2))
    # clr_img[black_idx] = [0, 255, 0]
    # cv2.imwrite('charuco.jpg', img)

    robot = robot_camera_calibration("192.168.50.110", (8,5), 'calibration_2021-01-14.pickle', 'semi_auto')
    _thread.start_new_thread( robot.robot.run_node, () )
    # _thread.start_new_thread( robot.performAutoCalibRoutine, () )
    _thread.start_new_thread( robot.performSemiAutoCalibRoutine, () )

    
    while not rospy.is_shutdown():
        pass
    robot.cleanup()

    exit()
