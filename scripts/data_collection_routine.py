#!/usr/bin/python3
import json
import numpy as np
# import urx
import cv2
from cv2 import aruco
import os
import math
import rospy
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import datetime
from ros_robot import RosRobot
from multiprocessing import Process
from ur_rtde import UrRtde
from abb_ros import AbbRobot
import random
import copy
from utils import *
# from mitsubishi_ros import MitsubishiRobot
import pathlib
from PIL import Image as PIL_Image

class CameraCalibrationDataCollection:
    def json_file_to_dict(self, calibration_config_file: str) -> dict:
        if not calibration_config_file.endswith('.json'):
            calibration_config_file += '.json'
        try:
            with open(calibration_config_file) as inputfile:
                return json.load(inputfile)
        except Exception as e:
            print(e)
            print(RED + "Error: Config file '" + calibration_config_file + "' not found." + RST)
            exit()
            
    def load_calibration_config(self, calibration_config_file: str) -> dict:
        ## LOAD CALIBRATION CONFIGURATION DATA
        self.calibration_config = self.json_file_to_dict(calibration_config_file)

        ## ROBOT INFO
        self.robot_name = self.calibration_config['robot']['name']
        self.robot_ip = self.calibration_config['robot']['ip']

        ## CALIBRATION TARGET DATA
        # allowing this to be more flexible for different targets
        # TODO: ugly code, needs to be cleaned up
        calib_target_keys = []
        for key in self.calibration_config['calibration_target']:
            if key == 'type':
                calib_target_keys.append('target_type')
                setattr(self, 'target_type', self.calibration_config['calibration_target'][key])
            elif key == 'size':
                calib_target_keys.append('target_size')
                setattr(self, 'target_size', (self.calibration_config['calibration_target'][key][0], 
                                              self.calibration_config['calibration_target'][key][1]))
            elif key == 'target2base':
                calib_target_keys.append('base_T_target')
                setattr(self, 'base_T_target', self.calibration_config['calibration_target'][key])
            elif key == 'aruco_dict':
                self.target_aruco_dict = ARUCO_DICT[self.calibration_config['calibration_target'][key]]
            else:
                calib_target_keys.append(key)
                setattr(self, key, self.calibration_config['calibration_target'][key])


        ## CALIBRATION DATA
        self.calibration_data_dir_relative = os.path.join(
            "../calibration_data", 
            self.calibration_config['calibration_data']['project_name']
            )
        self.calibration_data_dir_abs = str(pathlib.Path(
            os.path.join(os.path.dirname(__file__), self.calibration_data_dir_relative)
            ).resolve())
        
        self.ros_image_topic = self.calibration_config['calibration_data']['image_topic']
        self.data_collection_setup = self.calibration_config['calibration_data']['data_collection_setup']
        self.dump_file_name = self.calibration_config['calibration_data']['output_file_name']# + "_" + datetime.now().strftime("%Y-%m-%d-%H-%M") + '.json'
        self.calibration_config['calibration_data'].update({'output_file_name': self.dump_file_name})
        self.calibration_config['calibration_data'].update({'data_relative_dir': self.calibration_data_dir_relative})
        self.images_dir_abs = os.path.join(self.calibration_data_dir_abs, "images")
        self.images_dir_relative = os.path.join(self.calibration_data_dir_relative, "images")
        
        if not os.path.exists(self.calibration_data_dir_abs):
            os.makedirs(self.calibration_data_dir_abs)
        
        if not os.path.exists(self.images_dir_abs):
            os.makedirs(self.images_dir_abs)
        
        verbose = False 
        #TODO: make this generalizable for different targets
        if verbose:
            print('--------------------------------------------------------')
            print(f'########  R O B O T   I N F O  ########')
            print(f'           robot_name: {self.robot_name}')
            print(f'             robot_ip: {self.robot_ip}')
            print()
            print(f'########  C A L I B R A T I O N   D A T A  ########')
            print(f' calibration_data_dir: ')
            print(f'     ├── relative_dir: {self.calibration_data_dir_relative}')
            print(f'     └── absolute_dir: {self.calibration_data_dir_abs}')
            print(f'      ros_image_topic: {self.ros_image_topic}')
            print(f'data_collection_setup: {self.data_collection_setup}')
            print(f'       dump_file_name: {self.dump_file_name}')
            print()
            print(f'########  C A L I B R A T I O N   T A R G E T  ########')
            print(f'          target_type: {self.target_type}')
            print(f'          target_size: {self.target_size}')
            print(f'         checker_length: {self.checker_length}')
            print(f'          marker_length: {self.marker_length}')
            print(f'       legacy_pattern: {self.legacy_pattern}')
            print(f'                 blur: {self.blur}')
            print(f'        base_T_target: {self.base_T_target}')
            print(f'    target_aruco_dict: {self.target_aruco_dict}')
            print('--------------------------------------------------------')
            
        # check if the directory already has images
        self.use_existing_data = False #self.calibration_config['calibration_data']['use_existing_data']# + "_" + datetime.now().strftime("%Y-%m-%d-%H-%M") + '.json'
        self.data_collected = False

        num_imgs = len(os.listdir(self.images_dir_abs))
        if num_imgs:
            print(f"{ORN}Directory '{self.images_dir_abs}' already contains {num_imgs} images. Consider taking a back up in case you need them. Proceeding will overwrite existing images.{RST}")
            if input("Proceed to collecting new images? [Y/N]").lower() == 'n':
                print("Existing images will be used for calibration ...")
                self.use_existing_data = True
                self.data_collected = True
            
        
        return self.calibration_config
        
    def __init__(self, calibration_config_file: str):#, calibration_file, mode='auto'):
        ## LOAD CALIBRATION CONFIGURATION DATA
        self.calibration_config = self.load_calibration_config(calibration_config_file)
        
        self.cv_bridge = CvBridge()
        self.image_markers_publisher = rospy.Publisher(
            'markers_image', Image, queue_size=1)
        self.scale_factor = 1.0  # 1

        # TODO: checker and aruco have not been tested, 
        # only thhe charuco option is currently functional
        if self.target_type == 'checker':
            # Checkerboard properties
            self.checkerboard_dim = self.target_size
            self.checkerboard_size = 3
            self.object_points = np.zeros(
                (self.checkerboard_dim[0]*self.checkerboard_dim[1], 3), np.float32)
            self.object_points[:, :2] = 3*np.mgrid[0:self.checkerboard_dim[0],
                                                0:self.checkerboard_dim[1]].T.reshape(-1, 2)
        elif self.target_type == 'aruco':
            # Aruco properties
            self.aruco_dict = aruco.getPredefinedDictionary(self.target_aruco_dict)
            self.aruco_params = aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.cornerRefinementMinAccuracy = 0.0001
            self.aruco_params.cornerRefinementMaxIterations = 10000
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)


        elif self.target_type == 'charuco':
            # Charuco properties
            self.charuco_params = cv2.aruco.CharucoParameters()
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.cornerRefinementMinAccuracy = 0.0001
            self.aruco_params.cornerRefinementMaxIterations = 10000
            self.refine_params = cv2.aruco.RefineParameters()

            self.charuco_dict = aruco.getPredefinedDictionary(self.target_aruco_dict)
            self.charuco_board = aruco.CharucoBoard(
                self.target_size,
                squareLength=self.checker_length,
                markerLength=self.marker_length,
                dictionary=self.charuco_dict)
            self.charuco_board.setLegacyPattern(self.legacy_pattern)

            self.charuco_detector = cv2.aruco.CharucoDetector(
                board=self.charuco_board,
                charucoParams=self.charuco_params,
                detectorParams=self.aruco_params,
                refineParams=self.refine_params,
            )
            
        self.data_counter = 0

        self.max_angle = self.data_collection_setup[0]
        self.n_cycle = self.data_collection_setup[1]
        self.n_pose_per_cycle = self.data_collection_setup[2]
        self.radius = [self.data_collection_setup[3],
                       self.data_collection_setup[4]]

        self.calibration_data_list = {}

        # self.calibration_data_dir_relative = os.path.join(os.path.dirname(__file__), self.calibration_data_dir_relative)
        self.calibration_poses = []  # camera poses relative to aruco board
        self.setup_calibration_poses()
        
        if self.use_existing_data:
            return
        
        if self.robot_name == 'UR10':
            self.robot = RosRobot(UrRtde(self.robot_ip))
        elif self.robot_name == 'ABB':
            self.robot = RosRobot(AbbRobot(self.robot_ip))
        elif self.robot_name == 'Mitsubishi':
            #TODO: Add support for mitsubishi robot
            print('self.robot = RosRobot(MitsubishiRobot(self.robot_ip))')
            pass
        else:
            print("No robot selected. Exiting...")
            exit()
        
    def save_to_json_file(self, data_object, file_name: str):
        if not file_name.endswith('.json'):
            file_name = file_name + '.json'
            
        json_string = json.dumps(data_object, ensure_ascii=False, indent=4)
        file_path = os.path.join(self.calibration_data_dir_relative, file_name)
        
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(json_string)
        
        # f.close()
        rospy.loginfo("JSON file saved! %s", os.path.join(self.calibration_data_dir_abs, file_name))

    def save_image(self, image_name: str, frame: np.ndarray):
        if not image_name.endswith(".png"):
            image_name = image_name + ".png"
            
        cv2.imwrite(os.path.join(self.images_dir_relative, image_name), frame)
        rospy.loginfo("Image saved! %s", os.path.join(self.images_dir_abs, image_name))
        # rospy.loginfo("Image saved! %s", image_name)

    def get_image_info(self, image):
        try:
            # Open the image file
            with PIL_Image.fromarray(image) as img:
                # Get basic information about the image
                info = {
                    'format': img.format,
                      'mode': img.mode,
                      'size': img.size,
                      'info': img.info
                }
                return info
        except Exception as e:
            return f"Error: {e}"

    def setup_calibration_poses(self):

        # center pose
        transformation_matrix = np.eye(4)
        transformation_matrix[2, 3] = -self.radius[0]
        self.calibration_poses.append(transformation_matrix)

        for i in range(self.n_cycle):
            theta = (i+1) * self.max_angle / self.n_cycle
            for j in range(self.n_pose_per_cycle):
                phi = j * 2 * math.pi / self.n_pose_per_cycle

                rx = theta * math.cos(phi)
                ry = theta * math.sin(phi)
                rz = 0.5 * (random.random()-0.5)

                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = R.from_rotvec(
                    [rx, ry, rz]).as_matrix().transpose()
                transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3, :3], np.array([0.02 * (random.random() - 0.5), 0.02 * (
                    random.random() - 0.5), -self.radius[0] - random.random() * (self.radius[1] - self.radius[0])])).reshape(3)  # for big KU ARUCO
                # transformation_matrix[:3, 3] = np.matmul(transformation_matrix[:3,:3], np.array([0.02 * (random.random() - 0.5), 0.02 * (random.random() - 0.5), -self.radius[0] - random.random() * (self.radius[1] - self.radius[0])])).reshape(3) #for big small ARUCO
                self.calibration_poses.append(transformation_matrix)

    def getRosImage(self):

        ros_image = rospy.wait_for_message(self.ros_image_topic, Image)
        cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image)
        info = self.get_image_info(cv_image)
        # print(info)
        
        if info['mode'] == 'L':
            gray_cv_image = cv_image
            # rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        elif 'RGB' in info['mode']:
            # rgb_cv_image = cv_image
            gray_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        
        blur_gray_cv_image = cv2.GaussianBlur(gray_cv_image, (self.blur[0], self.blur[0]), self.blur[1])
        blur_rgb_ros_image = self.cv_bridge.cv2_to_imgmsg(cv2.cvtColor(blur_gray_cv_image, cv2.COLOR_GRAY2RGB))
        
        return blur_rgb_ros_image, blur_gray_cv_image

    def draw_and_publish_markers(self, rgb_ros_image, markerCorners, markerIds):
        markers_cv_image = aruco.drawDetectedMarkers(
            image=self.cv_bridge.imgmsg_to_cv2(rgb_ros_image),
            corners=markerCorners,
            ids=markerIds,
            borderColor=(0, 255, 0))
        self.image_markers_publisher.publish(self.cv_bridge.cv2_to_imgmsg(markers_cv_image, encoding="rgb8"))

    def getChArucoMarkers(self, rgb_ros_image: Image, gray_cv_image: np.ndarray):
        charucoCorners = charucoIds = markerCorners = markerIds = []
        
        while len(markerIds) < 6 or markerIds is None:
            (charucoCorners,
             charucoIds,
             markerCorners,
             markerIds
             ) = self.charuco_detector.detectBoard(image=gray_cv_image)

            if markerIds is None:
                rgb_ros_image, gray_cv_image = self.getRosImage()
                charucoCorners = charucoIds = markerCorners = markerIds = []
                continue

            if len(markerIds) < 6:
                self.draw_and_publish_markers(rgb_ros_image, markerCorners, markerIds)
                rgb_ros_image, gray_cv_image = self.getRosImage()
                input('\033[33mFound {} aruco(s), update ur pose manually and try again.\033[0m'.format(
                    len(markerIds)))
                charucoCorners = charucoIds = markerCorners = markerIds = []
                continue

        print(
            '\033[32mFound {} arucos. Moving to next pose.\033[0m'.format(len(markerIds)))
        # Outline the aruco markers found in our query image
        # print(markerIds)
        # print(f'# of markers: {len(markerIds)}')
        self.draw_and_publish_markers(rgb_ros_image, markerCorners, markerIds)
        
        return gray_cv_image
    
    def getArucoMarkers(self, rgb_ros_image: Image, gray_cv_image: np.ndarray):
        markerCorners = markerIds = []
        while len(markerIds) < 1 or markerIds is None:
            (markerCorners,
             markerIds,
             rejectedImgPoints
             ) = self.detector.detectMarkers(image=gray_cv_image)

            if markerIds is None:

                rgb_ros_image, gray_cv_image = self.getRosImage()
                markerCorners = markerIds = []
                continue

            if markerIds is not None and len(markerIds) > 0:
                print('y')
                # marker_index = 0     
                # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[marker_index], 0.05, camera_matrix, dist_coeffs)
                # # viz
                # cv2.drawFrameAxes( frame, camera_matrix, dist_coeffs, rvec, tvec, length=0.03 )

                self.draw_and_publish_markers(rgb_ros_image, markerCorners, markerIds)
            elif len(rejectedImgPoints) > 0:
                print('found Aruco but rejected. Check params')
        
        return gray_cv_image


    def getEEPose(self):
        robot_pose = self.robot.robot_controller.get_pose()

        tvec = [robot_pose[0], robot_pose[1], robot_pose[2]]

        return np.array(tvec), R.from_rotvec(robot_pose[3:6]).as_matrix()

    def collect_data(self):
        self.robot.kinematics.set_transform(
            'ur_base', self.target_type, np.array(self.base_T_target), mode='static')
        self.robot.kinematics.set_transform(
            self.target_type, 'next_cam_pose', self.calibration_poses[0], mode='static')
        input("Check rviz, then proceed ...")

        print(f'# of poses: {len(self.calibration_poses)}')
        # start routine
        from tqdm.auto import tqdm
        for target_pose in tqdm(self.calibration_poses, desc='Collecting images...'):
            # round
            for i in range(target_pose.shape[0]):
                for j in range(target_pose.shape[1]):
                    target_pose[i, j] = round(target_pose[i, j], 4)

            # print('Target pose:', target_pose)

            self.robot.kinematics.set_transform(
                self.target_type, 'next_cam_pose', target_pose, mode='static')
            # rospy.sleep(0.2)
            _, base_to_target = self.robot.kinematics.receive_transform(
                'ur_base', 'next_cam_pose')

            self.robot.set_TCP('davis')
            pose_msg = self.robot.kinematics.transformation_matrix_to_pose(
                base_to_target)
            self.robot.move_to_pose(pose_msg)
            # rospy.sleep(0.2)

            rgb_ros_image, gray_cv_image = self.getRosImage()

            # gray_cv_image = self.getChArucoMarkers(
            #     rgb_ros_image, gray_cv_image)

            gray_cv_image = self.fetch_target_func()(
                rgb_ros_image, gray_cv_image)
            
            current_EE_tvec, current_EE_rot = self.getEEPose()
            current_ee_transformation = np.vstack(
                [np.c_[current_EE_rot, current_EE_tvec.reshape(3, -1)], [0, 0, 0, 1]])

            image_name = str(self.data_counter) + '.png'

            self.calibration_data_list.update(
                {'data_sample_' + str(self.data_counter): {'ee_pose': current_ee_transformation.tolist(),
                                                           'image': os.path.join(self.images_dir_relative,image_name)}
                 }
                )

            # self.calibration_data_list.update(dump_data)
            self.save_image(image_name, gray_cv_image)
            self.save_to_json_file(self.calibration_data_list, self.dump_file_name)
            self.data_counter += 1
            # input("continue")
        
        if len(self.calibration_data_list) == len(self.calibration_poses):
            self.data_collected = True
            # self.save_to_json_file(self.calibration_config, "calibration_config_updated.json")

    

    def fetch_target_func(self):
        if self.target_type == "charuco":
            return self.getChArucoMarkers
        elif self.target_type == "aruco":
            return self.getArucoMarkers
        
            


    # def cleanup(self):
    #     self.robot.close()


