#!/usr/bin/python3

import json
import pathlib
import numpy as np
import cv2
import os
from cv2 import aruco
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from utils import *

class CameraCalibrator:
    def get_abs_images_dir(self):
            return str(
                pathlib.Path(
                    os.path.join(
                        os.path.dirname(__file__), 
                        self.calibration_config['calibration_data']['data_relative_dir'],
                        "images"
                        )
                    ).resolve()
                )

    def json_file_to_dict(self, json_file: str) -> dict:
        if not json_file.endswith('.json'):
            json_file += '.json'
        try:
            with open(json_file) as inputfile:
                return json.load(inputfile)
        except Exception as e:
            print(e)
            print(RED + "Error: Config txt_file '" + json_file + "' not found." + RST)
            exit()

    def __init__(self, calibration_config: dict):
        # read json_config
        self.calibration_config = calibration_config
        # print(self.calibration_config)
        
        # params
        self.workdir = ""
        self.datadir = self.get_abs_images_dir()

        self.pose_data_file = os.path.join(
            self.datadir[:self.datadir.find('images')], 
            self.calibration_config['calibration_data']['output_file_name']
            )

        self.charuco_legacy_pattern = self.calibration_config['calibration_target']['legacy_pattern']
        self.charuco_board_size = self.calibration_config['calibration_target']['size']
        self.charuco_checker_length = self.calibration_config['calibration_target']['checker_length']
        self.charuco_marker_length = self.calibration_config['calibration_target']['marker_length']
        self.markers_dict = ARUCO_DICT[self.calibration_config['calibration_target']['aruco_dict']]
        
        ## Data
        self.images = np.array(
            [os.path.join(self.datadir, f) for f in os.listdir(self.datadir) if f.endswith(".png")]
        )
        
        order = np.argsort([int(p[p.find('calibration_data'):].split(os.path.sep)[3].split(".")[0]) for p in self.images])
        self.images = self.images[order]
        # self.images = self.images[:9]  # first 9 elements 0:8
        self.image_size = (
            cv2.imread(self.images[0]).shape[0],
            cv2.imread(self.images[0]).shape[1],
        )
        
        self.pose_data = self.json_file_to_dict(self.pose_data_file)
        
        ## Calibration Target Info
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.markers_dict)
        
        self.charuco_board = cv2.aruco.CharucoBoard(
            size=self.charuco_board_size, 
            squareLength=self.charuco_checker_length, 
            markerLength=self.charuco_marker_length, 
            dictionary=self.aruco_dict
            )
        self.charuco_board.setLegacyPattern(self.charuco_legacy_pattern)
        
        self.charuco_params = cv2.aruco.CharucoParameters()
        
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementMinAccuracy = 0.0001
        self.detector_params.cornerRefinementMaxIterations = 10000
        
        self.refine_params = cv2.aruco.RefineParameters()
        
        self.charuco_detector = cv2.aruco.CharucoDetector(
            board=self.charuco_board,
            charucoParams=self.charuco_params,
            detectorParams=self.detector_params,
            refineParams=self.refine_params
        )
        ## Calibration Results
        self.allCharucoCorners = []
        self.allCharucoIds = []
        self.allMarkerCorners = []
        self.allMarkerIds = []
        self.allTvecs = []
        self.allRvecs = []
        self.camera_matrix = []
        self.distortion_coefficients = []
        self.reprojection_error = None
        self.rotation_vectors = []
        self.translation_vectors = []
        self.cam_T_target = []
        self.base_T_tcp = []
        self.tcp_T_cam = []
        self.calibration_results = None
        self.calibration_results_txt  = str(pathlib.Path(os.path.join(self.datadir, '..', 'calibration_results.txt' )).resolve())
        self.calibration_results_json = str(pathlib.Path(os.path.join(self.datadir, '..', 'calibration_results.json')).resolve())
        
    def read_charuco_board(self):
        """
        Charuco board pose estimation.
        """
        print(BLU + "READING CHARUCO BOARD CORNERS AND MARKERS:" + RST)

        # corners, ids, rejectedImgPoints = [], [], []
        fig = plt.figure()
        plt.gcf().set_dpi(300)
        frame_markers = None
        for im in self.images:
            print("=> Processing image {0}".format(im))
            frame = cv2.imread(im)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            (
                charucoCorners,
                charucoIds,
                markerCorners,
                markerIds,
            ) = self.charuco_detector.detectBoard(image=gray)

            if len(charucoIds) >= 6:
            
                # ax = fig.add_subplot(3, 3, int(im.split("/")[1].split(".")[0]))
                frame_markers = aruco.drawDetectedMarkers(
                    frame, markerCorners, markerIds
                )
                frame_markers = cv2.cvtColor(frame_markers, cv2.COLOR_BGR2RGB)
                plt.imshow(frame_markers)
                plt.title(im, fontsize=5)
                plt.axis("off")

                self.allCharucoCorners.append(charucoCorners)
                self.allCharucoIds.append(charucoIds)
                self.allMarkerCorners.append(markerCorners)
                self.allMarkerIds.append(markerIds)
            # decimator+=1
        # plt.show()
        print()

    def calibrate_camera_intrinsics(
        self,
        flags=0,
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            10000,
            np.finfo(float).eps,
        ),
    ):
        """
        Calibrates the camera using the detected corners.
        """
        self.read_charuco_board()
        print("+------------------------------+")
        print(f'|{BLU} INTRINSIC CAMERA CALIBRATION {RST}|')
        print("+------------------------------+")

        cameraMatrixInit = np.array(
            [
                [0.0, 0.0, self.image_size[0] / 2.0],
                [0.0, 0.0, self.image_size[1] / 2.0],
                [0.0, 0.0, 1.0],
            ]
        )

        distCoeffsInit = np.zeros((5, 1))

        (
            self.reprojection_error,
            self.camera_matrix,
            self.distortion_coefficients,
            self.rotation_vectors,
            self.translation_vectors,
            stdDeviationsIntrinsics,
            stdDeviationsExtrinsics,
            perViewErrors,
        ) = cv2.aruco.calibrateCameraCharucoExtended(
            charucoCorners=self.allCharucoCorners,
            charucoIds=self.allCharucoIds,
            board=self.charuco_board,
            imageSize=self.image_size,
            cameraMatrix=cameraMatrixInit,
            distCoeffs=distCoeffsInit,
            flags=flags,
            criteria=criteria,
        )
        np.set_printoptions(suppress=True)
        print(f'* {GRN}Reprojection Error:{RST} {self.reprojection_error}')
        print(f'* {GRN}Camera Matrix{RST}')
        print(f'  {self.camera_matrix[0]}')
        print(f'  {self.camera_matrix[1]}')
        print(f'  {self.camera_matrix[2]}')
        print(f'* {GRN}Distortion Coefficients{RST}')
        print("  " + str([c[0].tolist() for c in self.distortion_coefficients]))
        print()
        
        with open(self.calibration_results_txt, 'w') as txt_file:
            txt_file.write("\n")
            txt_file.write("+------------------------------+\n")
            txt_file.write("| INTRINSIC CAMERA CALIBRATION |\n")
            txt_file.write("+------------------------------+\n")
            txt_file.write(f"* Reprojection Error: {self.reprojection_error}\n")
            txt_file.write("* Camera Matrix\n")
            txt_file.write(f"  {self.camera_matrix[0]}\n")
            txt_file.write(f"  {self.camera_matrix[1]}\n")
            txt_file.write(f"  {self.camera_matrix[2]}\n")
            txt_file.write("* Distortion Coefficients\n")
            txt_file.write("  " + str([c[0].tolist() for c in self.distortion_coefficients]))
            txt_file.write("\n")
      
    def my_estimatePoseSingleMarkers(self, corners, marker_length, mtx, distortion):
        """
        NOT USEDDDD!!
        This will estimate the rvec and tvec for each of the marker corners
        detected by:
        `corners, ids, rejectedImgPoints = detector.detectMarkers(image)`
        @corners: is an array of detected corners for each detected marker
        in the image
        @marker_length: is the size of the detected markers
        @mtx: is the camera matrix
        @distortion: is the camera distortion matrix
        @RETURN list of rvecs, tvecs, and retval (so that it corresponds to
        the old estimatePoseSingleMarkers())
        """
        marker_points = np.array(
            [
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0],
            ],
            dtype=np.float32,
        )

        retval, rvec, tvec = cv2.solvePnP(
            marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE
        )
        # print(rvec)
        return rvec, tvec, retval

    def get_cam_T_target_poses(self, rvecs: list = [], tvecs: list = []):
        # print(BLU + "CALCULATING cam_T_target POSES" + RST)
        
        if len(rvecs) == 0 or len(tvecs) == 0:
            method = 'cv2.aruco.estimatePoseCharucoBoard'
        else:
            method = 'cv2.aruco.calibrateCameraCharucoExtended'
            
        charuco_correction_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        if method == 'cv2.aruco.estimatePoseCharucoBoard':
            for i in range(len(self.images)):
                rvec = np.empty(shape=(1,))
                tvec = np.empty(shape=(1,))
                _, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(
                    self.allCharucoCorners[i], self.allCharucoIds[i], self.charuco_board, self.camera_matrix, self.distortion_coefficients, rvec, tvec
                )
                self.allRvecs.append(rvecs)
                self.allTvecs.append(tvecs)
                p_cam_board = np.array(tvecs).reshape(3,-1)
                rotv_cam_board = np.array(rvecs).reshape(3)
                R_cam_board = Rotation.from_rotvec(rotv_cam_board)
                matrix_cam_board = np.matmul(R_cam_board.as_matrix(), charuco_correction_matrix)
                self.cam_T_target.append(np.vstack([np.c_[matrix_cam_board, p_cam_board], [0, 0, 0, 1]]))
            # print(f'# of cam_T_target Poses {len(self.cam_T_target)}')
            
        elif method == 'cv2.aruco.calibrateCameraCharucoExtended':
            for i in range(len(self.images)):
                p_cam_board = np.array(self.translation_vectors[i]).reshape(3,-1)
                rotv_cam_board = np.array(self.rotation_vectors[i]).reshape(3)
                R_cam_board = Rotation.from_rotvec(rotv_cam_board)
                matrix_cam_board = np.matmul(R_cam_board.as_matrix(), charuco_correction_matrix)
                self.cam_T_target.append(np.vstack([np.c_[matrix_cam_board, p_cam_board], [0, 0, 0, 1]]))
            # print(f'# of cam_T_target Poses: {len(self.cam_T_target)}')
        # print()

    def get_base_T_tcp_poses(self):
        # print(BLU + "GETTING base_T_tcp POSES" + RST)
        self.base_T_tcp = []
        for key in self.pose_data.keys():
            self.base_T_tcp.append(self.pose_data[key]['ee_pose'])
        # print(f'# of TCP to Base Poses: {len(self.base_T_tcp)}')
        # print()

    def solve_transform(self, X, Y):

            def ralign(X, Y):

                m, n = X.shape
                mx = X.mean(1)
                my = Y.mean(1)
                Xc = X - np.tile(mx, (n, 1)).T
                Yc = Y - np.tile(my, (n, 1)).T

                sx = np.mean(np.sum(Xc * Xc, 0))
                sy = np.mean(np.sum(Yc * Yc, 0))

                Sxy = np.dot(Yc, Xc.T) / n

                U, D, V = np.linalg.svd(Sxy, full_matrices=True, compute_uv=True)
                V = V.T.copy()

                # r = np.rank(Sxy)
                r = np.ndim(Sxy)
                d = np.linalg.det(Sxy)
                S = np.eye(m)
                if r > (m - 1):
                    if (np.det(Sxy) < 0):
                        S[m, m] = -1;
                    elif (r == m - 1):
                        if (np.det(U) * np.det(V) < 0):
                            S[m, m] = -1
                    else:
                        R = np.eye(2)
                        c = 1
                        t = np.zeros(2)
                        return R, c, t

                R = np.dot(np.dot(U, S), V.T)
                c = np.trace(np.dot(np.diag(D), S)) / sx
                t = my - c * np.dot(R, mx)

                return R, t

            R, T = ralign(X[0:3, :], Y[0:3, :])
            H = np.zeros((4, 4), dtype=float)
            H[3, 3] = 1
            for j in range(0, 3):
                H[j, 3] = T[j]
                for i in range(0, 3):
                    H[j, i] = R[j, i]

            return H

    def solveTransformation(self, H_TBs, H_ACs, initial_T0_C):
        [N, L] = [30, 0.5]

        X_A = np.zeros([4, 3 * N])
        X_A[0, 0:N] = np.linspace(0, L, N + 1)[1:]
        X_A[1, N:2 * N] = np.linspace(0, L, N + 1)[1:]
        X_A[2, 2 * N:3 * N] = np.linspace(0, L, N + 1)[1:]
        X_A[3, :] = 1

        solverBreakIterations = 2000
        solverMinRMSE = 1e-15
        solverRMSE = 1e3

        H_CT = initial_T0_C

        for solverIterations in range(0, solverBreakIterations):

            xyzlist = []
            colorlist = []

            # solve transform: X_A -> X_B

            for i in np.arange(0, len(H_TBs)):

                X_Bi = np.dot(H_TBs[i], np.dot(H_CT, np.dot(H_ACs[i], X_A)))
                X_Ai = X_A

                for j in range(0, N * 3):
                    xyzlist.append(X_Bi[0:3, j])
                    if j < N:
                        color = [255, 0, 0]
                    elif N <= j < 2 * N:
                        color = [0, 255, 0]
                    else:
                        color = [0, 0, 255]
                    colorlist.append(color)

                if (i == 0):
                    X_Bj = X_Bi
                    X_Aj = X_Ai
                else:
                    X_Bj = np.append(X_Bj, X_Bi, axis=1)
                    X_Aj = np.append(X_Aj, X_Ai, axis=1)

            H_AB = self.solve_transform(X_Aj, X_Bj)

            X_B = np.dot(H_AB, X_A)

            if solverIterations != 0:

                solverRMSE = np.average(np.linalg.norm((X_B - X_B_old)[0:3, :], axis=0))
                # print('Converging RMSE: {:.2e}'.format(solverRMSE))

            X_B_old = np.copy(X_B)

            # solve transform: X_C -> X_T

            for i in np.arange(0, len(H_TBs)):

                X_Ti = np.dot(np.linalg.inv(H_TBs[i]), X_B)
                X_Ci = np.dot(H_ACs[i], X_A)

                if (i == 0):
                    X_Tj = X_Ti
                    X_Cj = X_Ci
                else:
                    X_Tj = np.append(X_Tj, X_Ti, axis=1)
                    X_Cj = np.append(X_Cj, X_Ci, axis=1)

            H_CT = self.solve_transform(X_Cj, X_Tj)

            solverIterations += 1

            count = len(xyzlist)

            # if solverRMSE < solverMinRMSE:

                # print("calibrarion Done")
                # print(H_CT)
        return H_CT, H_AB

    def objective_function(self):
        # self.base_T_tcp_arr = self.base_T_tcp
        # self.cam_T_target_arr = self.cam_T_target
        # LHS = self.base_T_tcp_arr @ self.
        
        return

    def calibrate_camera_extrinsics(self, method = 'ARIC'):
        self.get_cam_T_target_poses()
        self.get_base_T_tcp_poses()

        print("+------------------------------+")
        print(f'|{BLU} EXTRINSIC CAMERA CALIBRATION {RST}|')
        print("+------------------------------+")
        if method == 'ARIC':
            initial_translation = np.array([0.0, 0.1, 0.1]).reshape(3,-1)
            initial_matrix = np.array([[-1,  0, 0],
                                       [ 0, -1, 0],
                                       [ 0,  0, 1]])
            initial_transformation = np.vstack([np.c_[initial_matrix, initial_translation], [0, 0, 0, 1]])

            self.tcp_T_cam, self.base_T_target = self.solveTransformation(self.base_T_tcp, self.cam_T_target, initial_transformation)
        elif method == 'OPENCV':
            R_C = []
            t_C = []
            R_B = []
            t_B = []
            for i in range(len(self.cam_T_target)):
                R_C.append(self.cam_T_target[i][:3,:3])
                t_C.append(self.cam_T_target[i][:3,3])
                R_B.append(np.array(self.base_T_tcp)[i][:3,:3])
                t_B.append(np.array(self.base_T_tcp)[i][:3,3])
            R_C = np.array(R_C)
            t_C = np.array(t_C)
            R_B = np.array(R_B)
            t_B = np.array(t_B)
            rotation_mat, translation_vec = cv2.calibrateHandEye(R_B, t_B, R_C, t_C)
            self.tcp_T_cam = np.vstack([np.c_[rotation_mat, translation_vec], [0, 0, 0, 1]])
        # tcp_T_cam
        print(f'* {GRN}tcp_T_cam{RST}')
        print(f'  {self.tcp_T_cam[0]}')
        print(f'  {self.tcp_T_cam[1]}')
        print(f'  {self.tcp_T_cam[2]}')
        print(f'  {self.tcp_T_cam[3]}')
        rot_mat_cam_to_tcp = self.tcp_T_cam[0:3,0:3]
        translation_cam_to_tcp = self.tcp_T_cam[0:3,3]
        r_cam_to_tcp =  Rotation.from_matrix(rot_mat_cam_to_tcp)
        angles_cam_to_tcp = r_cam_to_tcp.as_euler('xyz')#, degrees=True)
        angles_degrees_cam_to_tcp = r_cam_to_tcp.as_euler('xyz', degrees=True)
        # xyz -> sequence of output

        # print("Rotation Matrix")
        # print(rot_mat)
        print()
        print(f'* {GRN}Translation{RST}')
        print("  " + str(translation_cam_to_tcp))
        print()
        print(f'* {GRN}Euler Angles (ZYX){RST}')
        print(f'  {YLW}(rad){RST} {BLD}X: {RST}{angles_cam_to_tcp[0]}\t{BLD}Y: {RST}{angles_cam_to_tcp[1]}\t{BLD}Z: {RST}{angles_cam_to_tcp[2]}{RST}')
        print(f'  {YLW}(deg){RST} {BLD}X: {RST}{angles_degrees_cam_to_tcp[0]}\t{BLD}Y: {RST}{angles_degrees_cam_to_tcp[1]}\t{BLD}Z: {RST}{angles_degrees_cam_to_tcp[2]}{RST}')
        print()

        # base_T_target
        print(f'* {GRN}base_T_target{RST}')
        print(f'  {self.base_T_target[0]}')
        print(f'  {self.base_T_target[1]}')
        print(f'  {self.base_T_target[2]}')
        print(f'  {self.base_T_target[3]}')
        rot_mat_target_to_base = self.base_T_target[0:3,0:3]
        translation_target_to_base = self.base_T_target[0:3,3]
        r_target_to_base =  Rotation.from_matrix(rot_mat_target_to_base)
        angles_target_to_base = r_target_to_base.as_euler('xyz')#, degrees=True)
        angles_degrees_target_to_base = r_target_to_base.as_euler('xyz', degrees=True)
        # xyz -> sequence of output

        # print("Rotation Matrix")
        # print(rot_mat)
        print()
        print(f'* {GRN}Translation{RST}')
        print("  " + str(translation_target_to_base))
        print()
        print(f'* {GRN}Euler Angles (ZYX){RST}')
        print(f'  {YLW}(rad){RST} {BLD}X: {RST}{angles_target_to_base[0]}\t{BLD}Y: {RST}{angles_target_to_base[1]}\t{BLD}Z: {RST}{angles_target_to_base[2]}{RST}')
        print(f'  {YLW}(deg){RST} {BLD}X: {RST}{angles_degrees_target_to_base[0]}\t{BLD}Y: {RST}{angles_degrees_target_to_base[1]}\t{BLD}Z: {RST}{angles_degrees_target_to_base[2]}{RST}')
        print()

        with open(self.calibration_results_txt, 'a') as txt_file:
            txt_file.write("\n")
            txt_file.write("+------------------------------+\n")
            txt_file.write("| EXTRINSIC CAMERA CALIBRATION |\n")
            txt_file.write("+------------------------------+\n")
            txt_file.write("* tcp_T_cam\n")
            txt_file.write(f'  {self.tcp_T_cam[0]}\n')
            txt_file.write(f'  {self.tcp_T_cam[1]}\n')
            txt_file.write(f'  {self.tcp_T_cam[2]}\n')
            txt_file.write(f'  {self.tcp_T_cam[3]}\n')
            txt_file.write("* Translation\n")
            txt_file.write("  " + str(translation_cam_to_tcp) + '\n')
            txt_file.write(f'* Euler Angles (ZYX)\n')
            txt_file.write(f'  (rad) X: {angles_cam_to_tcp[0]}\tY: {angles_cam_to_tcp[1]}\tZ: {angles_cam_to_tcp[2]}\n')
            txt_file.write(f'  (deg) X: {angles_degrees_cam_to_tcp[0]}\tY: {angles_degrees_cam_to_tcp[1]}\tZ: {angles_degrees_cam_to_tcp[2]}\n')
            
            txt_file.write("* base_T_target\n")
            txt_file.write(f'  {self.base_T_target[0]}\n')
            txt_file.write(f'  {self.base_T_target[1]}\n')
            txt_file.write(f'  {self.base_T_target[2]}\n')
            txt_file.write(f'  {self.base_T_target[3]}\n')
            txt_file.write("* Translation\n")
            txt_file.write("  " + str(translation_target_to_base) + '\n')
            txt_file.write(f'* Euler Angles (ZYX)\n')
            txt_file.write(f'  (rad) X: {angles_target_to_base[0]}\tY: {angles_target_to_base[1]}\tZ: {angles_target_to_base[2]}\n')
            txt_file.write(f'  (deg) X: {angles_degrees_target_to_base[0]}\tY: {angles_degrees_target_to_base[1]}\tZ: {angles_degrees_target_to_base[2]}\n')
        
        results_dict = {'camera_matrix': self.camera_matrix.tolist(),
                        'distortion_coeffs': self.distortion_coefficients.tolist(),
                        'tcp_T_cam': {
                            'T': self.tcp_T_cam.tolist(),
                            'translation_vec': translation_cam_to_tcp.tolist(),
                            'rotation_mat': rot_mat_cam_to_tcp.tolist(),
                            'euler_angles_rad': angles_cam_to_tcp.tolist(),
                            'euler_angles_deg': angles_degrees_cam_to_tcp.tolist()
                            },
                        'base_T_target':{
                            'T': self.base_T_target.tolist(),
                            'translation_vec': translation_target_to_base.tolist(),
                            'rotation_mat': rot_mat_target_to_base.tolist(),
                            'euler_angles_rad': angles_target_to_base.tolist(),
                            'euler_angles_deg': angles_degrees_target_to_base.tolist()
                        }
                    }
        
        with open(self.calibration_results_json, 'w') as json_file:
            json.dump(results_dict, fp=json_file, ensure_ascii=True, indent=4)