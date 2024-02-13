#!/usr/bin/python3

import json
import os
from aric_camera_calibration import CameraCalibrator
from data_collection_routine import CameraCalibrationDataCollection
import _thread
import rospy

if __name__ == '__main__':
    calibration_config_file = rospy.get_param('/camera_calibration/calibration_config')
    data_collector = CameraCalibrationDataCollection(calibration_config_file)
    if not data_collector.use_existing_data:
        print(1)
        _thread.start_new_thread(data_collector.robot.run_node, ())
        print(2)
        _thread.start_new_thread(data_collector.collect_data, ())

    while not rospy.is_shutdown():
        if data_collector.data_collected:
            if data_collector.calibration_config['calibration_target']['type'] == 'aruco':
                raise NotImplementedError('Aruco calibration not implemented yet. Only collecting data is implemented.')
            # robot.cleanup()
            # print(data_collector.calibration_config)
            calibrator = CameraCalibrator(data_collector.calibration_config)
            calibrator.calibrate_camera_intrinsics()
            calibrator.calibrate_camera_extrinsics()
            break
    
    rospy.signal_shutdown(reason="Camera calibration done.")
    _thread.exit()

