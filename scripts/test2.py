#!/usr/bin/python3

from aric_camera_calibration import CameraCalibrator
from data_collection_routine import CameraCalibrationDataCollection
import _thread
import rospy
from roslaunch.parent import ROSLaunchParent

if __name__ == '__main__':
    
    calibration_config_file = rospy.get_param('/camera_calibration/calibration_config')
    data_collector = CameraCalibrationDataCollection(calibration_config_file)
    # _thread.start_new_thread(data_collector.robot.run_node, ())
    # _thread.start_new_thread(data_collector.collect_data, ())

    while not rospy.is_shutdown():
        if 1:#data_collector.data_collected:
            # robot.cleanup()
            calibrator = CameraCalibrator(data_collector.calibration_config)
            calibrator.calibrate_camera_intrinsics()
            calibrator.calibrate_camera_extrinsics()
            break
    
    rospy.signal_shutdown(reason="Camera calibration done.")
    _thread.exit()

