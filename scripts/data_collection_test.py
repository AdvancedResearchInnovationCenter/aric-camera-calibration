#!/usr/bin/python3

# import json
import argparse
import os
from data_collection_routine import CameraCalibrationDataCollection
import rospy
import time

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--calibration-config-file', type=str, required=True, help='Specifies the configuration file.')
    # parser.add_argument('-v', '--verbose', action='store_true', required=False, help='Output device information.')

    args = parser.parse_args()
    calibration_config_file = args.calibration_config_file

    rospy.init_node('test')
    data_collector = CameraCalibrationDataCollection(calibration_config_file)
    saved = False
    # while not rospy.is_shutdown():
    #     rgb, gray = data_collector.getRosImage()
    #     if not saved:
    #         data_collector.save_image('test.png', data_collector.cv_bridge.imgmsg_to_cv2(rgb))
    #         data_collector.save_image('test2.png', gray)
    #         saved = True