# ARIC Camera Calibration Routine

**NOTE**: This package uses an old, unoptimized version of `ros_robot_pkg`, but it's working!

## Requirements
- OpenCV >=4.8

## How to use
1. Modify `calibration_config.json` as per your setup. (Currently, **ChAruCo board** is the only supported calibration target)
2. Use `bringup_calibration.launch`

## `calibration_config.json`
```json
{
    "robot": {
        "name": "UR10",
        "ip": "192.168.50.110"
    },
    "calibration_target": {
        "type": "charuco",
        "size": [11,8],
        "checker_length": 0.022,
        "marker_length": 0.016,
        "legacy_pattern": true,
        "aruco_dict": "DICT_4X4_250",
        "blur":[11,2],
        "target2base": [[-1, 0,  0, 0.055],
                        [ 0, 1,  0, -0.53],
                        [ 0, 0, -1,     0],
                        [ 0, 0,  0,     1]]
    },
    "calibration_data": {
        "project_name": "tactile_ov7521_3",
        "image_topic": "/ardu_cam/image_raw",
        "data_collection_setup": [0.4, 5, 2, 0.10, 0.15],
        "output_file_name": "tactile_ov7521"
    }
}
```