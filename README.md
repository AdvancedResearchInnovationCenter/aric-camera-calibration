# ARIC Camera Calibration Routine

**NOTE**: This package uses an old, unoptimized version of `ros_robot_pkg`, but it's working!

## Requirements

- OpenCV >=4.8
- OpenCV contrib >= 4.8

## How to use

1. Modify `calibration_config.json` as per your setup. (Currently, **ChAruCo board** is the only supported calibration target)
2. Use `bringup_calibration.launch`
3. Calibration data will be saved in a new directory:
   ```
   ros_robot
     ├── CMakeLists.txt
     ├── README.md
     ├── calibration_data  <-- NEW DIRECTORY CREATED FOR YOUR CALIBRATION DATA
     ├── launch
     ├── package.xml
     ├── scripts
     ├── srv
     └── xacros
   ```

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

### 1. `robot`

Info about the robot used

|      Key | Description                |             Value             |  Type  |
| -------: | -------------------------- | :----------------------------: | :----: |
| `name` | Specifies the robot in use | "UR10", "ABB", or "Mitsubishi" | string |
|   `ip` | Robot's IP address         |   _e.g._ "192.168.50.110"   | string |

### 2. `calibration_target`

Info about the calibration target

|                Key | Description                                                                                                                 |                                                                                                              Value                                                                                                              |         Type         |                                                                               Note                                                                               |
| -----------------: | --------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :-------------------: | :---------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|           `type` | Calibration target type                                                                                                     |                                                                                                 "charuco", "aruco", or "checker"                                                                                                 |        string        |                                                                Currently, only "charuco" will work                                                                |
|           `size` | Calibration target size                                                                                                     |                                                                                                 [rows x cols] (_e.g._ [11,8])                                                                                                 |      [int, int]      |                                                               num of squares,_NOT_ inner corners                                                               |
| `checker_length` | Charuco/chess board black square length (m)                                                                                 |                                                                                                          _e.g._ 0.022                                                                                                          |         float         |                                                  It's better to measure it after printing the calibration target                                                  |
|  `marker_length` | Charuco/aruco marker length (m)                                                                                             |                                                                                                          _e.g._ 0.016                                                                                                          |         float         |                                                  It's better to measure it after printing the calibration target                                                  |
| `legacy_pattern` | This is related to Charuco targets.`<br>`It specifies whether you are using the old or the new  pattern for Charuco board |                                                                                                            True/False                                                                                                            |         bool         |                                Check this[issue](https://github.com/opencv/opencv/issues/23873#issuecomment-1620504453) for more info                                |
|     `aruco_dict` | Which aruco dictionary is in use                                                                                            |                                                                                                    _e.g._ `DICT_4X4_250`                                                                                                    |        string        | Use the same naming pattern as in `cv2.aruco` library [here](https://docs.opencv.org/4.8.0/de/d67/group__objdetect__aruco.html#ga4e13135a118f497c6172311d601ce00d) |
|           `blur` | Gaussian blur parameters used to smoothen the captured images                                                               |                                                                                       [kernel_size, standard_deviation] (_e.g._  [11,2])                                                                                       |     [int, float]     |                      [cv2.GaussianBlur()](https://docs.opencv.org/4.8.0/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1)                      |
|    `target2base` | The transformation matrix from the calibration target to the robot base                                                     | $`{}^{base}T_{target} = \begin{bmatrix}  R_{3×3} & T_{3×1} \\[0.5em] 0_{1×3} & 1 \end{bmatrix}`$ `<br>` _e.g._ `<br>` [[-1, 0,  0, 0.055],`<br>`[ 0, 1,  0, -0.53],`<br>` [ 0, 0, -1, 0],`<br>` [ 0, 0,  0, 1]] | float (list of lists) |                            For convenience, we set the calibration target`<br>` orientation to be the same as the camera orientation                            |

### 3. `calibration_data`

Info about the output calibration data

| Key                       | Description                       | Value                                                                                                      | Type                            | Note                                                                                              |
| ------------------------- | --------------------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------- |
| `project_name`          | Your project name                 | _e.g._ "D435_calibration"                                                                                | string                          | A new directory named after the project will be`<br>` created to store all the calibration data |
| `image_topic`           | Inbound image stream              | _e.g._ "/image/raw"                                                                                      | string                          | The input `image_topic` will be subscribed to                                                   |
| `data_collection_setup` | Random pose generation parameters | [$`\phi`$, n_cycle, n_pose_per_cycle, min_radius, max_radius] `<br>` _e.g._ [0.4, 5, 20, 0.10, 0.15] | [float, int, int, float, float] |                                                                                                   |
| `output_file_name`      | Pose-image pairs data JSON file   | _e.g._ "D435_calibration_2nd.json"                                                                       | string                          |                                                                                                   |

## Sample Output

```txt
+------------------------------+
| INTRINSIC CAMERA CALIBRATION |
+------------------------------+
* Reprojection Error: 0.3585812344507631
* Camera Matrix
  [438.59222618   0.         323.89686632]
  [  0.         439.81991094 240.11146683]
  [0. 0. 1.]
* Distortion Coefficients
  [0.11232303654662454, -0.47277989637474027, -6.726553087628563e-05, 0.0005557680260483787, 0.5754486589853096]

+------------------------------+
| EXTRINSIC CAMERA CALIBRATION |
+------------------------------+
* tcp_T_cam
  [ 0.99841971 -0.00733935  0.0557155  -0.00005566]
  [0.00560671 0.99949772 0.03119081 0.00015634]
  [-0.05591644 -0.03082914  0.99795938  0.13995561]
  [0. 0. 0. 1.]

* Translation
  [-0.00005566  0.00015634  0.13995561]

* Euler Angles (ZYX)
  (rad) X: -0.03088235865331096	Y: 0.055945617151973215	Z: 0.005615524215201508
  (deg) X: -1.7694288122440347	Y: 3.205447745062774	Z: 0.32174583728456024
```
