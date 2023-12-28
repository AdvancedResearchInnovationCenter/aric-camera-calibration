"""
This script includes a class and methods for robot kinematics
"""

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point
import tf2_ros
import datetime
from scipy.spatial.transform import Rotation as R

class RobotKinematics:
    """
    This is a class for robot kinematics functionality compatible with tf trees
    """
    def __init__(self):
        #TF related variables
        self.tfBuffer = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.brs = tf2_ros.StaticTransformBroadcaster()

        self.static_transform_list = []


    def set_transform(self, parent, frame, transformation_matrix, mode='normal'):
        #Sends a transform to the tf tree
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = frame

        t.transform = self.transformation_matrix_to_tf_transform(transformation_matrix)
        
        if mode=='normal':
            self.br.sendTransform(t)
        elif mode=='static':
            self.static_transform_list.append(t)
            self.brs.sendTransform(self.static_transform_list)
    
    def send_multiple_transform(self, parent, frames, transformation_matrices, mode='normal'):
        #Sends multiple transforms to the tf tree
        transform_list = []
        for i in range(len(frames)):
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = parent
            t.child_frame_id = frames[i]

            t.transform = self.transformation_matrix_to_tf_transform(transformation_matrices[i])    
            
            if mode=='normal':
                transform_list.append(t)
            else:
                self.static_transform_list.append(t)   

        if mode=='normal':
            self.br.sendTransform(transform_list) #TODO do not append
        elif mode=='static':
            self.brs.sendTransform(self.static_transform_list)

    def transformation_matrix_to_tf_transform(self, transformation_matrix):
        #converts from transformation matrix to tf transform
        transform = Transform()
        transform.translation.x = transformation_matrix[0, 3]
        transform.translation.y = transformation_matrix[1, 3]
        transform.translation.z = transformation_matrix[2, 3]

        quat = R.from_matrix(transformation_matrix[:3, :3]).as_quat()
        transform.rotation.x = quat[0]
        transform.rotation.y = quat[1]
        transform.rotation.z = quat[2]
        transform.rotation.w = quat[3]   

        return transform

    def tf_transform_to_transformation_matrix(self, tf_transform):
        #converts from tf transform to transformatio matrix
        transformation_matrix = np.eye(4)

        quat = [tf_transform.rotation.x, tf_transform.rotation.y, tf_transform.rotation.z, tf_transform.rotation.w]
        rotation = R.from_quat(quat) 
        dcm = rotation.as_matrix()
        transformation_matrix[:3, :3] = dcm

        transformation_matrix[0, 3] = tf_transform.translation.x
        transformation_matrix[1, 3] = tf_transform.translation.y
        transformation_matrix[2, 3] = tf_transform.translation.z

        return transformation_matrix

    def pose_to_transformation_matrix(self, Pose_msg):
        #converts from pose message to transformation matrix
        transformation_matrix = np.eye(4)

        quat = [Pose_msg.orientation.x, Pose_msg.orientation.y, Pose_msg.orientation.z, Pose_msg.orientation.w]
        rotation = R.from_quat(quat) 
        dcm = rotation.as_matrix()
        transformation_matrix[:3, :3] = dcm

        transformation_matrix[0, 3] = Pose_msg.position.x
        transformation_matrix[1, 3] = Pose_msg.position.y
        transformation_matrix[2, 3] = Pose_msg.position.z

        return transformation_matrix

    def transformation_matrix_to_pose(self, transformation_matrix):
        #converts from transformation matrix to pose message
        pose_msg = Pose()

        pose_msg.position.x = transformation_matrix[0, 3]
        pose_msg.position.y = transformation_matrix[1, 3]
        pose_msg.position.z = transformation_matrix[2, 3]

        quat = R.from_matrix(transformation_matrix[:3, :3]).as_quat()
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]  

        return pose_msg

    def receive_transform(self, parent, frame):
        #receive transform from tf tree
        transform = self.tfBuffer.lookup_transform(parent, frame, rospy.Time(),timeout=rospy.Duration(secs=1))
        transformation_matrix = self.tf_transform_to_transformation_matrix(transform.transform)

        return transform, transformation_matrix
    
    def wait_for_transform(self, parent, frame):
        #wait and receive transform from tf tree
        transform = self.tfBuffer.lookup_transform(parent, frame, rospy.Time().now(), timeout=rospy.Duration(secs=3))
        transformation_matrix = self.tf_transform_to_transformation_matrix(transform.transform)

        return transform, transformation_matrix

    def convert_vector_base_frame(self, vector, parent, target):
        #convert a 3D vector from parent frame to target
        _, tranform_matrix = self.receive_transform(parent, target)
        rot_matrix = tranform_matrix[:3, :3]
        return np.matmul(rot_matrix, vector.reshape((3,1)))

    
    def add_transformations(self, transformation_matrix_a, transformation_matrix_b):
        #Combine two transformation matrices
        return np.matmul(transformation_matrix_a, transformation_matrix_b)

    def subtract_transformations(self, transformation_matrix_a, transformation_matrix_b):
        #get the difference between two transformation matrices
        result_rotation = np.matmul(transformation_matrix_a[:3,:3], transformation_matrix_b[:3,:3].transpose())

        result_translation = transformation_matrix_a[:3, 3] - np.matmul(transformation_matrix_a[:3,:3], np.matmul(transformation_matrix_b[:3,:3].transpose(), transformation_matrix_b[:3, 3]))

        result_transformation = np.eye(4)
        result_transformation[:3, :3] = result_rotation
        result_transformation[:3, 3] = result_translation

        return result_transformation

    def convert_wrench_base_frame(self, wrench, parent, target):
        #convert a 3D vector from parent frame to target
        _, tranform_matrix = self.receive_transform(parent, target)

        skew_symmetrix_pose = np.array([ [0, -tranform_matrix[2, 3], -tranform_matrix[1, 3]], [tranform_matrix[2, 3], 0, -tranform_matrix[0, 3]], [-tranform_matrix[1, 3], tranform_matrix[0, 3], 0] ])
        
        adjoint_matrix = np.zeros((6,6))
        adjoint_matrix = np.zeros((6,6))
        adjoint_matrix[0:3, 0:3] = np.transpose(tranform_matrix[:3,:3])
        adjoint_matrix[3:6, 3:6] = np.transpose(tranform_matrix[:3,:3])
        adjoint_matrix[3:6, 0:3] = np.matmul(np.transpose(tranform_matrix[:3,:3]), np.transpose(skew_symmetrix_pose))
    
        return np.matmul(adjoint_matrix, wrench.reshape(6,1))

