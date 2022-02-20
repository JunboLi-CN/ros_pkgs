#!/usr/bin/env python
# AKAMAV
# project: Indoor-copter
# author: Junbo Li
# Version: 2021.12.31


import cv2
import numpy as np
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion


#---------- function: get_map ----------
#   purpose: Computes the undistortion and rectification transformation map.
#   parameters:
#   ~img_size: size of the input_frame (width, height).
#   ~cam_mtx: internal parameter matrix from camera.
#   ~dist: distortion cofficients.
#   ~alpha: free scaling parameter between 0 (when all the pixels in the undistorted image are valid) and 1 (when all the source image pixels are retained in the undistorted image).
#   ~R: rectification_matrix (default: identity matrix(3*3)).
#       Optional rectification transformation in the object space (3x3 matrix). R1 or R2,
#       computed by stereoRectify can be passed here. If the matrix is empty, the identity transformation is assumed. In cvInitUndistortMap R assumed to be an identity matrix.
def get_map(img_size, cam_mtx, dist, alpha=0, R=np.eye(3)):
    new_cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist, img_size, alpha, img_size)
    map1, map2 = cv2.initUndistortRectifyMap(cam_mtx, dist, R, new_cam_mtx, img_size, m1type=cv2.CV_32FC1)
    return map1, map2, roi


#---------- function: img_rectification ----------
#   purpose: Rectify the image according to the acquired map.
#   parameters:
#   ~img: input frame.
#   ~map1, map2: transformation map.
#   ~roi: The image after removing the cropped area.
def img_rectification(img, map1, map2, roi):
    corrected_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_CUBIC)
    x, y, w, h = roi
    corrected_img = corrected_img[y:(y + h), x:(x + w)]
    return corrected_img


#---------- function: aruco_detector ----------
#   purpose: Detect aruco code and return its ID and pos.
#   parameters:
#   ~img: input frame.
#   ~aruco_dict: indicates the type of markers that will be searched.
#   ~markerLength: 	the length of the markers' side. The returning translation vectors will be in the same unit. Normally, unit is meters.
#   ~cam_mtx: internal parameter matrix from camera.
#   ~dist: distortion cofficients.
#   Note: estimatePoseSingleMarkers: 
#         This function receives the detected markers and returns their pose estimation respect to the camera individually. 
#         So for each marker, one rotation and translation vector is returned. 
#         The returned transformation is the one that transforms points from each marker coordinate system to the camera coordinate system. 
#         The marker corrdinate system is centered on the middle of the marker, with the Z axis perpendicular to the marker plane. 
#         The coordinates of the four corners of the marker in its own coordinate system are: 
#         (-markerLength/2, markerLength/2, 0), (markerLength/2, markerLength/2, 0), (markerLength/2, -markerLength/2, 0), (-markerLength/2, -markerLength/2, 0).
def aruco_detector(img, aruco_dict, markerLength, cam_mtx, dist):
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters, cameraMatrix=cam_mtx, distCoeff=dist)
    if ids is not None:
        rot_vec, trs_vec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cam_mtx, dist)
    else:
        rot_vec, trs_vec = False, False
    return corners, ids, rot_vec, trs_vec


#---------- function: pose_msg_sender ----------
#   purpose: get the pose of the camera wrt aruco: Change the reference coordinate system from the camera to aruco marker.
#   parameters:
#   ~aruco_rot_vec: Rotation vector of aruco marker
#   ~aruco_trs_vec: Translation vector of aruco marker
def refepose_converter(aruco_rot_vec, aruco_trs_vec):
    aruco_rot_mtx = cv2.Rodrigues(aruco_rot_vec)[0]
    aruco_pose_mtx = np.array(
               [[aruco_rot_mtx[0][0], aruco_rot_mtx[0][1], aruco_rot_mtx[0][2], aruco_trs_vec[0]],
                [aruco_rot_mtx[1][0], aruco_rot_mtx[1][1], aruco_rot_mtx[1][2], aruco_trs_vec[1]],
                [aruco_rot_mtx[2][0], aruco_rot_mtx[2][1], aruco_rot_mtx[2][2], aruco_trs_vec[2]],
                [                 0.,                  0.,                  0.,               1.]])
    cam_pose_mtx = np.linalg.inv(aruco_pose_mtx)
    cam_rot_mtx = np.array(
                  [[cam_pose_mtx[0][0], cam_pose_mtx[0][1], cam_pose_mtx[0][2]],
                   [cam_pose_mtx[1][0], cam_pose_mtx[1][1], cam_pose_mtx[1][2]],
                   [cam_pose_mtx[2][0], cam_pose_mtx[2][1], cam_pose_mtx[2][2]]])
    cam_rot_vec = cv2.Rodrigues(cam_rot_mtx)[0].reshape(3,)
    cam_trs_vec = np.array([cam_pose_mtx[0][3], cam_pose_mtx[1][3], cam_pose_mtx[2][3]])
    return cam_rot_vec, cam_trs_vec


#---------- function: axis_converter ----------
#   purpose: Convert the axis indicated by the arrow to the specified axis (default: X axis).
#   parameters:
#   ~rot_quaternion: original rotation (default: X axis).
#   ~dest_axis: destinate rotation (default: Z axis, perpendicular to the camera).
#   Note: 
#         Quaternion.w = cos (angle / 2) 
#         Quaternion.x = axis.x * sin (angle / 2) 
#         Quaternion.y = axis.y * sin (angle / 2)  
#         Quaternion.z = axis.z * sin (angle / 2)
def axis_converter(rot_quaternion, dest_axis='z'):
    if dest_axis is 'z':
        # X ---> Z axis: rotate -90° along the Y axis
        z_rot_quaternion = Quaternion(0.707, 0., -0.707, 0.)
        rot_quaternion *= z_rot_quaternion
    elif dest_axis is 'y':
        # X ---> Y axis: rotate +90° along the Z axis
        y_rot_quaternion = Quaternion(0.707, 0., 0., 0.707)
        rot_quaternion *= y_rot_quaternion
    elif dest_axis is 'x':
        print("No need to convert! it is already x axis.")
    else:
        print("No such axis! return original rotation (x axis).")
    return rot_quaternion


#---------- function: pose_msg_sender ----------
#   purpose: Convert the obtained position information to "geometry_msgs.Pose" type information.
#   parameters:
#   ~rot_vec: Rotation vector
#   ~trs_vec: Translation vector
#   ~dest_axis: destinate rotation (default: Z axis, perpendicular to the camera).
def pose2msg_converter(rot_vec, trs_vec, dest_axis='z'):
    # Convert rotation vector to rotation matrix
    rot_mtx = cv2.Rodrigues(rot_vec)[0]
    pose = Pose()

    pose.position.x = trs_vec[0]
    pose.position.y = trs_vec[1]
    pose.position.z = trs_vec[2]

    # orientation is expressed as a quaternion
    rot_quaternion = Quaternion(matrix=rot_mtx)
    rot_quaternion = axis_converter(rot_quaternion, dest_axis)
    pose.orientation.w = rot_quaternion[0]
    pose.orientation.x = rot_quaternion[1]
    pose.orientation.y = rot_quaternion[2]
    pose.orientation.z = rot_quaternion[3]
    
    return pose