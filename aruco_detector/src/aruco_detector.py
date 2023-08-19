#!/usr/bin/env python
# AKAMAV
# project: imav2022
# author: Junbo Li
# Version: 2022.07.27


import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_detector_functions import *


#---------- Parameters ----------
#   ~markerID: aruco code ID
#   ~aruco_dict: camera dictionary
#   ~dist: distortion cofficients
#   ~cam_mtx: internal parameter matrix
#   ~markerLength: 	the length of the markers' side. The returning translation vectors will be in the same unit. Normally, unit is meters.
node_name = "aruco_detector"
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
# parameters from launch file
cam_topic        = rospy.get_param(node_name + "/cam_topic")
markerID         = rospy.get_param(node_name + "/markerID")
markerLength     = rospy.get_param(node_name + "/markerLength")
update_frequency = rospy.get_param(node_name + "/update_frequency")
dist             = np.array(rospy.get_param(node_name + "/distortion_coefficients")["data"])
cam_mtx          = np.array(rospy.get_param(node_name + "/camera_matrix")["data"]).reshape(3, 3)


#---------- function: aruco_callback ----------
#   purpose: Read the image from by camera pulished rostopic and detect aruco code.
#   parameters:
#   ~ros_img: pulished ros_image.
#   ~cam_pos: pulished cam_pos.
def aruco_callback(ros_img):
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    rot_vec, trs_vec = aruco_detector(cv_image, aruco_dict, markerLength, markerID, cam_mtx, dist)
    if (rot_vec is not False) and (trs_vec is not False):
        trans = pose2msg_converter(rot_vec[0][0], trs_vec[0][0])
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = "cam"
        trans.child_frame_id = "aruco"
        br.sendTransform(trans)
    loop_rate.sleep()


#---------- MAIN ----------
if __name__ == '__main__':
    try:
        bridge = CvBridge()
        # ROS node(Publisher)
        rospy.init_node(node_name, anonymous = True)
        # Node cycle rate (in Hz).
        loop_rate = rospy.Rate(update_frequency)
        br = tf2_ros.TransformBroadcaster()
        # Subscribers
        rospy.Subscriber(cam_topic, Image, aruco_callback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
