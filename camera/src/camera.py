#!/usr/bin/env python
# AKAMAV
# project: imav2022
# author: Junbo Li
# Version: 2022.07.27

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#---------- Parameters ----------
#   Camera parameters:
#   ~device_num: device_num of the camera
#   ~width: width of the camera
#   ~height: height of the camera
#   ~dist: distortion cofficients
#   ~cam_mtx: internal parameter matrix
node_name = "camera"
# parameters from launch file
device_num = rospy.get_param(node_name + "/device_num")
width      = rospy.get_param(node_name + "/width")
height     = rospy.get_param(node_name + "/height")
fps        = rospy.get_param(node_name + "/fps")
dist       = np.array(rospy.get_param(node_name + "/distortion_coefficients")["data"])
cam_mtx    = np.array(rospy.get_param(node_name + "/camera_matrix")["data"]).reshape(3, 3)


#---------- MAIN ----------
if __name__ == '__main__':
    try:
        bridge = CvBridge()
        # ROS node(Publisher)
        rospy.init_node(node_name, anonymous = True)
        pub = rospy.Publisher("cam_img", Image, queue_size = 10)
        cap = cv2.VideoCapture(device_num)#cv2.CAP_DSHOW
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS , fps)
        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret is True:
                #img_undistorted = cv2.undistort(frame, cam_mtx, dist)
                image_message = bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                image_message.header.stamp = rospy.Time.now()
                pub.publish(image_message)

                #---------- for debugging ----------
                # cv2.imshow("output", frame)
                # key = cv2.waitKey(1)
                # if key == ord(' '): # press "space" to save the current frame
                #     filename = "save_img01.png"
                #     cv2.imwrite(filename, frame)
                # if key == 27: # press "esc" to quit
                #     break

            else:
                print("no more frame!")
                break
        else:
            print("VideoCapture error!")
        cap.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
