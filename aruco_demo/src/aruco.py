#!/usr/bin/env python
# AKAMAV
# project: Indoor-copter
# author: Junbo Li
# Version: 2021.12.31


import rospy
from visualization_rviz import *
from aruco_code_detector_functions import *


#---------- Parameters ----------
#   Camera parameters:
#   ~dist: distortion cofficients
#   ~cam_mtx: internal parameter matrix
#   ~markerLength: 	the length of the markers' side. The returning translation vectors will be in the same unit. Normally, unit is meters.
camera_marker_id = 0
#aruco_marker_id = 11
markerLength = 0.093
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
dist = np.array(([[0.285183, -1.210188, -0.015466, 0.012806, 0.000000]]))
cam_mtx = np.array([[964.54659, 0.       , 345.78709],
                    [  0.     , 961.75125, 206.33139],
                    [  0.     , 0.       , 1.       ]])


#---------- MAIN ----------#
if __name__ == '__main__':
    try:
        # ROS node(Publisher)
        rospy.init_node('camera', anonymous = True)
        pub = rospy.Publisher('drone_pose', Marker, queue_size = 10)
        cap = cv2.VideoCapture(2)#cv2.CAP_DSHOW
        #img_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) # (width, height)
        #map1, map2, roi = get_map(img_size, cam_mtx, dist)
        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret is True:
                #frame = img_rectification(frame, map1, map2, roi)
                corners, ids, rot_vec, trs_vec = aruco_detector(frame, aruco_dict, markerLength, cam_mtx, dist)
                if ids is not None:
                    for i in range(rot_vec.shape[0]):
                        # aruco_pose ---> camera_pose
                        cam_rot_vec, cam_trs_vec = refepose_converter(rot_vec[i][0], trs_vec[i][0])
                        # Collect and send the result to the receiving node(here: rviz)
                        time = rospy.Time.now()
                        lifetime = rospy.Duration(10) # in seconds
                        # color = (R, G, B, A) 0.0~1.0
                        camera_color = (1.0, 0.0, 0.0, 1.0)
                        #aruco_color = (0.0, 1.0, 0.0, 1.0)
                        # camera_pose
                        camera_pose = pose2msg_converter(cam_rot_vec, cam_trs_vec)
                        # aruco_pose
                        #aruco_pose = pose2msg_converter(rot_vec[i][0], trs_vec[i][0])
                        camera_marker = pose2marker(camera_pose, time, lifetime, camera_marker_id, Color=camera_color)
                        #aruco_marker = pose2marker(aruco_pose, time, lifetime, aruco_marker_id, Color=aruco_color)
                        pub.publish(camera_marker)
                        #pub.publish(aruco_marker)
                        camera_marker_id += 1
                        #camera_marker_id %= 10
                        # Visualization
                        cv2.aruco.drawAxis(frame, cam_mtx, dist, rot_vec[i], trs_vec[i], 0.03)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.putText(frame, "Id: " + str(ids), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    #cv2.putText(frame, "distance: " + str(100*trs_vec[0][0][2]), (0, 164), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                else:
                    cv2.putText(frame, "No Aruco_code detected!", (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("output", frame)
                key = cv2.waitKey(1)
                if key == 27: # press "esc" to quit
                    break
                if key == ord(' '): # press "space" to save the current frame
                    filename = "save_test01.png"
                    cv2.imwrite(filename, frame)
                    break
            else:
                print("no more frame!")
                break
        else:
            print("VideoCapture error!")
        cap.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass

