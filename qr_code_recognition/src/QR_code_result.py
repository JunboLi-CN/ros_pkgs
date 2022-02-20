#!/usr/bin/env python
# AKAMAV
# project: Indoor-copter
# author: Junbo Li
# Version: 2021.10.19

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received message: %s", data.data)

def QR_code_result():
    # ROS node(Subscriber)
    rospy.init_node('QR_code_result', anonymous = True)
    rospy.Subscriber('QR_code', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    QR_code_result()
