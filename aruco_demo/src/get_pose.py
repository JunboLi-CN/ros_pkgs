#!/usr/bin/env python
# AKAMAV
# project: Indoor-copter
# author: Junbo Li
# Version: 2021.12.31

import rospy
from geometry_msgs.msg import Pose

def callback(data):
    rospy.loginfo("pose: %s", data)

def get_pose():
    # ROS node(Subscriber)
    rospy.init_node('get_pose', anonymous = True)
    rospy.Subscriber('drone_pose', Pose, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    get_pose()
