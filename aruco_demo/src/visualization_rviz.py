#!/usr/bin/env python
# AKAMAV
# project: Indoor-copter
# author: Junbo Li
# Version: 2022.01.01


from visualization_msgs.msg import Marker


# ---------- visualization_msgs/Marker Message ----------
# ----------      Raw Message Definition       ----------
#
# parameters:
# ~Header header                        # header for time/frame information
# ~string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
# ~int32 id                             # object ID useful in conjunction with the namespace for manipulating and deleting the object later
# ~int32 type                           # Type of object
# ~int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
# ~geometry_msgs/Pose pose              # Pose of the object
# ~geometry_msgs/Vector3 scale          # Scale of the object 1,1,1 means default (usually 1 meter square)
# ~std_msgs/ColorRGBA color             # Color [0.0-1.0]
# ~duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
# ~bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

# type:
# ~uint8 ARROW=0
# ~uint8 CUBE=1
# ~uint8 SPHERE=2
# ~uint8 CYLINDER=3
# ~uint8 LINE_STRIP=4
# ~uint8 LINE_LIST=5
# ~uint8 CUBE_LIST=6
# ~uint8 SPHERE_LIST=7
# ~uint8 POINTS=8
# ~uint8 TEXT_VIEW_FACING=9
# ~uint8 MESH_RESOURCE=10
# ~uint8 TRIANGLE_LIST=11

# action:
# ~uint8 ADD=0
# ~uint8 MODIFY=0
# ~uint8 DELETE=2
# ~uint8 DELETEALL=3

# else:
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
# geometry_msgs/Point[] points
# Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
# number of colors must either be 0 or equal to the number of points
# Note: alpha is not yet used
# std_msgs/ColorRGBA[] colors
# Note: only used for text markers
# string text
# Note: only used for MESH_RESOURCE markers
# string mesh_resource
# bool mesh_use_embedded_materials


def pose2marker(Pose, Time, Lifetime, ID=0, NS="cam_pose", Frame_id="map", 
                Shape=Marker.ARROW, Scale=0.02, Color=(1.0, 0.0, 0.0, 1.0)):
    marker = Marker()
    marker.header.frame_id = Frame_id
    marker.header.stamp = Time
    marker.ns = NS
    marker.id = ID
    marker.type = Shape
    marker.action = Marker.ADD
    marker.pose = Pose
    marker.scale.x = Scale
    marker.scale.y = Scale
    marker.scale.z = Scale
    marker.color.r = Color[0]
    marker.color.g = Color[1]
    marker.color.b = Color[2]
    marker.color.a = Color[3]
    # lifetime = rospy.Duration() means never to auto-delete!
    marker.lifetime = Lifetime
    return marker