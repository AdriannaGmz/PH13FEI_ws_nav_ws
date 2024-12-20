#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

#Position of lidar
my_x = 0.0
my_y = 0.0
my_z = 0.0
my_scale = 1.0/1000  # meters

rospy.init_node('ouster_marker_node' )

topic = '/visual_ouster_marker'
publisher = rospy.Publisher(topic, Marker, queue_size=2)


# create the marker 
marker = Marker()
marker.id = 0
# marker.ns = "lidar_ouster_ns"
marker.header.frame_id = "/os_sensor"     #/base_link
# marker.header.frame_id = "/base_link"
marker.header.stamp = rospy.Time.now()  

#   set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
# marker.type = 2
#   or
# marker.type = marker.SPHERE  #marker.CUBE , marker.ARROW , marker.CYLINDER, marker.SPHERE

marker.type = marker.MESH_RESOURCE
# marker.mesh_resource = "package://husky_isa4_nav/rviz/wheel.stl"  #set scale to 1.0
marker.mesh_resource = "package://husky_isa4_nav/rviz/LidarOs0.stl"  

# Set the scale, color of the marker
marker.scale.x = my_scale
marker.scale.y = my_scale
marker.scale.z = my_scale

marker.color.r = 0.3
marker.color.g = 0.3
marker.color.b = 0.4
marker.color.a = 1.0

# Set the pose of the marker 
marker.pose.position.x = my_x
marker.pose.position.y = my_y
marker.pose.position.z = my_z

# Rotations,    https://www.andre-gaschler.com/rotationconverter/

# No rotation
# marker.pose.orientation.x = 0.0
# marker.pose.orientation.y = 0.0
# marker.pose.orientation.z = 0.0
# marker.pose.orientation.w = 1.0

# +1.57 around X
# marker.pose.orientation.x = 0.7068252
# marker.pose.orientation.y = 0.0
# marker.pose.orientation.z = 0.0
# marker.pose.orientation.w = 0.7073883

# Euler angles of multiple axis rotations (degrees)
# 90 in x, 90 in y, 0 in z
marker.pose.orientation.x = 0.5
marker.pose.orientation.y = 0.5
marker.pose.orientation.z = 0.5
marker.pose.orientation.w = 0.5


while not rospy.is_shutdown():
  publisher.publish(marker)
  rospy.rostime.wallsleep(1.0)
	
