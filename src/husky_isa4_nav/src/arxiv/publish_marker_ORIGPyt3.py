#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

topic = 'visualization_ir_kam'
publisher = rospy.Publisher(topic, Marker, queue_size=2)

rospy.init_node('ir_kam_visualiziation_node' )

x = -0.35
y = -0.4
z = 0.2

marker = Marker()
marker.header.stamp = rospy.Time()
marker.header.frame_id = "ir_kam_link"
marker.ns = "gas_camera";
marker.id = 0;
#marker.type = marker.CUBE
marker.type = marker.MESH_RESOURCE
marker.mesh_resource = "package://gas_cam/rviz/1711-00.stl"


marker.action = marker.ADD
marker.scale.x = 1/1000
marker.scale.y = 1/1000
marker.scale.z = 1/1000
#marker.scale.x = -0.35
#marker.scale.y = -0.4
#marker.scale.z = 0.2
#marker.pose.position.x = x/2
#marker.pose.position.y = y/2
#marker.pose.position.z = z/2
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0

marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.color.a = 0.5
marker.color.r = 0.0;
marker.color.g = 0.0;
marker.color.b = 1.0;


while not rospy.is_shutdown():
  publisher.publish(marker)
  rospy.rostime.wallsleep(1.0)
	
