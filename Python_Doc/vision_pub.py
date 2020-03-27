#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import random
import time

rospy.init_node('Vision_pub_node')
visionmarker_pub = rospy.Publisher('obj_markerss', Marker, queue_size=1)

v = Marker()

v.pose.position.x = 1
v.pose.position.y = 2
v.pose.position.z = 3
v.pose.orientation.x = 11
v.pose.orientation.y = 22
v.pose.orientation.z = 33
v.pose.orientation.w = 44
v.ns = 'obj'

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    time.sleep(2)
    visionmarker_pub.publish(v)  
    print("I send vision pose") 
    
    rate.sleep()

