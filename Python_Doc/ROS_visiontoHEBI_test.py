#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import random
import time

flag = 0
flag_vision = 0
p = Pose()
p_vision = Pose()

p.position.x = 0.285
p.position.y = 0.170
p.position.z = -0.266
p.orientation.x = 0
p.orientation.y = 0.9877
p.orientation.z = 0
p.orientation.w = 0.1564

def callback(msg):
    global flag
    print("I receive the request from Matlab")
    flag = 1

def marker_callback(msg):
    if msg.ns == 'obj':   
        if msg.id == 1: 
            global flag_vision
            print("I receive the object pose from vision")    
            flag_vision = 1
            p_vision.position.x = msg.pose.position.x
            p_vision.position.y = msg.pose.position.y
            p_vision.position.z = msg.pose.position.z
            p_vision.orientation.x = msg.pose.orientation.x
            p_vision.orientation.y = msg.pose.orientation.y
            p_vision.orientation.z = msg.pose.orientation.z
            p_vision.orientation.w = msg.pose.orientation.w

rospy.init_node('Vision_node')

posemsg_pub = rospy.Publisher('pose', Pose, queue_size=1)
request_sub = rospy.Subscriber('requestfrommatlab', Int32, callback)
objpose_sub = rospy.Subscriber('obj_markerss', Marker, marker_callback)
#/det/pick_object/obj_marker
rate = rospy.Rate(1)

while not rospy.is_shutdown():
   
    if flag == 1:
        time.sleep(2)
        if flag_vision == 1:
            p.position.x = p_vision.position.x
            p.position.y = p_vision.position.y
            p.position.z = p_vision.position.z
            p.orientation.x = p_vision.orientation.x
            p.orientation.y = p_vision.orientation.y
            p.orientation.z = p_vision.orientation.z
            p.orientation.w = p_vision.orientation.w  
            flag_vision = 0 
            print("I send the object pose obtained from vision") 
        else:
            p.position.x = 0.285
            p.position.y = 0.170
            p.position.z = -0.266
            p.orientation.x = 0
            p.orientation.y = 0.9877
            p.orientation.z = 0
            p.orientation.w = 0.1564
            print("I send the initial object pose") 

        posemsg_pub.publish(p)
        flag = 0
    rate.sleep()



