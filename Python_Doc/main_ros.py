#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import random
import time

flag = 0

def callback(msg):
    global flag
    print("I receive the request")
    flag = 1

rospy.init_node('Vision_node')

posemsg_pub = rospy.Publisher('pose', Pose, queue_size=1)
request_sub = rospy.Subscriber('request', Int32, callback)

p = Pose()
rate = rospy.Rate(1)

while not rospy.is_shutdown():

    #p.position.x = random.randrange(1, 10)
    p.position.x = 0.315
    p.position.y = -0.18 #-0.265
    p.position.z = -0.185
    p.orientation.x = 0
    p.orientation.y = 0.9239
    p.orientation.z = 0.3827
    p.orientation.w = 0

    if flag == 1:
        time.sleep(2)
        posemsg_pub.publish(p)
        flag = 0
    
    rate.sleep()



