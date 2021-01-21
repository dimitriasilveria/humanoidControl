#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Float64

   

rospy.init_node('publishAngles', anonymous=True)
publisher = rospy.Publisher('/angles',Float64, queue_size=1)
theta = Float64()
theta.data = 0.5 
rate = rospy.Rate(10) # 10hz (but I have to check what is the proper rate)
while not rospy.is_shutdown(): #it keeps the code publishing while roscore is running
    publisher.publish(theta)
    rate.sleep()
