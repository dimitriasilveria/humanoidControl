#! /usr/bin/env python

import rospy
import numpy as np 
from std_msgs.msg import Float64
from globalVariables import GlobalVariables

theta= Float64()
a = GlobalVariables()


rospy.init_node('angles') #iniciando o nó
#ang 1-6 perna direita e ang7-12 perna esquerda

Vr = np.array([0.0, 0.0, -0.5147, 1.056, 0.0, -0.5147]).reshape((6,1)) #perna direita
Vl = np.array([0.0, 0.0, -0.5147, 1.056, 0.0, -0.5147]).reshape((6,1)) #perna esquerda
# Vr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1)) #perna direita
# Vl = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((6,1)) #perna esquerda
p1 = rospy.Publisher('/robotis_op3/r_hip_yaw_position/command',Float64,queue_size=1) # '/robotis_op3/r_hip_yaw_position/command' é o tópico onde a mensagem será publicada"
p2 = rospy.Publisher('/robotis_op3/r_hip_roll_position/command',Float64,queue_size=1)
p3 = rospy.Publisher('/robotis_op3/r_hip_pitch_position/command',Float64,queue_size=1)
p4 = rospy.Publisher('/robotis_op3/r_knee_position/command',Float64,queue_size=1)
p5 = rospy.Publisher('/robotis_op3/r_ank_roll_position/command',Float64,queue_size=1)
p6 = rospy.Publisher('/robotis_op3/r_ank_pitch_position/command',Float64,queue_size=1)
p7 = rospy.Publisher('/robotis_op3/l_hip_yaw_position/command',Float64,queue_size=1)
p8 = rospy.Publisher('/robotis_op3/l_hip_roll_position/command',Float64,queue_size=1)
p9 = rospy.Publisher('/robotis_op3/l_hip_pitch_position/command',Float64,queue_size=1)
p10 = rospy.Publisher('/robotis_op3/l_knee_position/command',Float64,queue_size=1)
p11 = rospy.Publisher('/robotis_op3/l_ank_roll_position/command',Float64,queue_size=1)
p12= rospy.Publisher('/robotis_op3/l_ank_pitch_position/command',Float64,queue_size=1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # Vr = a.getThetaR()   #vetor perna direira p1-6
    # Vl = a.getThetaL()   #vetor perna esquerda p7-12

    theta.data = Vr[0] #escrevendo Vr[0] em theta
    p1.publish(theta) #publicando theta
    print("publicando", Vr)
    theta.data = Vr[1]
    p2.publish(theta)

    theta.data = Vr[2]
    p3.publish(theta)

    theta.data = Vr[3]
    p4.publish(theta)

    theta.data = Vr[4]
    p5.publish(theta)

    theta.data = Vr[5]
    p6.publish(theta)

    theta.data = Vl[0]
    p7.publish(theta)

    theta.data = Vl[1]
    p8.publish(theta)

    theta.data = Vl[2]
    p9.publish(theta)

    theta.data = Vr[3]
    p10.publish(theta)

    theta.data = Vl[4]
    p11.publish(theta)

    theta.data = Vl[5]
    p12.publish(theta)

    rate.sleep()



