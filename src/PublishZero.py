import rospy
from std_msgs.msg import Float64
import numpy as np

angleMsg = Float64()
rospy.init_node('angles') #iniciando o nó
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
p12= rospy.Publisher('/robotis_op3/r_ank_pitch_position/command',Float64,queue_size=1)
publishers = np.array([p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12])

theta = np.zeros((6,2))    
theta[:,0] = np.array([0.0, 0.0, 0.5146, 2.1122, 0.5147, 0.0]) #perna direita
theta[:,1] = np.array([0.0, 0.0, 0.5146, 2.1122, 0.5147, 0.0]) #perna esquerda


while not rospy.is_shutdown():
    angleMsg.data = theta[0,0]#escrevendo theta[0] em angle
    publishers[0].publish(angleMsg) #publicando angle

    angleMsg.data = theta[1,0]
    publishers[1].publish(angleMsg)

    angleMsg.data = theta[2,0]
    publishers[2].publish(angleMsg)

    angleMsg.data = theta[3,0]
    publishers[3].publish(angleMsg)

    angleMsg.data = theta[4,0]
    publishers[4].publish(angleMsg)

    angleMsg.data = theta[5,0]
    publishers[5].publish(angleMsg)

    angleMsg.data = theta[0,1]
    publishers[6].publish(angleMsg)

    angleMsg.data = theta[1,1]
    publishers[7].publish(angleMsg)

    angleMsg.data = theta[2,1]
    publishers[8].publish(angleMsg)

    angleMsg.data = theta[3,1]
    publishers[9].publish(angleMsg)

    angleMsg.data = theta[4,1]
    publishers[10].publish(angleMsg)

    angleMsg.data = theta[5,1]
    publishers[11].publish(angleMsg)