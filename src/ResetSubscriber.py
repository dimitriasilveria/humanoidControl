import rospy
from std_msgs.msg import Empty
from std_srvs.srv import Empty
empty_packat = Empty()
def callback():
    #std_srvs::Empty empty_packet; 
    if(resetworld_client.call(empty_packet)):
        ROS_INFO("Reset World")
    else:
        ROS_INFO("Failed to reset the world")

def listener():
    rospy.init_node('subscribing_reset',anonymous=True)
    rospy.Subscriber("resetDarwinSimulation", Empty, callback)

    rospy.spin()

# if __name__ == '__main__':
#     listener()