import rospy
from custom_msg_pkg.msg import Coordinate  

def callbackCoordinateMsg(data):
    rospy.loginfo("------")
    rospy.loginfo("Get_date")
    rospy.loginfo("Start Time(sec): %d", data.start_time.secs)
    rospy.loginfo("Message Sequence: %d", data.msg_seq)
    rospy.loginfo("X: %d", data.x)
    rospy.loginfo("Y: %d", data.y)
    rospy.loginfo("Z: %d", data.z)

def main():
    rospy.init_node('get_node',anonymous=False)
        
    rospy.Subscriber("/xyz_topic",Coordinate,callbackCoordinateMsg)
    
    rospy.spin()
    
if __name__== '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass