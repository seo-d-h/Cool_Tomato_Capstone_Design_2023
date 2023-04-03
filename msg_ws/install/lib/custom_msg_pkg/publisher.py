import rospy 
import random 
from custom_msg_pkg.msg import Coordinate 

def main():
    rospy.init_node('send_node',anonymous=False)
    
    pub = rospy.Publisher('xyz_topic',Coordinate,queue_size=10)
    
    rate = rospy.Rate(1)
    
    msg = Coordinate()
    count = 1
    
    while not rospy.is_shutdown():
        msg.start_time = rospy.Time.now()
        msg.msg_seq = count
        msg.x = random.randrange(1,101)
        msg.y = random.randrange(1,101)
        msg.z = random.randrange(1,101)
        
        rospy.loginfo("------")
        rospy.loginfo("Start Time(sec): %d", msg.start_time.secs)
        rospy.loginfo("Message Sequence: %d", msg.msg_seq)
        rospy.loginfo("X: %d", msg.x)
        rospy.loginfo("Y: %d", msg.y)
        rospy.loginfo("Z: %d", msg.z)
        
        pub.publish(msg)
        
        rate.sleep()
        
        count +=1
        
if __name__== '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass