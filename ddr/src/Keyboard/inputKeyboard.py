#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def inputKey():
    
    pub = rospy.Publisher('keyboardTopic', Twist, queue_size = 10)
    rospy.init_node('keyInput', anonymous=True)
    r = rospy.Rate(50) 
    msg = Twist()

    while not rospy.is_shutdown():
        value = input("Premi ->\n")
        k = 0.2
        if value == 'w':
            #print("su")
            msg.linear.x = k
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
        elif value == 'd':
            #print("destra")
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = -2*k
        elif value == 's':
            #print("giu")
            msg.linear.x = -k
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
        elif value == 'a':
            #print("sinistra")
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 2*k
        elif value == 'q':
            #print("sinistra")
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        inputKey()
    except rospy.ROSInterruptException: pass


