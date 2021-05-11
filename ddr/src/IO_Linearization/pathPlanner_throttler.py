#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from gazebo_msgs.msg import ModelStates
import math
from matplotlib import pyplot as plt



# Function that calculate rpy angles from quaternion
def euler_from_quaternion(x, y, z, w):
        
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)   
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)  
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians

# Circular trajectory
def circular(w,R,t):
   
    xd_d =    -R + R*np.cos(w*t)
    yd_d =     R*np.sin(w*t)
    xd_dot =  -w*R*np.sin(w*t)
    yd_dot =   w*R*np.cos(w*t)
    return xd_d,yd_d,xd_dot,yd_dot

# Linear trajectory
def linear(v,t):
    
    xd_d =    v*t
    yd_d =    v*t
    xd_dot =  v
    yd_dot =  v  
    return xd_d,yd_d,xd_dot,yd_dot

def callback(tmp):

    t = rospy.get_time()    
    x = round(tmp.pose[1].position.x,2) 
    y = round(tmp.pose[1].position.y,2) 
    z = round(tmp.pose[1].position.z,2) 
    X = round(tmp.pose[1].orientation.x,2)
    Y = round(tmp.pose[1].orientation.y,2)
    Z = round(tmp.pose[1].orientation.z,2)
    W = round(tmp.pose[1].orientation.w,2)
    roll_x,pitch_y,yaw_z = euler_from_quaternion(X,Y,Z,W)
    theta = yaw_z

    # Impose a desired trajectory
    xd_d,yd_d,xd_dot,yd_dot = circular(0.5, 0.8, t)
    #xd_d,yd_d,xd_dot,yd_dot = linear(0.1,t)

    # Publish odometry and desired trajectory on a topic
    msg = Float64MultiArray()
    msg.data = [x,y,theta,xd_d,yd_d,xd_dot,yd_dot]
    pub.publish(msg)
    
    # # Debugging
    # print('x:',round(x,2),' y:',round(y,2),' t:',round(theta,2))
    # print('xd:',round(xd_d,2),' yd:',round(yd_d,2))
    # print('ex:',round(xd_d-x,2),'ey:',round(xd_d-x,2))
    # plt.plot(xd_d,yd_d, 'b*',x,y,'ro')
    # plt.legend('Desired','Actual')
    # plt.axis("equal")
    # ax_lim = 5
    # plt.xlim((-ax_lim,ax_lim))
    # plt.ylim((-ax_lim,ax_lim))
    # plt.draw()
    # plt.pause(0.00000000001)
   


rospy.init_node('pathPlanner', anonymous=True)
sub = rospy.Subscriber('/gazebo_model_states_throttled',ModelStates,callback)
pub = rospy.Publisher('pathPlannerTopic', Float64MultiArray, queue_size = 10)

rospy.spin()



