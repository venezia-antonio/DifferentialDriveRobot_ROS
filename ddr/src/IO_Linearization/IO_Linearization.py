#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

def kinematicInversion(v,w):
    w_left = (v + 0.3*w)/0.04
    w_right = (v - 0.3*w)/0.04
    wheel_velocities = [w_left, w_right]
    return wheel_velocities

def getInputVelocity(tmp): 
    # get odometry and desired trajectory from pathPlanner and calculate the control
    # action following the I/O Linearization
    x = tmp.data[0]
    y = tmp.data[1]
    theta = tmp.data[2]
    x_d = tmp.data[3]
    y_d = tmp.data[4]
    xd_dot = tmp.data[5]
    yd_dot = tmp.data[6]

    # Tuning coefficients
    k1 = 2
    k2 = 2
    b = 0.15
    
    # Control Action
    u1 = xd_dot + k1*(x_d - x)
    u2 = yd_dot + k2*(y_d - y)

    # Kinematic inversion
    v = np.cos(theta)*u1 + np.sin(theta)*u2
    w = -(1/b)*np.sin(theta)*u1 + (1/b)*np.cos(theta)*u2
    wheel_velocities = kinematicInversion(v, w)

    # Publishing the control action on a topic
    controlAction.data = wheel_velocities
    pub.publish(controlAction)


# Inizialing the node, with publish and subscribe action
rospy.init_node('custom_controller', anonymous=True)
sub = rospy.Subscriber('pathPlannerTopic',Float64MultiArray,getInputVelocity)
pub = rospy.Publisher('ddr_joint_state_controller/command', Float64MultiArray,queue_size = 10)
rate = rospy.Rate(50)
controlAction = Float64MultiArray()

rospy.spin()

