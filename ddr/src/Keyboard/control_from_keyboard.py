#!/usr/bin/env python

# This .py read control inputs from keyTopic messages and publish che converted control input v,w on the command topic
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

def kinematicInversion(v,w):
    w_left = (v + 0.3*w)/0.04
    w_right = (v - 0.3*w)/0.04
    wheel_velocities = [w_left, w_right]
    return wheel_velocities

def getInputVelocity(msg): # get input velocity from keyboard

    # Get desidered linear and angular velocity from the topic and convert them
    vx = msg.linear.x
    vy = msg.linear.y
    vz = msg.linear.z
    wx = msg.angular.x
    wy = msg.angular.y
    wz = msg.angular.z
    wheel_velocities = kinematicInversion(vx, wz)

    # Publish the control action /command topic
    controlAction.data = wheel_velocities
    pub.publish(controlAction)

# Initializing the node
rospy.init_node('control_from_keyboard', anonymous=True)

# Let the node to publish and subscribe to/on a topic
sub = rospy.Subscriber('keyboardTopic',Twist,getInputVelocity) 
pub = rospy.Publisher('ddr_joint_state_controller/command', Float64MultiArray,queue_size = 10)

# Rate of the control action
rate = rospy.Rate(50)

# Declare a message with Float64MultiArray layout
controlAction = Float64MultiArray()

# Process everithing until ctrl+c
rospy.spin()

