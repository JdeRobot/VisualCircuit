#!/usr/bin/env python3

import rospy
import math
import random
import numpy as np
from time import sleep
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


x, y, yaw = 0.0, 0.0, 0.0

def callback(msg):

    global x, y, yaw
    
    x = msg.x
    y = msg.y
    yaw = msg.theta


def navigate(x1, y1, output_wire_1):

    global x, y, yaw

    data = Twist()
    
    while True:

    	vel_const = 0.5
    	dist = abs(math.sqrt(((x1-x)**2) + ((y1-y)**2)))
    	
    	linear_velocity = dist * vel_const
    	
    	ang_const = 4.0
    	angle = math.atan2(y1-y, x1-x)
    	angular_velocity = (angle-yaw)*ang_const
    	
    	data.linear.x = linear_velocity
    	data.angular.z = angular_velocity
    	
    	output_wire_1.publish(data)

    	if dist < 0.01:
	     break
	    

def PIDControllerROS(input_wires, output_wires, parameters):

    rospy.init_node("turtlesim_pid", anonymous=True)

    input_wire_1 = rospy.Subscriber(input_wires[0], Pose, callback)
    
    output_wire_1 = rospy.Publisher(output_wires[0], Twist, queue_size=10)
    
    while not rospy.is_shutdown():
    
    	x_target = random.uniform(1.0, 7.9)
    	y_target = random.uniform(1.0, 7.9)
    	rospy.loginfo("New Target Set")
    	navigate(x_target, y_target, output_wire_1)
    
    	
    
    
    
