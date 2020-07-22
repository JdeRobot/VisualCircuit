#!/usr/bin/env python3

import rospy
import numpy as np
from time import sleep
from wires.wire import Wire_Read
from geometry_msgs.msg import Twist

linear_velocity, angular_velocity = 0.0, 0.0

def callback(msg):
    
    global linear_velocity, angular_velocity
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

def MotorDriverROS(input_wires, output_wires, parameters):

    rospy.init_node("turtlesim_motor", anonymous=True)
    output_wire_1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    
    input_wire_1 = rospy.Subscriber(input_wires[0], Twist, callback)
    
    data = Twist()
    while not rospy.is_shutdown():
    
        data.linear.x = linear_velocity
        data.angular.z = angular_velocity
        output_wire_1.publish(data)

