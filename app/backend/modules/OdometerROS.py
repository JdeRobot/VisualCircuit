#!/usr/bin/env python3

import numpy as np
import rospy
from turtlesim.msg import Pose
from time import sleep


x, y, yaw = 0.0, 0.0, 0.0

def callback(msg):
    
    global x, y, yaw
    x = msg.x
    y = msg.y
    yaw = msg.theta

def OdometerROS(input_wires, output_wires, parameters):

    rospy.init_node("turtlesim_odometer", anonymous=True)
    
    data = Pose()
    odometer_subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback)

    topic_name = output_wires[0]
    output_wire_1 = rospy.Publisher(topic_name, Pose, queue_size=10)
    
    while not rospy.is_shutdown():
    
        data.x = x
        data.y = y
        data.theta = yaw
        output_wire_1.publish(data)

    
