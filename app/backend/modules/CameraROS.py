#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Image
from time import sleep
from wires.wire_str import Wire_Write

img = None

def callback(msg):
    
    global img
    img = msg

def CameraROS(input_wires, output_wires, parameters):

    rospy.init_node("camera_ros", anonymous=True)
    
    camera_subscriber = rospy.Subscriber("/camera/image_color", Image, callback)

    shm_w = Wire_Write(output_wires[0])
    
    while not rospy.is_shutdown():
        shm_w.write(img)
        
    shm_w.release()

    
