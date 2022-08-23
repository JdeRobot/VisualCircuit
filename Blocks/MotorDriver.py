import rospy
import numpy as np
from time import sleep
from geometry_msgs.msg import Twist

linear_velocity, angular_velocity = 0.0, 0.0

def callback(inp):
    global linear_velocity, angular_velocity
    

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Publishes Twist Command to drive Motors
    It publishes to the ROSTopic name from the `ROSTopic` parameter. Default is `/robot/cmd_vel`.\n
    It reads an array as an input by the `read_array()` function.\n
    This is assumed to be of the format `[ linear_velocity, angular_velocity ]`.\n
    This data is then converted into a Twist() message with the `linear.x = linear_velocity` and `angular.z = angular_velocity`
    
    The data is then published continuously 
    '''
    rospy.init_node("motordriverVC", anonymous=True)

    rostopic_name = parameters.read_string("ROSTopic")
    # Create a Publisher that publishes to the given ROSTopic
    publisher = rospy.Publisher(rostopic_name, Twist, queue_size=10)
    
    auto_enable = True
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True
        
    # Create a Twist message
    data = Twist()

    while(auto_enable or inputs.read_number('Enable') and not rospy.is_shutdown()):
        msg = inputs.read_array("Inp")

        if msg is None:
            continue

        linear_velocity = float(msg[0])
        angular_velocity = float(msg[1])

        data.linear.x = linear_velocity
        data.angular.z = angular_velocity
        publisher.publish(data)

        synchronise()