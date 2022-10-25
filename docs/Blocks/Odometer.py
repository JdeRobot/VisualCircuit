import numpy as np
import rospy
from turtlesim.msg import Pose

x, y, yaw = 0.0, 0.0, 0.0

def callback(msg):
    global x, y, yaw
    x = msg.x
    y = msg.y
    yaw = msg.theta

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Reads Data from An Odometer
    It reads the ROSTopic name from the `ROSTopic` parameter.
    It then initializes a Subscriber to subscribe to that ROSTopic, once the data is obtained through the callback
    function, it is formatted into an array with the format: `[ x, y, yaw ]`\n
    This data is then shared to the wire using the `share_array()` function.
    
    **Inputs**: None

    **Outputs**: Array [X, Y, Yaw]

    **Parameters**: ROSTopic
    '''
    rospy.init_node("odometerVC", anonymous=True)
    rostopic_name = parameters.read_string("ROSTopic")
    odometer_subscriber = rospy.Subscriber(rostopic_name, Pose, callback)

    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    while(auto_enable or inputs.read_number('Enable') and not rospy.is_shutdown()):
        data = [x, y, yaw]

        outputs.share_array("Out", data)

        synchronise()