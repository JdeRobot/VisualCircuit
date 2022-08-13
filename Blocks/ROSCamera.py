import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
img = None

def callback(msg):
    global img
    img = np.asarray(bridge.imgmsg_to_cv2(msg, "bgr8"), dtype=np.uint8)

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Gets Image from a ROSCamera
    The camera topic is read from the `ROSTopic` parameter, by default it is `/robot/camera`\n

    The image message is converted to OpenCV compatible format via the `imgmsg_to_cv2()` function.
    
    This is then shared ahead using the `share_image()` function.
    '''
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    rospy.init_node("camera_ros", anonymous=True)
    subscriber_name = parameters.read_string("ROSTopic")
    camera_subscriber = rospy.Subscriber(subscriber_name, Image, callback)

    while(auto_enable or inputs.read_number('Enable') and not rospy.is_shutdown()):
        if img is None:
            continue
        
        outputs.share_image("Out", img)
        synchronise()
