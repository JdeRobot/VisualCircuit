import numpy as np
import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

data = []

def callback(msg):
    '''
    The callback function is required by the Subscriber to the ROSTopic. This callback function reads the orientation list from the IMU
    It then converts the quaternion angles to euler ones. This gives us the roll, pitch and yaw of the body.
    We convert these radian values to degrees to get the orientation of the body.

    Aside from these values the IMU also gives us the angular velocity of the body.\n
    All of these values are stored in the global `data` variable of the block.
    '''
    global data
    # Get the orientation list from the IMU sensor
    orientation_list = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    # Convert orientation obtainted into values of roll, pitch and yaw
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    # Convert val;ues in Radians to Degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    # Obtain Angular Velocities from IMU sensor
    angVel_x = msg.angular_velocity.x
    angVel_y = msg.angular_velocity.y
    angVel_z = msg.angular_velocity.z
    # Store all this data in 'data' variable for use in main function
    data = [roll, pitch, yaw, angVel_x, angVel_y, angVel_z]

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Reads IMU sensor data
    This is a specialized block used to read IMU sensor data.

    It reads the ROSTopic name from the `ROSTopic` parameter. Default is `mavros/imu/data`.\n
    This data is sent to the callback function which converts the orientation list obtained into roll, pitch and yaw for the
    robot that the IMU is present on. Alongwith orientation, it also gives the angular velocity of the robot.
    This data is shared in the form of an array using the `share_array()` function.
    
    **Inputs**: None

    **Outputs**: Array [Roll, Pitch , Yaw, Angular Velocity in X, Angular Velocity in Y, Angular Velocity in Z]

    **Parameters**: ROSTopic
    '''
    global data 
    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True

    rospy.init_node('imu_vc')
    rostopic_name = parameters.read_string("ROSTopic")
    rospy.Subscriber(rostopic_name, Imu, callback)

    while ((auto_enable or inputs.read_number('Enable')) and not rospy.is_shutdown()):
        if not data:
            continue

        outputs.share_array("Out", data)
        synchronise()