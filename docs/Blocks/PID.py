import numpy as np
import math
from time import sleep

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Applies PID for a Given Error Value
    The error is read as an input from the `inputs wire`.\n
    The Kp, Ki, and Kd parameters are read from the parameters of the same name.
    Once there it applies the PID technique to the error variable in order to minimize it.

    The resulting values are shared through the `share_array()` function. 
    
    **Inputs**: Error

    **Outputs**: `cmd_vel` (Linear Velocity, Angular Velocity)

    **Parameters**: Kp, Ki, Kd
    '''
    auto_enable = True
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    kp = parameters.read_number("Kp")
    ki = parameters.read_number("Ki")
    kd = parameters.read_number("Kd")

    previousError, I = 0, 0
    # Problem this should be inside the while loop
    msg = inputs.read_number("Inp")

    while(auto_enable or inputs.read_number('Enable')):
        if msg is None:
            continue

        error = float(msg)
        sleep(0.01)

        P = error
        I = I + error
        D = error - previousError
        PIDvalue = (kp*P) + (ki*I) + (kd*D)
        previousError = error

        linear_velocity = 5.0
        angular_velocity = -PIDvalue

        data = [linear_velocity, angular_velocity]
        outputs.share_array("Out", data)
        synchronise()