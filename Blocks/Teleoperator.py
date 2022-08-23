import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Used to Imitate the Movements of the Operator
    It takes in an array as input, depending on the array variables, it will output another array
    containing the velocity it deems appropriate.
    The linear_velocity can be given via the `Linear` parameter.

    The output data is a list of the format: `[ linear_velocity, angular_velocity ]`\n
    This is then shared to the output wire using the `share_array()` function.
    '''
    auto_enable = True
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    linear_velocity = parameters.read_number("Linear")

    while(auto_enable or inputs.read_number('Enable')):
        msg = inputs.read_array("Inp")

        # (x, y, width, height)
        x, y = float(msg[0]), float(msg[1])
        x1, y1 = x+float(msg[2]), y+float(msg[3]) 

        # Teleoperator Control Logic
        cx = (x+x1)/2.0
                
        if cx < 320:
            angular_velocity = -0.5
        else:
            angular_velocity = 0.5
        
        data = [linear_velocity, angular_velocity]
        outputs.share_array("Out", data)
        
        synchronise()