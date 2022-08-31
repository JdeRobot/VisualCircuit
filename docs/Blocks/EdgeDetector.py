import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Detects Edges in an Image\n
    It takes in two parameters `Lower` and `Upper`. These parameters are used as the limits in Canny Edge 
    Detection. First we convert the input `BGR` image to `GRAY`. Next we apply Canny Edge Detection via the 
    `cv2.Canny()` function. The resulting image is then converted back to `BGR`.

    This image is then shared to the wire via the `share_image()` function.
    
    **Inputs**: BGR Image

    **Outputs**: BGR Image

    **Parameters**: Lower, Upper (Threshold values)
    '''
    lower = int(parameters.read_string("Lower"))
    upper = int(parameters.read_string("Upper"))

    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    while(auto_enable or inputs.read_number('Enable')):
        frame = inputs.read_image("Img")
        if frame is None:
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edge_img = cv2.Canny(frame, lower, upper)
        edge_img = cv2.cvtColor(edge_img, cv2.COLOR_GRAY2BGR)
        
        outputs.share_image('Out', edge_img)

        synchronise()