import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Thresholds an Image
    THis block reads the parameters `LowerThreshold` and `UpperThreshold`.\n
    Based on these values it converts the input image form `BGR` into `GRAY` and applies the `cv2.threshold()` function on it.

    The image is then converted back into `BGR` and shared to the output wire using the
    `share_image()` function.

    [Further reading](https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html)
    
    **Inputs**: BGR Image

    **Outputs**: BGR Image

    **Parameters**: LowerThreshold, UpperThreshold
    '''
    lower = parameters.read_number("LowerThreshold")
    upper = parameters.read_number("UpperThreshold")
    
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
        (T, thresh) = cv2.threshold(frame, lower, upper, cv2.THRESH_BINARY)
        output = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        
        outputs.share_image('Out', output)

        synchronise()