import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Filters Colour according to given parameters\n
    The image to be filtered is read through the inputs.
    We can give a Filter between any HSV range by changing the range of the parameters LowerRGB and UpperRGB. 

    `while` loop is the part of the program that is executed continuously.
    It is enabled by default but can be disabled by passing in 0 through the enable wire .

    Here the image is tranformed from `BGR` to `HSV` and then the filter is applied through the `cv2.inRange()`
    function. Finally the filtered image is overlayed on the orignal by the means of the
    `cv2.bitwise_and()` function. This filtered image is then shared through the `share_image()` function.
    
    **Inputs**: BGR Image

    **Outputs**: BGR Image

    **Parameters**: LowerHSV, UpperHSV
    '''
    lower_rgb = np.array([int(x.strip()) for x in parameters.read_string('LowerRGB').split(',')])
    upper_rgb = np.array([int(x.strip()) for x in parameters.read_string('UpperRGB').split(',')])

    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True

    while (auto_enable or inputs.read_number('Enable')):
        frame = inputs.read_image('Img')
        if frame is None:
            continue
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_rgb, upper_rgb)
        filtered = cv2.bitwise_and(frame, frame, mask= mask)

        outputs.share_image('Out', filtered)
        synchronise()

