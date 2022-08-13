import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    """
    ## Blurs an Object\n
    The object to be blurred is read through the inputs.
    We have multiple available blurs including Gaussian, Averaging and Median Blur. 
    We can change these blurs by changing the name given in the parameter block

    `while` loop is the part of the program that is executed continuously.
    It is enabled by default but can be disabled by passing in 0 through the enable wire.

    Outputs the blurred image through the `share_image()` function
    """
    # Blur Type
    blur_type : str = parameters.read_string("BlurType") 
    """Type of blur, reads from a parameter"""
    # Kernel Size
    kernel = tuple([int(x.strip()) for x in parameters.read_string("Kernel").split(',')])
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    while(auto_enable or inputs.read_number('Enable')):
        frame = inputs.read_image("Img")
        if frame is None:
            continue

        if blur_type == 'Gaussian':
            blurred_img = cv2.GaussianBlur(frame, kernel, 0)
                        
        elif blur_type == 'Averaging':
            blurred_img = cv2.blur(frame, kernel)
                        
        elif blur_type == 'Median':
            blurred_img = cv2.medianBlur(frame, kernel[0])

        outputs.share_image('Out', blurred_img)
        synchronise()
