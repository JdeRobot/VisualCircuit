import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Reads an Image from a Specified Path
    This box reads an image from a given file path. The path to be specified is written in the parameter
    `ImagePath`.\n
    It is read through the `cv2.imread()` function and shared through the `share_image()` function.
    '''
    path = parameters.read_string("ImagePath")
    image = cv2.imread(path)
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    while(auto_enable or inputs.read_number('Enable')):
        outputs.share_image('Out', image)
        synchronise()


        