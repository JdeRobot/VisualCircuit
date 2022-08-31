import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Streams Video from File
    The filepath of your video is given in the `PathToFile` parameter.
    *Note:* that this file path is relative to the `modules` folder of the final built application.

    Capturing begins using the `cv2.VideoCapture()` function. 
    The video is then read frame by frame and each frame is shared to the output wire using the
    `share_image()` function.
    
    **Inputs**: None

    **Outputs**: BGR Image

    **Parameters**: PathToFile
    '''
    filepath = parameters.read_string("PathToFile")
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    cap = cv2.VideoCapture(filepath)

    while(auto_enable or inputs.read_number('Enable') and cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            outputs.share_image('Out', frame)
        else:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            
        synchronise()