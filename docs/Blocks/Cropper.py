import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Crops an Image\n
    The image which is to be cropped is read through the inputs using the `inputs.read_image()` function.
    The parameters ask for **x**, **y**, **w**, **h**\n
    ```
    x: x co-ordinate of where the crop should start\n
    y: y co-ordinate of where the crop should start\n
    w: width of the crop\n
    h: height of the crop\n
    ```
    Image is cropped by simple list slicing.\n
    `while` loop is the part of the program that is executed continuously.
    It is enabled by default but can be disabled by passing in 0 through the enable wire.
    Output is shared via `share_image()`

    **Inputs**: BGR Image

    **Outputs**: Resized BGR Image

    **Parameters**: x, y, width, height
    '''
    x, y, w, h = np.array([int(x.strip()) for x in parameters.read_string("xywh").split(",")])
    
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    while(auto_enable or inputs.read_number('Enable')):
        frame = inputs.read_image("Img")
        if frame is None:
            continue

        
        cropped_img = frame[y:y+h, x:x+w, :]
        outputs.share_image('Out', cropped_img)

        synchronise()