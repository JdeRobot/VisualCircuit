import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Erodes an Image\n
    You can specify the kernel dimensions and number of iterations in the parameters.

    We first convert the colour of the image from `BGR` to `GRAY` then we apply erosion on it 
    using the `cv2.erode()` function.\n
    Finaly we convert from `GRAY` back to `BGR` and output the image through the `share_image()` function.
    
    [Further reading]([Further reading](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html))
    
    **Inputs**: BGR Image

    **Outputs**: BGR Image

    **Parameters**: Kernel, Iterations
    '''
    kernel = np.array([int(x.strip()) for x in parameters.read_string("Kernel").split(",")])
    kernel = np.ones(kernel, np.uint8)
    iters = int(parameters.read_string("Iterations"))
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
        dilated = cv2.erode(frame, kernel, iterations = iters)
        dilated = cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR)
        
        outputs.share_image('Out', dilated)

        synchronise()