import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Detects Contours in an Image\n
    The image in which contours are to be detected is read through the inputs.
    First the image is converted from `BGR` to `Grayscale`, the thresholding values are `60, 255`.
    The function used is `cv2.threshold()`.
    Once it is thersholded, the contours are detected in the image using `cv2.findContours()`

    The program then detects the biggest contour present in the image and finds the co-ordinates of its center
    using the `cv2.moments()` function. 

    This co-ords of the center alongwith the contour characteristics are part of the output array.
    THis array is shared through `share_array()`

    `while` loop is the part of the program that is executed continuously.
    It is enabled by default but can be disabled by passing in 0 through the enable wire.

    [Further reading](https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html)
    '''
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    while(auto_enable or inputs.read_number('Enable')):
        img = inputs.read_image("Img")
        if img is None:
            continue

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(img_gray, 60, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)

        # Find the biggest contour (if detected)
        if len(contours) > 0:
            cont = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(cont) # Get the minimum area bounding rectangle for the contour
            # ( center (x,y), (width, height), angle of rotation )

            # Get the moments of the image
            M = cv2.moments(cont)
            if M['m00'] != 0:
                # Get the coords of the center of the moments
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            data = [cx, cy, rect[1][0], rect[1][1], rect[2]]
            # Send the data to the wire in the form of an array 
            outputs.share_array('Out', data)

        synchronise()
