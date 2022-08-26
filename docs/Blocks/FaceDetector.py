import cv2
import numpy as np

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Detects Faces in the Image
    This block applies a Harr Cascade based model on the input image. 
    It takes as an input the parameter `BoxOrImage`. This parameter has two possible values:
    `BoxOrImage: image / box`\n
    If `image` is given: The output is the image passed in with a bounding box around the area where 
    a face is detected. Image is shared through the `share_image()` function.\n
    Else if `box` is given, the output is the co-ordinates of the bounding box in the form of an array. It 
    is chared through the `share_array()` function.
    
    **Inputs**: BGR Image

    **Outputs**: BGR Image with Bounding Boxes

    **Parameters**: BoxOrImage ('box' for Bounding Boxes, 'image' for Image with Detections)
    '''
    choice = parameters.read_string("BoxOrImage")
    
    auto_enable = False
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True

    classifier = cv2.CascadeClassifier('utils/models/haar_cascade/haarcascade_frontalface_default.xml')

    while(auto_enable or inputs.read_number('Enable')):
        img = inputs.read_image("Img")
        if img is None:
            continue
        bboxes = classifier.detectMultiScale(img)
        
        if choice == 'image':
            for box in bboxes:
                x, y, x1, y1 = box
                x1, y1 = x+x1, y+y1
                cv2.rectangle(img, (x, y), (x1, y1), (0,255,0), 2)
            outputs.share_image('Out', img)
        else:
            to_write = [320, 240, 0, 0]
            for box in bboxes:
                to_write = [box[0], box[1], box[2], box[3]]
                break
            outputs.share_array("Out", to_write)
    
    synchronise()


        