import cv2

def main(inputs, outputs, parameters, synchronise):
    '''
    ## Displays the given Image

    Takes an image as an input and displays it on the user's screen.
    The `cv2.imshow()` function is used in order to display the image.
    
    **Inputs**: BGR Image

    **Outputs**: None

    **Parameters**: None
    '''
    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True
    while (auto_enable or inputs.read_number('Enable')):
        img = inputs.read_image("Img")
        if img is None:
            continue

        cv2.imshow("frame", img)
        cv2.waitKey(10)

        synchronise()