import cv2


def main(inputs, outputs, parameters, synchronise):
    '''
    ## Opens your Camera using OpenCV\n
    The Camera block opens your webcam using OpenCV and begins capturing the video feed.
    This video feed is then propagated forward through the `share_image()` function

    `while` loop is the part of the program that is executed continuously.
    It is enabled by default but can be disabled by passing in 0 through the enable wire.

    **Inputs**: None

    **Outputs**: BGR Image

    **Parameters**: None
    '''
    cap = cv2.VideoCapture(0)
    auto_enable = False
    try:
        enable = inputs.read_number('Enable')
    except Exception:
        auto_enable = True
    try:
        while cap.isOpened() and (auto_enable or inputs.read_number('Enable')):
            ret, frame = cap.read()
            if not ret:
                continue

            outputs.share_image("Img", frame)
            synchronise()
    except Exception as e:
        print('Error:', e)
        pass
    finally:
        print("Exiting")
        cap.release()