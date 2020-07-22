import cv2
import numpy as np
from time import sleep
from wires.wire_img import Wire_Write

def Camera(input_wires, output_wires, parameters):

    cap = cv2.VideoCapture(0)

    shm_w = Wire_Write(output_wires[0])

    try:
        while True:
            ret, frame = cap.read()
            shm_w.add(frame)
    except KeyboardInterrupt:
        pass

    shm_w.release()
