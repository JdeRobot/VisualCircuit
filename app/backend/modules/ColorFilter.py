import cv2 as cv
import numpy as np
from time import sleep
from wires.wire_img import Wire_Read, Wire_Write

def ColorFilter(input_wires, output_wires, parameters):

    shm_r = Wire_Read(input_wires[0])
    shm_w = Wire_Write(output_wires[0])

    while True:

        sleep(0.03)
        frame = shm_r.get()

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        lower = np.array([100,150,0])
        upper = np.array([140,255,255])

        mask = cv.inRange(hsv, lower, upper)
        filtered = cv.bitwise_and(frame,frame, mask= mask)

        shm_w.add(filtered)

    shm_r.release()
    shm_w.release()
