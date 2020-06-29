import cv2 as cv
import numpy as np
from time import sleep
from wire import Wire_Read, Wire_Write

def EdgeDetector(input_wires, output_wires, parameters):

    print("edge")
    print(input_wires)
    print(output_wires)

    shm_r = Wire_Read(input_wires[0])
    shm_w = Wire_Write(output_wires[0])
    #aperture = float(parameters[0])

    while True:

        sleep(0.03)

        frame = shm_r.get()
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        edge_img = cv.Canny(frame, 100, 200)
        edge_img = cv.cvtColor(edge_img, cv.COLOR_GRAY2BGR)

        shm_w.add(edge_img)

    shm_r.release()
    shm_w.release()
