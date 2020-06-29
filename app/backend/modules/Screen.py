import cv2
import numpy as np
from time import sleep
from wire import Wire_Read

def Screen(input_wires, output_wires, parameters):

    print("screen")
    print(input_wires)
    print(output_wires)

    shm_r = Wire_Read(input_wires[0])

    while True:
        sleep(0.03)
        f = shm_r.get()
        cv2.imshow('Screen', f)
        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

    shm_r.release()
