import cv2
import numpy as np
from time import sleep
from wires.wire_str import Wire_Write

def TemplateWrite(input_wires, output_wires, parameters):

    to_write = np.array(['red', 'blue', '1.212'], dtype='<U6')

    shm_w = Wire_Write(output_wires[0])

    try:
        while True:
            shm_w.add(to_write)

    except KeyboardInterrupt:
        pass

    shm_w.release()
