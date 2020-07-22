import numpy as np
from time import sleep
from wires.wire_img import Wire_Write

def ImageWrite(input_wires, output_wires, parameters):

    # Your Image Goes Here
    img = np.array((640,480,3), dtype=np.uint8)

    shm_w = Wire_Write(output_wires[0])

    try:
        while True:
            shm_w.add(img)
            
    except KeyboardInterrupt:
        pass

    shm_w.release()
