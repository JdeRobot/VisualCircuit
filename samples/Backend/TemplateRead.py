import numpy as np
from time import sleep
from wires.wire_str import Wire_Read

def TemplateRead(input_wires, output_wires, parameters):

    shm_r = Wire_Read(input_wires[0])
    
    try:
        while True:
            sleep(0.03)
            data = shm_r.get()
            print(data)

    except KeyboardInterrupt:
        pass

    shm_r.release()
