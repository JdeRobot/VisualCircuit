import cv2
import numpy as np
from time import sleep
from utils.wires.wire_img import read_image
from utils.tools.freq_monitor import monitor_frequency

def loop(block_name, input_wires, output_wires, parameters, flags):

    # To monitor block frequency
    ticks = np.array([0])
    if flags[0] == 1:
        monitor_frequency(block_name, ticks)

    input_0 = read_image(input_wires[0])
    
    try:
        while True:
            
            ticks[0] += 1

            img = input_0.get()

            '''
            Write program logic here
            img contains the shared image which was read.
            Use sleep(sleep_value) to control frequency
            '''
            
    # Process Destructor        
    except KeyboardInterrupt:
        shm_r.release()
