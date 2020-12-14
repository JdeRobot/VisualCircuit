import cv2
import numpy as np
from time import sleep
from utils.wires.wire_img import share_image
from utils.tools.freq_monitor import monitor_frequency

def loop(block_name, input_wires, output_wires, parameters, flags):

    if flags[0] == 1:
        ticks = np.array([0])
        monitor_frequency(block_name, ticks)

    output_0 = share_image(output_wires[0])

    
    try:
        while True:
            
            '''
            Write program logic here
            img contains the image which is to be shared.
            Use sleep(sleep_value) to control frequency

            Your Image Goes Below (Any Resolution)
            '''   
            
            img = np.array((640,480,3), dtype=np.uint8)
            output_0.add(img)
            
    except KeyboardInterrupt:
        shm_w.release()


    
