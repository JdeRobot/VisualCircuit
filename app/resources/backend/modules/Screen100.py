import cv2
import numpy as np
from time import sleep
from utils.wires.wire_img import read_image
from utils.tools.freq_monitor import monitor_frequency

def loop(block_name, input_wires, output_wires, parameters, flags):

    ticks = np.array([0])
    if flags[0] == 1:
        monitor_frequency(block_name, ticks)

    input_0 = read_image(input_wires[0])

    try:
        while True:
        
            sleep(0.03)
            ticks[0] += 1
            img = input_0.get()
            if img is not None:
                cv2.imshow('Screen', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    except KeyboardInterrupt:
        input_0.release()