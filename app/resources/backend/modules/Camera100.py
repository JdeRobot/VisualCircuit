import cv2
import numpy as np
from time import sleep
from utils.wires.wire_img import share_image
from utils.tools.freq_monitor import monitor_frequency

def loop(block_name, input_wires, output_wires, parameters, flags):

    ticks = np.array([0])
    if flags[0] == 1:
        monitor_frequency(block_name, ticks)

    cap = cv2.VideoCapture(0)

    output_0 = share_image(output_wires[0])

    try:
        while True:
            
            ticks += 1
            ret, frame = cap.read()
            output_0.add(frame)
            
    except KeyboardInterrupt:
        output_0.release()