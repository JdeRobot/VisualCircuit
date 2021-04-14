import cv2 as cv
import numpy as np
from time import sleep
from utils.wires.wire_img import share_image, read_image
from utils.tools.freq_monitor import monitor_frequency

def loop(block_name, input_wires, output_wires, parameters, flags):

    ticks = np.array([0])
    if flags[0] == 1:
        monitor_frequency(block_name, ticks)

    input_0 = read_image(input_wires[0])
    output_0 = share_image(output_wires[0])

    lower_range = np.array([int(x.strip()) for x in parameters[0].split(',')])
    upper_range = np.array([int(x.strip()) for x in parameters[1].split(',')])

    while True:

        sleep(0.01)
        ticks += 1
        frame = input_0.get()
        
        if frame is not None:

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, lower_range, upper_range)
            filtered = cv.bitwise_and(frame,frame, mask= mask)
    
            output_0.add(filtered)

    input_0.release()
    output_0.release()