import cv2 as cv

def edge_detector(wires_list, input_wires, output_wires, parameters):

    input_wire_1 = input_wires[0]
    output_wire_1 = output_wires[0]
    parameter_1 = int(parameters[0])

    while(True):
        img = wires_list[int(input_wire_1)].read()
        if img is not None:
            img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            wires_list[int(output_wire_1)].write(cv.Canny(img, 100, 200, 100, parameter_1))
