import cv2 as cv

def screen(wires_list, input_wires, output_wires, parameters):

    input_wire_1 = input_wires[0]

    while(True):
        img = wires_list[int(input_wire_1)].read()

        if img is not None:
            cv.imshow("Monitor", img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
