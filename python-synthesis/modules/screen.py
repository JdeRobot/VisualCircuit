import cv2 as cv

def screen(wires_list, input_wires, output_wires, parameters):

    input_wire_1 = [wire for wire in wires_list if input_wires[0] == wire.id][0]

    while(True):
        img = input_wire_1.read()

        if img is not None:
            cv.imshow("Monitor", img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
