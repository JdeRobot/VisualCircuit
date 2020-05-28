import cv2 as cv

def camera(wires_list, input_wires, output_wires, parameters):

   cap = cv.VideoCapture(0)

   output_wire_1 = [wire for wire in wires_list if output_wires[0] == wire.id][0]

   while(True):
        ret, frame = cap.read()
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        output_wire_1.write(frame)
