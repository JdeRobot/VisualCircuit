import cv2 as cv

def camera(wires_list, input_wires, output_wires, parameters):

   output_wire_1 = output_wires[0]
   cap = cv.VideoCapture(0)

   while(True):
        ret, frame = cap.read()
        wires_list[int(output_wire_1)].write(frame)
