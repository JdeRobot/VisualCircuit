import cv2
import numpy as np
import time

className = []
classesFile = 'utils/models/yolov3/yolov3.txt'

with open(classesFile,'rt') as f:
    className = f.read().rstrip('\n').split('\n')

def findObjects(outputs, img):
    '''
    This function is used for drawing highlighted information over the image.\n
    According to the output given by the model, the function tags all of the detected objects in the frame.
    It then draws bounding boxes over all the detected objects. The tags are also displayed on the screen.
      
    '''
    confThreshold = 0.3
    nmsThreshold = 0.3
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []

    for output in outputs:
    
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w,h = int(det[2] * wT), int(det[3] * hT)
                x,y = int((det[0]*wT) - w/2), int((det[1]*hT) - h/2)
                bbox.append([x,y,w,h])
                classIds.append(classId)
                confs.append(float(confidence))
    
    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)
    
    for i in range(len(indices)):
        box = bbox[i]
        x,y,w,h = box[0],box[1],box[2],box[3]
        cv2.rectangle(img,(x,y),(x+w, y+h),(255,0,255),2)
        cv2.putText(img,f'{className[classIds[i]].upper()} {int(confs[i]*100)}%',
                    (x,y-10),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255),2)
           
def main(inputs, outputs, parameters, synchronise):
    '''
    ## Detects Objects in the Image
    The model used for object detection is the `yolov3-tiny` model.

    Currently the block utilizes the host's CPU in order to carry out computation for the model.
    Objects in the image are identified using the model. What type of object it is, if inferred is added as a 
    tag to it, a bounding box is also drawn over it.

    This image is then shared to the output wire using the `share_image()` function.
    
    **Inputs**: BGR Image

    **Outputs**: BGR Image with Bounding Boxes

    **Parameters**: None
    '''
    auto_enable = True
    try:
        enable = inputs.read_number("Enable")
    except Exception:
        auto_enable = True
    
    whT = 320

    modelConfiguration = 'utils/models/yolov3/yolov3-tiny.cfg'
    modelWeights = 'utils/models/yolov3/yolov3-tiny.weights'

    net = cv2.dnn.readNetFromDarknet(modelConfiguration,modelWeights)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    while(auto_enable or inputs.read_number('Enable')):
        frame = inputs.read_image("Img")
        
        if frame is None:
            continue

        #converting img to blob
        blob = cv2.dnn.blobFromImage(frame, 1/255,(whT,whT),[0,0,0],1, crop = False)

        #Passing blob to network
        net.setInput(blob)

        layerNames = net.getLayerNames()
        outputNames = []
        outLayers = net.getUnconnectedOutLayers()
        
        for i in range(len(outLayers)):
            for j in range(len(outLayers[i])):
                outputNames.append(layerNames[outLayers[i][j] - 1])

        #forward Pass
        results = net.forward(outputNames)
        findObjects(results,frame)

        outputs.share_image("Out", frame)
