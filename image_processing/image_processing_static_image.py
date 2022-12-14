# MEHMET UYSAL
import cv2
import numpy as np


net = cv2.dnn.readNet('../../yolo_custom_detection/yolov3_balloons.weights',
                      '../../yolo_custom_detection/yolov3_testing_balloons.cfg')
classes = ["balloon"]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

#loading image
img = cv2.imread('../../images/test/3.jpg')
img = cv2.resize(img, None, fx=0.5, fy=0.5)
height, width, channels = img.shape

# object detection
blob = cv2.dnn.blobFromImage(img, 0.00392, (640, 640), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

#showing informations on the screen
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)

            #rectangle coordinates
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)

            boxes.append([x,y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)


indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
font = cv2.FONT_HERSHEY_PLAIN
for i in range(len(boxes)):
    if i in indexes:
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, label, (x, y+30), font, 1, (0, 255, 0))

cv2.imshow("MEHMET UYSAL", img)
cv2.waitKey(0)
cv2.destroyAllWindows()


