import cv2
import numpy as np

balon_cascade = cv2.CascadeClassifier('/media/yesill/Yerel Disk/Programlama/Projeler/uav/cascades/cascades/91p2381n16stage.xml')

cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    objects = balon_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=8)
    for (x,y,w,h) in objects:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
        cv2.putText(frame, f"balon", (x, y+5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 2)
        roi_color = frame[y:y+h, x:x+w]

    cv2.imshow("title", frame)

    command = cv2.waitKey(1)
    if command & 0xFF == ord('q'):
        print(f'program sonlandirildi')
        break
    elif command & 0xFF == ord('r'):
        print(f'kirmizi balonlağg tespit ediliyoğg')
    elif command & 0xFF == ord('g'):
        print(f'yeschill balonlağg tespit ediliyoğg')
    elif command & 0xFF == ord('b'):
        print(f'movi balonlağg tespit ediliyoğg')

cap.release()
cv2.destroyAllWindows()
