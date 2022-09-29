import cv2, time


cap = cv2.VideoCapture(1)

file_name_sayac = 0
size = 1
font = cv2.FONT_HERSHEY_PLAIN
start_time = time.time()
frame_sayac = 0

izgara = [((0, 0), (305, 159)),
          ((305, 0), (330, 159)),
          ((330, 0), (639, 159)),
          ((0, 159), (305, 184)),
          ((305, 159), (330, 184)),
          ((330, 159), (639, 184)),
          ((0, 184), (305, 479)),
          ((305, 184), (330, 479)),
          ((330, 184), (639, 479))]

while True:
    frame_sayac += 1

    _, frame = cap.read()

    frame = cv2.resize(frame, None, fx=size, fy=size)
    #frame = cv2.flip(frame, 1)

    y, x, _ = frame.shape

    #print(f"{x}, {y}")

    #kamera merkezi
    cv2.circle(frame, (int(x/2), int(y/2)), 3, (0, 255, 0), cv2.FILLED)
    cv2.circle(frame, (367, 182), 8, (0, 255, 0), thickness=1)
    cv2.circle(frame, (379, 157), 8, (255, 0, 0), thickness=1)
    cv2.circle(frame, (346, 154), 8, (0, 0, 255), thickness=1)


    #cv2.rectangle(frame, (340, 150), (380, 190), color=(125, 0, 125), thickness=1)

    #hedef alani
    cv2.rectangle(frame, (330, 184), (305, 159), color=(0, 0, 255), thickness=3)

    """cv2.rectangle(frame, izgara[0][0], izgara[0][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[1][0], izgara[1][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[2][0], izgara[2][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[3][0], izgara[3][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[4][0], izgara[4][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[5][0], izgara[5][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[6][0], izgara[6][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[7][0], izgara[7][1], color=(255, 0, 255), thickness=2)
    cv2.rectangle(frame, izgara[8][0], izgara[8][1], color=(255, 0, 255), thickness=2)"""

    #fps
    fps = frame_sayac / (time.time() - start_time)

    #cv2.putText(frame,round(fps,2),(10,50),font,2,(255,255,255), 2)

    cv2.imshow("frame", frame)

    komut = cv2.waitKey(1)
    if komut & 0xFF == ord('q'):
        break
    elif komut & 0xFF == ord('a'):
        cv2.imwrite(f"{file_name_sayac}.png", frame)
        file_name_sayac += 1

cap.release()
cv2.destroyAllWindows()



