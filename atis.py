import cv2, serial, time, gorev, numpy as np


cam_no = 0

cap = cv2.VideoCapture(cam_no)

mission = gorev.Mission()

mission.baglan(ip_port="192.168.43.86:10000")

#port = serial.Serial("port_name")

#hsv boundries
turuncu_lr = np.array([0, 144, 92])
turuncu_hr = np.array([179, 255, 255])

turuncu2_lr = np.array([0, 168, 135])
turuncu2_hr = np.array([179, 255, 255])

red_lr = np.array([])
red_hr = np.array([])

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


def main():
    gimbal_yaw = 90
    gimbal_pitch = 90
    frame_counter = 0
    start_time = time.time()
    while_gimbal_yaw = 0
    while_gimbal_pitch = 0
    #port.write(b"KONTROL")
    fps = 10 #sadece init etmek icin!!!
    while True:
        frame_counter += 1
        _, frame = cap.read()
        frame = cv2.resize(frame, None, fx=mission.size, fy=mission.size)
        if cam_no == 0:
            frame = cv2.flip(frame, 1)

        #------------------------goruntu isleme-------------------------------------
        """hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame,turuncu_lr,turuncu_hr)
        contours, hirearchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            x, y, w, h = cv2.boundingRect(sorted(contours, key=cv2.contourArea, reverse=True)[0])
            if (60 < w < 200) and (60 < h < 200):
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                _yon = mission.yon((x, y, w, h))"""

        angular_yaw_velocity = 2
        angular_pitch_velocity = 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)

        if len(faces) > 0:
            for (x, y, w, h) in faces:
                roi_gray = gray[y:y + h, x:x + w]

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, ((x + (w // 2)), (y + (h // 2))), 10, (0, 250, 0), cv2.FILLED)

                #print(mission.yon((x, y, w, h)))

                _yon = mission.yon((x, y, w, h))

                # gimbal hareketi
                if _yon[0] == "m":
                    #self.port.write("FREON")
                    print("FREON")
                    pass
                else:
                    #self.port.write("FREOFF")
                    if _yon[-1] == "d":
                        #--------fps assagida tanimli bug cikarabilir!--------
                        while_gimbal_yaw += angular_yaw_velocity / (int(fps) / 2)
                        if while_gimbal_yaw > angular_yaw_velocity:
                            gimbal_yaw += angular_yaw_velocity
                            msg = f"YAW{gimbal_yaw}"
                            # self.port.write(bytes(msg, "utf-8"))
                            print(msg)
                            while_gimbal_yaw = 0
                    elif _yon[-1] == "b":
                        while_gimbal_yaw += angular_yaw_velocity / (int(fps) / 2)
                        if while_gimbal_yaw > angular_yaw_velocity:
                            gimbal_yaw -= angular_yaw_velocity
                            msg = f"YAW{gimbal_yaw}"
                            # self.port.write(bytes(msg, "utf-8"))
                            print(msg)
                            while_gimbal_yaw = 0
                    if _yon[0] == "g":
                        while_gimbal_pitch += angular_yaw_velocity / (int(fps) / 2)
                        if while_gimbal_pitch > angular_pitch_velocity:
                            gimbal_pitch += angular_pitch_velocity
                            msg = f"PIT{gimbal_pitch}"
                            # self.port.write(bytes(msg, "utf-8"))
                            print(msg)
                            while_gimbal_pitch = 0
                    elif _yon[0] == "k":
                        # gimbal yukari bakmayacak
                        pass


        #hedef alani cizdirme
        cv2.rectangle(frame, (305, 159), (330, 184), (170, 20, 125), 3)

        #izgara cizdirme
        for pt1, pt2 in mission.izgara:
            cv2.rectangle(frame, pt1, pt2, (255, 0, 255))

        elapsed_time = time.time() - start_time
        fps = frame_counter / elapsed_time
        cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), mission.font, 2, (0, 0, 0), 2)

        cv2.imshow("frame", frame)

        komut = cv2.waitKey(20)
        if komut & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


main()
