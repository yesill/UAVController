#orion asama 2
import cv2, numpy, time, serial
from PyQt5.QtCore import QThread, pyqtSignal

#orion asama 2
class ThreadAsama2(QThread):
    end_signal = pyqtSignal(bool)
    msg_signal = pyqtSignal(str)
    kamera_signal = pyqtSignal(numpy.ndarray)
    atis_izni_signal = pyqtSignal(bool)

    def __init__(self, mission, threadKamera, port_name):
        super(ThreadAsama2, self).__init__()
        self.konumlar = mission.konumOku()
        self.mission = mission
        self.threadKamera = threadKamera
        self.frame = None
        self.calisiyor = False
        self.stoper = False

        #hsv boundries
        self.turuncu_lr = numpy.array([0, 144, 92])
        self.turuncu_hr = numpy.array([179, 255, 255])

        self.turuncu2_lr = numpy.array([0, 168, 135])
        self.turuncu2_hr = numpy.array([179, 255, 255])

        threadKamera.frame_signal.connect(self.update_frame)

    def update_frame(self, frame):
        temp_frame = frame
        #temp_frame = cv2.cvtColor(temp_frame, cv2.COLOR_BGR2RGB)
        temp_frame = cv2.flip(temp_frame, 1)
        self.frame = temp_frame

    def run(self):
        self.calisiyor = True
        self.stoper = False

        enlem = self.konumlar["asama2_start"]["lat"]
        boylam = self.konumlar["asama2_start"]["lon"]
        irtifa = self.konumlar["asama2_start"]["alt"]
        hiz = self.konumlar["asama2_start"]["spd"]
        #renk = "a"

        self.msg_signal.emit(f"Asama2 baslatildi.")

        #kalkis
        try:
            #arac kalkis yapmamis ise kalkis yap
            if self.mission.arac.location.global_relative_frame.alt < 2 and not self.mission.arac.armed:
                self.mission.kalkis(irtifa=irtifa)
                self.msg_signal.emit(f"kalkis basarili")
            time.sleep(1)
        except:
            self.msg_signal.emit(f"ERROR: Asama2 kalkis yapilamadi!")

        #roi reset
        try:
            time.sleep(1)
            # 0,0,0 means reset the roi which means follow the direction of travel
            self.mission.set_roi(lat=0, lon=0, alt=0)
            time.sleep(2)
            self.msg_signal.emit(f"Asama2 start ROI gerceklestirildi.")
        except:
            self.msg_signal.emit(f"ERROR: Asama2 ROI gerceklestirilemiyor!")

        #gorev konumu
        try:
            self.msg_signal.emit(f"Asama2 konumuna ({enlem}, {boylam}, {irtifa}m) gidiliyor.")
            self.mission.git(enlem=enlem, boylam=boylam, hiz=hiz, irtifa=irtifa)
            self.msg_signal.emit(f"Asama2 konumuna gidildi.")
        except:
            self.msg_signal.emit(f"ERROR: Asama2 konuma gidilemedi!")

        #roi balon
        try:
            time.sleep(1)
            # 0,0,0 means reset the roi which means follow the direction of travel
            self.mission.set_roi(lat=self.konumlar["asama2_balon"]["lat"],
                                 lon=self.konumlar["asama2_balon"]["lon"],
                                 alt=self.konumlar["asama2_balon"]["alt"])
            time.sleep(2)
            self.msg_signal.emit(f"Asama2 balon ROI gerceklestirildi.")
        except:
            self.msg_signal.emit(f"ERROR: Asama2 ROI gerceklestirilemiyor!")

        """#gimbal reset
        try:
            self.gimbal_yaw = 90
            self.gimbal_pitch = 90
            msg_yaw = f"YAW{self.gimbal_yaw}"
            msg_pitch = f"PCH{self.gimbal_pitch}"
            #self.port.write(bytes(msg_yaw, "utf-8"))
            #self.port.write(bytes(msg_pitch, "utf-8"))
            self.msg_signal.emit(f"Asama2 gimbal sifirlaniyor.")
        except:
            self.msg_signal.emit(f"ERROR: Asama2 gimbal sifirlanmiyor!")"""

        #image processing balon: start
        try:
            #main_loop: start
            start_time = time.time()
            frame_sayac = 0

            while True:
                frame_sayac += 1
                self.frame = cv2.resize(self.frame, None, fx=self.mission.size, fy=self.mission.size)

                hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_frame, self.turuncu2_lr, self.turuncu2_hr)
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    #biggest_contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
                    x, y, w, h = cv2.boundingRect(sorted(contours, key=cv2.contourArea, reverse=True)[0])
                    if (60 < w < 200) and (60 < h < 200):
                        cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                        _yon = self.mission.yon((x, y, w, h))

                        mevcut_heading = self.mission.arac.heading

                        angular_yaw_velocity = 2
                        angular_pitch_velocity = 2

                        if _yon[0] == "m":
                            #print(f"atis izni bekleniyor")
                            self.msg_signal.emit("atis izni bekleniyor!")
                        else:
                            if _yon[-1] == "d":
                                self.mission.condition_yaw(heading=mevcut_heading + angular_yaw_velocity)  # east means plus
                            elif _yon[-1] == "b":
                                self.mission.condition_yaw(heading=mevcut_heading - angular_yaw_velocity)
                            if _yon[0] == "g":
                                #velocity arti asagi hareket
                                self.mission.irtifa_velocity(ay=0.1)
                            elif _yon[0] == "k":
                                #velocity eksi yukari hareket
                                self.mission.irtifa_velocity(ay=-0.1)


                # hedef alani cizdirme
                cv2.rectangle(self.frame, (self.mission.hedef[0], self.mission.hedef[1]),
                              (self.mission.hedef[2], self.mission.hedef[3]),
                              (170, 20, 125), 3)

                # fps
                """end_time = time.time() - start_time
                fps = frame_sayac / end_time
                cv2.putText(self.frame, f"FPS: {round(fps, 2)}", (10, 50), self.mission.font, 2, (255, 255, 255), 2)"""

                #self.kamera_signal.emit(self.frame)
                self.kamera_signal.emit(cv2.bitwise_and(self.frame, self.frame, mask=mask))

                if self.stoper:
                    break
                else:
                    time.sleep(1/60)

            self.msg_signal.emit(f"asama2 basariyla gerceklestirildi")
            # main_loop: end

        except:
            self.msg_signal.emit("ERROR: asama2 gerceklestirilemiyor!")

        self.end_signal.emit(True)

    def stop(self):
        self.stoper = True

    def terminater(self):
        self.calisiyor = False
        self.terminate()