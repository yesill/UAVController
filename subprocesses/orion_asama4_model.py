#orion asama 4
import cv2, numpy, time
from PyQt5.QtCore import QThread, pyqtSignal


#orion asama 4
class ThreadAsama4(QThread):
    end_signal = pyqtSignal(bool)
    msg_signal = pyqtSignal(str)
    kamera_signal = pyqtSignal(numpy.ndarray)
    atis_izni_signal = pyqtSignal(bool)

    def __init__(self, mission, threadKamera):
        super(ThreadAsama4, self).__init__()
        self.konumlar = mission.konumOku()
        self.mission = mission
        self.threadKamera = threadKamera
        self.frame = None
        self.calisiyor = False
        self.stoper = False
        threadKamera.frame_signal.connect(self.update_frame)

    def update_frame(self, frame):
        temp_frame = frame
        #temp_frame = cv2.cvtColor(temp_frame, cv2.COLOR_BGR2RGB)
        temp_frame = cv2.flip(temp_frame, 1)
        self.frame = temp_frame

    def run(self):
        self.calisiyor = True
        self.stoper = False

        enlem = self.konumlar["asama4_start"]["lat"]
        boylam = self.konumlar["asama4_start"]["lon"]
        irtifa = self.konumlar["asama4_start"]["alt"]
        hiz = self.konumlar["asama4_start"]["spd"]
        renk = "a"

        self.msg_signal.emit(f"Asama4 baslatildi.")

        #kalkis
        try:
            #arac kalkis yapmamis ise kalkis yap
            if self.mission.arac.location.global_relative_frame.alt < 2 and not self.mission.arac.armed:
                self.msg_signal.emit(f"kalkis yapiliyor.")
                self.mission.kalkis(irtifa=self.konumlar["asama4_start"]["alt"])
                self.msg_signal.emit(f"kalkis basarili. irtifa: {self.mission.arac.location.global_relative_frame.alt}")
            time.sleep(1)
        except:
            self.msg_signal.emit(f"ERROR: Asama4 kalkis yapilamadi!")

        #roi-1
        try:
            # 0,0,0 means reset the roi which means follow the direction of travale
            self.mission.set_roi(lat=enlem,
                                 lon=boylam,
                                 alt=irtifa)
            self.msg_signal.emit(f"Asama4 ROI gerceklestirildi.")
        except:
            self.msg_signal.emit(f"ERROR: Asama4 ROI gerceklestirilemiyor!")

        #gorev konumu
        try:
            self.msg_signal.emit(f"Asama4 konumuna ({enlem}, {boylam}, {irtifa}m) gidiliyor.")
            self.mission.git(enlem=enlem,
                             boylam=boylam,
                             hiz=hiz,
                             irtifa=irtifa,
                             heading_to_travel=True)
            self.msg_signal.emit(f"Asama4 konumuna gidildi")
        except:
            self.msg_signal.emit(f"ERROR: Asama4 konuma gidilemedi!")

        #roi-2
        try:
            # 0,0,0 means reset the roi which means follow the direction of travale
            self.mission.set_roi(lat=self.konumlar["asama4_balon"]["lat"],
                                 lon=self.konumlar["asama4_balon"]["lon"],
                                 alt=self.konumlar["asama4_balon"]["alt"])
            self.msg_signal.emit(f"Asama4 ROI gerceklestirildi.")
        except:
            self.msg_signal.emit(f"ERROR: Asama4 ROI gerceklestirilemiyor!")

        #image processing balon: start
        try:
            start_time = time.time()
            frame_sayac = 0

            while True:
                frame_sayac += 1
                self.frame = cv2.resize(self.frame, None, fx=self.mission.size, fy=self.mission.size)
                blob = cv2.dnn.blobFromImage(self.frame, 0.00392, (256, 256), (0, 0, 0), True, crop=False)
                self.mission.net.setInput(blob)
                outs = self.mission.net.forward(self.mission.output_layers)

                # nesne tanima: start
                confidences = []
                boxes = []
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        confidence = scores[numpy.argmax(scores)]
                        if confidence > 0.5:
                            center_x = int(detection[0] * self.mission.width)
                            center_y = int(detection[1] * self.mission.height)
                            w = int(detection[2] * self.mission.width)
                            h = int(detection[3] * self.mission.height)

                            # rectangle coordinates
                            x = int(center_x - w / 2)
                            y = int(center_y - h / 2)

                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))

                # aynı balonu birden fazla kez tanımasını önlemek için indexes
                indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

                # en yakın balonun: (hedef merkeze uzaklığı, boxes listesi içerisindeki index numarası)
                en_yakin = (9999, 0)
                # istenilen renkten bir balon dahi yoksa istenilmeyen renktende olsa bir balon seçiyor bunu engellemek için
                istenilen_renkten_en_az_bir_balon_var = False
                for i in range(len(boxes)):
                    # aynı balonu birden fazla kez tanımasını önlemek için indexes
                    if i in indexes:
                        x, y, w, h = boxes[i]
                        label = f"{object} ({round(confidences[i] * 100)}%)"

                        # renk kontrolü
                        if renk == self.mission.renk_yogunlugu(balon=boxes[i], frame=self.frame) or renk == "a":
                            istenilen_renkten_en_az_bir_balon_var = True
                            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.putText(self.frame, label, (x, y + 30), self.mission.font, 2, (0, 255, 0), thickness=3)

                            # merkeze en yakın balonu bulmak için mesafeleri karşılaştırıyoruz
                            mesafe = self.mission.merkezler_arasi_mesafe(boxes[i])
                            if mesafe <= en_yakin[0]:
                                en_yakin = (mesafe, i)

                        else:
                            continue

                # en yakin balonun merkezinden, hedef alani merkezine çizgi çizdirme
                if len(boxes) > 0 and istenilen_renkten_en_az_bir_balon_var:
                    x, y, w, h = boxes[en_yakin[1]]
                    cv2.circle(self.frame, ((x + (w // 2)), (y + (h // 2))), 10, (0, 250, 0), cv2.FILLED)
                    cv2.line(self.frame, ((x + w // 2), (y + h // 2)),
                             ((self.mission.hedef[0] + self.mission.hedef[2] // 2),
                              (self.mission.hedef[1] + self.mission.hedef[3] // 2)),
                             (125, 125, 0), 3)

                    # rotating the camera: start
                    _yon = self.mission.yon(boxes[en_yakin[1]])
                    if _yon != "mm":
                        print(_yon)

                    mevcut_heading = self.mission.arac.heading
                    angular_yaw_velocity = 5
                    angular_pitch_velocity = 2

                    if _yon[-1] == "d":
                        # arac.gimbal.rotate(pitch=0,roll=0,yaw=10)
                        self.mission.condition_yaw(heading=mevcut_heading + angular_yaw_velocity)  # east means plus
                    elif _yon[-1] == "b":
                        if mevcut_heading < angular_yaw_velocity:
                            mevcut_heading = 359
                        self.mission.condition_yaw(heading=mevcut_heading - angular_yaw_velocity)

                        # arac.gimbal.rotate(pitch=0,roll=0,yaw=10)
                    elif _yon[-1] == "k":
                        # pitch up
                        # arac.gimbal.rotate(pitch=10,roll=0,yaw=0)
                        # pitch 0 ve hala k ise arac yukselmeli
                        pass
                    elif _yon[-1] == "g":
                        # pitch down
                        # arac.gimbal.rotate(pitch=-10,roll=0,yaw=0)pain
                        # pitch -90 ve hala g ise arac geriye gitmeli!
                        pass
                    elif _yon[-1] == "m":
                        # atis: start
                        print("atis izni bekleniyor!")
                        # atis izni: end
                    else:
                        print("error!")
                    # rotating the camera: end

                # ekranda hic balon yok ise
                elif len(boxes) <= 0:
                    print(f"görüş alanı içerisinde balon yok!")

                # nesne tanima: end

                # hedef alani cizdirme
                cv2.rectangle(self.frame, (self.mission.hedef[0], self.mission.hedef[1]),
                              (self.mission.hedef[0] + self.mission.hedef[2],
                               self.mission.hedef[1] + self.mission.hedef[3]),
                              (170, 20, 125), 3)

                # fps
                end_time = time.time() - start_time
                fps = frame_sayac / end_time
                cv2.putText(self.frame, f"FPS: {round(fps,2)}", (10,50), self.mission.font, 2, (0,0,0), 2)

                self.kamera_signal.emit(self.frame)

                if self.stoper:
                    break

            self.msg_signal.emit("asama4 gerceklestirildi")
            #main_loop: end

        except:
            self.msg_signal.emit("ERROR: asama4 gerceklestirilemiyor!")

        self.end_signal.emit(True)

    def stop(self):
        self.stoper = True

    def terminater(self):
        self.calisiyor = False
        self.terminate()
