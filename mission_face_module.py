import cv2, time
import konumlar
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


class MissionFace():

    def __init__(self):
        self.arac = None

        # modelin yüklenmesi ve hazırlığı
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.cap = cv2.VideoCapture(0)

        self.size = 1
        self.sil, self.img = self.cap.read()
        self.cap.release()
        self.img = cv2.resize(self.img, None, fx=self.size, fy=self.size)
        self.height, self.width, self.channels = self.img.shape  #640,480 if size == 1
        self.oran = 0.1
        #hedef alaninin kenar uzunluklarının yaty ve dikeyle oranı
        self.hedef = (int(self.width * (1-self.oran)//2), int(self.height * (1-self.oran)//2), int(self.width*self.oran), int(self.height*self.oran))
        self.izgara = [((0, 0), (self.hedef[0], self.hedef[1])),                                                    #kuzey-batı bolgesi
                  ((self.hedef[0], 0), (self.hedef[0]+self.hedef[2], self.hedef[1])),                               #kuzey bölgesi
                  ((self.hedef[0]+self.hedef[2], 0), (self.width, self.hedef[1])),                                  #kuzey-doğu bölgesi
                  ((0, self.hedef[1]), (self.hedef[0], self.hedef[1]+self.hedef[3])),                               #batı bölgesi
                  ((self.hedef[0], self.hedef[1]), (self.hedef[0]+self.hedef[2], self.hedef[1]+self.hedef[3])),     #merkez bölge
                  ((self.hedef[0]+self.hedef[2], self.hedef[1]), (self.width, self.hedef[1]+self.hedef[3])),        #doğu bölgesi
                  ((0, self.hedef[1]+self.hedef[3]), (self.hedef[0], self.height)),                                 #güney-batı bölgesi
                  ((self.hedef[0], self.hedef[1]+self.hedef[3]), (self.hedef[0]+self.hedef[2], self.height)),       #güney bölgesi
                  ((self.hedef[0]+self.hedef[2], self.hedef[1]+self.hedef[3]), (self.width, self.height))]          #güney-doğu bölgesi
        self.starting_time = time.time()
        self.font = cv2.FONT_HERSHEY_PLAIN

    #image processing: start
    def icinde_mi(self,merkez_nokta, kose_nokta1, kose_nokta2):
        # girilen merkez nokta, sol-üst ve sağ-alt noktasi verilen dikdörtgenin içerisinde mi kontrol eden fonksiyon
        if kose_nokta1[0] <= merkez_nokta[0] <= kose_nokta2[0] and kose_nokta1[1] <= merkez_nokta[1] <= kose_nokta2[1]:
            return True
        else:
            return False

    def merkezler_arasi_mesafe(self,balon):
        # girilen balon dikdörtgeninin merkezi ile hedef alanı merkezi arasındaki mesafeyi veren fonksiyon
        x1, y1, w1, h1 = balon
        x2, y2, w2, h2 = self.hedef
        return int(
            (abs((x1 + (w1 // 2)) - (x2 + (w2 // 2))) ** 2 + abs((y1 + (h1 // 2)) - (y2 + (h2 // 2))) ** 2) ** (1 / 2))

    def yon(self,balon):
        # verilen balonun izgara içerisinde ki hangi bölgede olduğu bilgisini veren fonksiyon
        x1, y1, w1, h1 = balon
        balon_merkez_nokta = (x1+(w1//2), y1+(h1//2))

        if self.icinde_mi(balon_merkez_nokta, self.izgara[0][0], self.izgara[0][1]):
            return f"kb"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[1][0], self.izgara[1][1]):
            return f"kk"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[2][0], self.izgara[2][1]):
            return f"kd"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[3][0], self.izgara[3][1]):
            return f"bb"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[4][0], self.izgara[4][1]):
            return f"mm"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[5][0], self.izgara[5][1]):
            return f"dd"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[6][0], self.izgara[6][1]):
            return f"gb"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[7][0], self.izgara[7][1]):
            return f"gg"
        elif self.icinde_mi(balon_merkez_nokta, self.izgara[8][0], self.izgara[8][1]):
            return f"gd"
    #image processing: end

    #drone kit: start
    def baglan(self,ip_port):
        try:
            self.arac = connect(f"udp:{ip_port}", baud=115200, wait_ready=True)
            print("baglanti kuruldu")

        except:
            print("baglanti kurulamadi")

    def baglantiyi_kes(self):
        self.arac.close()
        self.arac = None

    def kalkis(self,irtifa=3):
        """
            Arac once arm edilir. Arm edildikten sonra verilen irtifaya yukselir.
            :param irtifa:  Verilen irtifaya kadar yukselir, irtifa girilmemis ise 1 metre yukselir.
            :return: Kalkis tamamlandiginda True degeri doner.
        """
        if self.arac.location.global_relative_frame.alt < 1 or not self.arac.armed:
            print ("Arm edilebilirlik kontrol ediliyor")
            while not self.arac.is_armable:
                print ("Arac arm edilebilir durumda degil..")
                time.sleep(1)
            print ("Arac arm ediliyor..")
            self.arac.mode = VehicleMode("GUIDED")
            self.arac.armed = True
            while not self.arac.armed:
                print ("Arac arm edilemedi, arm icin bekleniliyor..")
                time.sleep(1)
            print ("Arac arm edildi, kalkis yapiliyor!")
            self.arac.simple_takeoff(irtifa)
            while True:
                print (" irtifa:", self.arac.location.global_relative_frame.alt," metre")
                if self.arac.location.global_relative_frame.alt >= irtifa * 0.95:
                    #print ("Hedef irtifaya ulasildi")
                    break
                time.sleep(1)
            return True
        else:
            return True

    def inis(self,rtl=False):
        """
            Irtifa 0.3 metrenin altina indikten 1 saniye sonra indi kabul edilir.
            :param rtl: True -> kalktigin konuma don ve inis yap, False -> oldugun yerde inis yap.
            :return: Inis tamamlandiginda True degeri doner.
        """
        self.arac.mode = VehicleMode("RTL" if rtl else "LAND")
        while True:
            print(f"irtifa: {self.arac.location.global_relative_frame.alt},{self.arac.mode}")
            if self.arac.location.global_relative_frame.alt <= 0.3:
                time.sleep(1)   #indiginden emin olmak icin
                break
            time.sleep(2)
        return True

    def konumKontrol(self,enlem,boylam,irtifa=None):
        """
            Aracin mevcut enleminin ve boylaminin, aracin olması istenilen enleminden ve boylamindan farki
            hata payi icerisinde ise konuma ulasmis kabul edilir.
            :param hata_payi: Varsayilan olarak 0.0000010 kabul edilir.
            :param irtifa: Irtifada istenildigi takdirde hata payina gore kontrol edilebilir.
            :return: Konuma ulasildi kabul ediliyor ise True, konuma ulasilamadi kabul ediliyor ise False degeri doner!
        """
        hata_payi_enlem_boylam = 0.0000010
        hata_payi_irtifa = 0.20
        lat = self.arac.location.global_relative_frame.lat
        lon = self.arac.location.global_relative_frame.lon
        alt = self.arac.location.global_relative_frame.alt
        if irtifa is not None:
            #irtifa = (float(irtifa) if type(irtifa) == "float" else irtifa)
            irtifa = float(irtifa)
            if abs(enlem-lat) < enlem*hata_payi_enlem_boylam and abs(boylam-lon) < boylam*hata_payi_enlem_boylam and abs(irtifa-alt) < irtifa-hata_payi_irtifa:
                return True
            else:
                return False
        else:
            if abs(enlem-lat) < enlem*hata_payi_enlem_boylam and abs(boylam-lon) < boylam*hata_payi_enlem_boylam:
                return True
            else:
                return False

    def git(self,enlem,boylam,hiz=0.5,irtifa=None,heading_to_travel=True,check_irtifa=False):
        """
            Verilen enlem ve boylama gider.
            Irtifa ve hiz belirtilmesi tavsiye edilir.
            :param enlem: enlem
            :param boylam:  boylam
            :param irtifa: irtifa. Deger belirtilmedigi takdirde mevcut irtifasini korur.
            :param hiz: Deger belirtilmedigi takdirde 0.5m/sn ile yavasca hareket eder.
            :param heading_to_travel: True -> Heading hareket yonune esit, False -> Mevcut Heading korunur.
            :param cehck_irtifa: True -> Belirtilen konumun irtifasini ulisilip ulasilmadigini kontrol et.
            :return: verilen enlem ve boylama ulasilabilir ise True, ulasilamaz ise False degeri doner.
        """
        if irtifa is None:
            """Irtifa girilmedigi takdirde mevcut irtifa korunur."""
            irtifa = self.arac.location.global_relative_frame.alt

        print(f"Git komutu -> enlem: {enlem}, boylam: {boylam}, irtifa: {irtifa}, hiz: {hiz}")

        self.arac.simple_goto(location=LocationGlobalRelative(enlem,boylam,irtifa), groundspeed=hiz)
        kontrol_irtifa = None
        if check_irtifa:
            kontrol_irtifa = irtifa
            print(f"kontol irtifa: {kontrol_irtifa}")

        while not self.konumKontrol(enlem=enlem,boylam=boylam,irtifa=kontrol_irtifa):
            if heading_to_travel:
                relative_angle = 0
                #print(f"condition_yaw relative girildi! derece: {relative_angle}")
                #print(f"heading: {arac.heading}")
                self.condition_yaw(heading=relative_angle,relative=True)
            time.sleep(1)
        else:
            return True

    def return_to_launch(self):
        self.git(enlem=konumlar.home["lat"],
                 boylam=konumlar.home["lon"],
                 hiz=konumlar.home["spd"],
                 irtifa=konumlar.home["alt"],
                 heading_to_travel=True)
        return True

    def irtifa_fonk(self,irtifa, abs=True, hiz=0.5):
        """
        Yukselme fonksiyonu.
        Belirli bir yukseklige ulasmak icin abs=True, Belirtilen irtifa kadar yukselmek icin abs=False.
        :param irtifa:
        :param abs: True -> Belirtilen irtifaya yuksel, False -> Belirtilen irtifa kadar yuksel
        :return: True -> Belirtilen irtifaya gore hareket tamamlandi.
        """
        return self.git(enlem=self.arac.location.global_relative_frame.lat, boylam=self.arac.location.global_relative_frame.lon,
                       hiz=hiz, irtifa=irtifa, check_irtifa=True)

    def condition_yaw(self,heading, relative=False):
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.arac.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            1 if relative else 0, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.arac.send_mavlink(msg)

    def heading_degistir(self,heading):
        if 0 <= heading <= 359:
            while self.arac.heading != heading:
                self.condition_yaw(heading=0)
                time.sleep(0.2)
            return True
        else:
            return False
    #drone kit: end

    #image process action: start
    def main_loop(self, renk=False):
        """
            goruntu isleme ve atis islemlerinin gerceklestirildigi fonksiyon
            :param renk: asama iki icin renk secimi. True -> renk secimi yapilacak, False -> renk secimi yapilmayacak.
                         varsayilan olarak False kabul ediliyor.
            :return: Deger dondurmuyor.
        """
        # ana fonksiyon
        self.cap = None
        self.cap = cv2.VideoCapture(0)
        frame_counter = 0
        while self.cap.isOpened():
            frame_counter += 1
            _, img = self.cap.read()
            img = cv2.resize(img, None, fx=self.size, fy=self.size)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # nesne tanima
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
            if len(faces) > 0:
                for (x, y, w, h) in faces:
                    roi_gray = gray[y:y + h, x:x + w]
                    color = (0, 255, 0)  # BGR
                    stroke = 2

                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(img, ((x + (w // 2)), (y + (h // 2))), 10, (0, 250, 0), cv2.FILLED)
                    #rotating the camera: start
                    _yon = self.yon([x,y,w,h])
                    if _yon != "mm":
                        print(_yon)

                    mevcut_heading = self.arac.heading
                    angular_yaw_velocity = 5
                    angular_pitch_velocity = 2

                    if _yon[-1] == "d":
                        #arac.gimbal.rotate(pitch=0,roll=0,yaw=10)
                        self.condition_yaw(heading=mevcut_heading + angular_yaw_velocity) #east means plus
                    elif _yon[-1] == "b":
                        if mevcut_heading < angular_yaw_velocity:
                            mevcut_heading = 359
                        self.condition_yaw(heading=mevcut_heading - angular_yaw_velocity)

                        #arac.gimbal.rotate(pitch=0,roll=0,yaw=10)
                    elif _yon[-1] == "k":
                        #pitch up
                        #arac.gimbal.rotate(pitch=10,roll=0,yaw=0)
                        #pitch 0 ve hala k ise arac yukselmeli
                        pass
                    elif _yon[-1] == "g":
                        #pitch down
                        #arac.gimbal.rotate(pitch=-10,roll=0,yaw=0)pain
                        #pitch -90 ve hala g ise arac geriye gitmeli!
                        pass
                    elif _yon[-1] == "m":
                        #atis: start
                        if cv2.waitKey(1) & 0xFF == ord('a'):
                            print("atis izni verildi!")
                            for _ in range(3):
                                print("atis yapiliyor!!!")
                                time.sleep(1)
                        else:
                            print("atis izni bekleniyor!")
                        #atis izni: end
                    else:
                        print("error!")
                    #rotating the camera: end

            #hedef alani cizdirme
            cv2.rectangle(img, (self.hedef[0], self.hedef[1]),
                          (self.hedef[0] + self.hedef[2], self.hedef[1] + self.hedef[3]),
                          (170, 20, 125), 3)
            #cv2.putText(img, "Hedef Alani", (hedef[0]+5, hedef[1]+20), font, 1, (170, 20, 125), 2)

            #fps
            elapsed_time = time.time() - self.starting_time
            fps = frame_counter / elapsed_time
            #cv2.putText(img, "FPS: " + str(round(fps, 2)), (10, 50), font, 2, (0, 0, 0), 2)

            #görüntüyü ekrana bastirma
            cv2.imshow("iha goruntu", img)

            # komutlar (renk degistirme, kapatma, vb.)
            komut = cv2.waitKey(20)
            if komut & 0xFF == ord('q'):
                print(f"akış sonlandırıldı..!")
                break

        self.cap.release()
        cv2.destroyAllWindows()
    #image process action: end

    #scenarios: start
    def asama2(self,enlem,boylam,hiz=0.5,irtifa=2):
        #arac kalkis yapmamis ise kalkis yap
        if self.arac.location.global_relative_frame.alt < 1 and not self.arac.armed:
            self.kalkis(irtifa=irtifa)
            print(f"kalkis basarili")
        time.sleep(1)
        self.git(enlem=enlem,boylam=boylam,hiz=hiz,irtifa=irtifa,heading_to_travel=True)
        self.main_loop()
        #baslangic irtifasina geri don
        self.irtifa_fonk(irtifa=irtifa,hiz=1)
        return True

    def asama3(self, renk=0):
        """
        gorev asama 3 fonksiyonu
        :param enlem:
        :param boylam:
        :param hiz:
        :param irtifa:
        :param renk: Patlatimasi istenilen balonlarin rengi
        :return: True -> gorev basariyla tamamlandi! False -> gorev basarisiz
        """
        try:
            #arac kalkis yapmamis ise kalkis yap
            if self.arac.location.global_relative_frame.alt < 1 and not self.arac.armed:
                self.kalkis(irtifa=konumlar.asama3["alt"])
            time.sleep(1)
            self.git(enlem=konumlar.asama3["lat"],boylam=konumlar.asama3["lon"],
                     hiz=konumlar.asama3["spd"],irtifa=konumlar.asama3["alt"],
                     heading_to_travel=True)
            self.main_loop(renk=True)
            return True
        except:
            #gorev basarisiz
            return False

    def asama4(self,enlem,boylam,hiz=0.5,irtifa=2):
        try:
            #arac kalkis yapmamis ise kalkis yap
            if self.arac.location.global_relative_frame.alt > 1 or self.arac.armed:
                self.kalkis(irtifa=irtifa)
                #print(f"kalkis basarili")
            time.sleep(1)
            self.git(enlem=enlem,boylam=boylam,hiz=hiz,irtifa=irtifa,heading_to_travel=True)
            #heading to roi: start

            #heading to roi: end
            return True
        except:
            #gorev basarisiz
            return False
    #scenarios: end
