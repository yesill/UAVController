import cv2, time, json
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


class Mission():

    def __init__(self):
        self.arac = None

        # modelin yüklenmesi ve hazırlığı
        self.cap = cv2.VideoCapture(0)
        self.net = cv2.dnn.readNet('../yolo_custom_detection/yolov3_balloons.weights',
                                   '../yolo_custom_detection/yolov3_testing_balloons.cfg')

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        self.kirmizi_sinirlar = ((17, 20, 46), (84, 82, 166))  # BGR!!!
        self.yesil_sinirlar = ((84, 83, 62), (185, 183, 148))  # BGR!!!
        self.mavi_sinirlar = ((0, 0, 0), (0, 0, 0,))  # BGR!!

        self.size = 1
        _, self.img = self.cap.read()
        self.cap.release()
        self.img = cv2.resize(self.img, None, fx=self.size, fy=self.size)
        self.height, self.width, self.channels = self.img.shape  #640,480 if size == 1
        self.oran = 0.1
        #hedef alaninin kenar uzunluklarının yaty ve dikeyle oranı
        #self.hedef = (int(self.width * (1-self.oran)//2), int(self.height * (1-self.oran)//2), int(self.width*self.oran), int(self.height*self.oran))
        self.hedef = (305, 159, 330, 184)
        """self.izgara = [((0, 0), (self.hedef[0], self.hedef[1])),                                                    #kuzey-batı bolgesi
                  ((self.hedef[0], 0), (self.hedef[0]+self.hedef[2], self.hedef[1])),                               #kuzey bölgesi
                  ((self.hedef[0]+self.hedef[2], 0), (self.width, self.hedef[1])),                                  #kuzey-doğu bölgesi
                  ((0, self.hedef[1]), (self.hedef[0], self.hedef[1]+self.hedef[3])),                               #batı bölgesi
                  ((self.hedef[0], self.hedef[1]), (self.hedef[0]+self.hedef[2], self.hedef[1]+self.hedef[3])),     #merkez bölge
                  ((self.hedef[0]+self.hedef[2], self.hedef[1]), (self.width, self.hedef[1]+self.hedef[3])),        #doğu bölgesi
                  ((0, self.hedef[1]+self.hedef[3]), (self.hedef[0], self.height)),                                 #güney-batı bölgesi
                  ((self.hedef[0], self.hedef[1]+self.hedef[3]), (self.hedef[0]+self.hedef[2], self.height)),       #güney bölgesi
                  ((self.hedef[0]+self.hedef[2], self.hedef[1]+self.hedef[3]), (self.width, self.height))]          #güney-doğu bölgesi"""

        self.izgara = [((0, 0), (305, 159)),        #kb
                       ((305, 0), (330, 159)),      #kk
                       ((330, 0), (639, 159)),      #kd
                       ((0, 159), (305, 184)),      #bb
                       ((305, 159), (330, 184)),    #mm
                       ((330, 159), (639, 184)),    #dd
                       ((0, 184), (305, 479)),      #gb
                       ((305, 184), (330, 479)),    #gg
                       ((330, 184), (639, 479))]    #gd

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

    def renk_yogunlugu(self, balon, frame):
        # girilen balonun merkez noktasından yatayda 20%, dikeyde 20% lik bir dikdörtgen belirler.
        # merkeze yakın bu dikdörtgenin içeriisndeki her bir pikselin BGR değerlerini okuyup ort. alır.
        # bulunan ortalama değer belirtilen rengin sınırları içerisinde ise renk baş harfini çıktı verir.
        tx, ty, tw, th = balon
        temp_oran = 0.1
        merkez_nokta = (tx + tw // 2, ty + th // 2)
        box_pt1 = (int(merkez_nokta[0] - tw * temp_oran), int(merkez_nokta[1] - th * temp_oran))
        box_pt2 = (int(merkez_nokta[0] + tw * temp_oran), int(merkez_nokta[1] + th * temp_oran))
        color = [0, 0, 0]  # bgr
        sayac = 0
        for j in range(box_pt1[0], box_pt2[0] - 1):
            for i in range(box_pt1[1], box_pt2[1] - 1):
                temp_color = frame[i, j]
                color[0] += int(temp_color[0])
                color[1] += int(temp_color[1])
                color[2] += int(temp_color[2])
                sayac += 1
        color[0] = color[0] // sayac
        color[1] = color[1] // sayac
        color[2] = color[2] // sayac
        # cv2.rectangle(frame, box_pt1, box_pt2, (0, 0, 0), 2)
        ks = self.kirmizi_sinirlar
        ys = self.yesil_sinirlar
        """ms = mavi_sinirlar"""
        if ks[0][0] <= color[0] <= ks[1][0] and ks[0][1] <= color[1] <= ks[1][1] and ks[0][2] <= color[2] <= ks[1][2]:
            return "r"
        elif ys[0][0] <= color[0] <= ys[1][0] and ys[0][1] <= color[1] <= ys[1][1] and ys[0][2] <= color[2] <= ys[1][2]:
            return f"g"
        """ mavi renk balonum yok o yüzden bu kısım hazır değil
        elif ms[0][0] <= color[0] <= ms[1][0] and ms[0][1] <= color[1] <= ms[1][1] and ms[0][2] <= color[2] <= ms[1][2]:
            return f"b" """
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
            rec_sayac = 0
            while not self.arac.armed:
                if rec_sayac > 10:
                    print("sayac > 10")
                    self.kalkis()
                print ("Arac arm edilemedi, arm icin bekleniliyor..")
                time.sleep(1)
                rec_sayac += 1
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

    def inis(self):
        """
            Irtifa 0.3 metrenin altina indikten 1 saniye sonra indi kabul edilir.
            :param rtl: True -> kalktigin konuma don ve inis yap, False -> oldugun yerde inis yap.
            :return: Inis tamamlandiginda True degeri doner.
        """
        self.arac.mode = VehicleMode("LAND")
        while True:
            #print(f"irtifa: {self.arac.location.global_relative_frame.alt},{self.arac.mode}")
            if self.arac.location.global_relative_frame.alt <= 0.2:
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

    def git(self,enlem,boylam,hiz=0.5,irtifa=None,check_irtifa=False):
        """
            Verilen enlem ve boylama gider.
            Irtifa ve hiz belirtilmesi tavsiye edilir.
            :param enlem: enlem
            :param boylam:  boylam
            :param irtifa: irtifa. Deger belirtilmedigi takdirde mevcut irtifasini korur.
            :param hiz: Deger belirtilmedigi takdirde 0.5m/sn ile yavasca hareket eder.
            :param cehck_irtifa: True -> Belirtilen konumun irtifasini ulisilip ulasilmadigini kontrol et.
            :return: verilen enlem ve boylama ulasilabilir ise True, ulasilamaz ise False degeri doner.
        """
        if irtifa is None:
            """Irtifa girilmedigi takdirde mevcut irtifa korunur."""
            irtifa = self.arac.location.global_relative_frame.alt

        print(f"Git komutu -> enlem: {enlem}, boylam: {boylam}, irtifa: {irtifa}")

        self.arac.simple_goto(location=LocationGlobalRelative(enlem,boylam,irtifa), groundspeed=hiz)
        kontrol_irtifa = None
        if check_irtifa:
            kontrol_irtifa = irtifa
            print(f"kontol irtifa: {kontrol_irtifa}")

        while not self.konumKontrol(enlem=enlem,boylam=boylam,irtifa=kontrol_irtifa):
            """if heading_to_travel:
                relative_angle = 0
                #print(f"condition_yaw relative girildi! derece: {relative_angle}")
                #print(f"heading: {arac.heading}")
                #self.condition_yaw(heading=relative_angle,relative=True)"""
            time.sleep(1)
        else:
            return True

    def condition_yaw(self, heading, relative=False):
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

    def irtifa_velocity(self, ig=0, ss=0, ay=0):
        # Zamanlı (x) + ileri - geri ------- (y) + sağ - sol -------- (z) + aşağı - yukarı
        msg = self.arac.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            ig, ss, ay,
            0, 0, 0, 0, 0)
        self.arac.send_mavlink(msg)

    def heading_degistir(self, heading):
        if 0 <= heading <= 359:
            while self.arac.heading != heading:
                self.condition_yaw(heading=0)
                time.sleep(0.2)
            return True
        else:
            return False

    def set_roi(self, lat, lon, alt):
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.arac.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            lat,
            lon,
            alt
        )
        # send command to vehicle
        self.arac.send_mavlink(msg)
    #drone kit: end

    def konumOku(self):
        temp = dict()
        with open("konumlar.json", "r") as file:
            temp = json.load(file)
        return temp

    def konumYaz(self, temp):
        with open("konumlar.json", "w") as file:
            json.dump(temp, file, indent=4, sort_keys=True)
