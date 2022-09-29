from PyQt5.QtCore import QThread, pyqtSignal


class ThreadKalkis(QThread):
    end_signal = pyqtSignal(bool)
    msg_signal = pyqtSignal(str)

    def __init__(self, mission):
        super(ThreadKalkis, self).__init__()
        self.mission = mission
        self.calisiyor = False

    def run(self):
        self.calisiyor = True

        self.msg_signal.emit(f"kalkis yapiliyor.")

        try:
            self.mission.kalkis()
            self.msg_signal.emit(f"Kalkis basarili. Irtifa: {self.mission.arac.location.global_relative_frame.alt}")
        except:
            self.msg_signal.emit("ERROR: kalkis esnasinda bir hata meydana geldi")
        self.end_signal.emit(True)
        del self.mission

    def stop(self):
        self.calisiyor = False
        self.terminate()

class ThreadInis(QThread):
    end_signal = pyqtSignal(bool)
    msg_signal = pyqtSignal(str)

    def __init__(self, mission):
        super(ThreadInis, self).__init__()
        self.mission = mission
        self.calisiyor = False

    def run(self):
        self.calisiyor = True

        self.msg_signal.emit(f"inis yapiliyor")

        try:
            self.mission.inis()
            self.msg_signal.emit(f"Inis yapildi")
        except:
            self.msg_signal.emit("ERROR: Inis esnasinda bir hata meydana geldi")
        self.end_signal.emit(True)
        del self.mission

    def stop(self):
        self.calisiyor = False
        self.terminate()

class ThreadRTL(QThread):
    end_signal = pyqtSignal(bool)
    msg_signal = pyqtSignal(str)

    def __init__(self, mission):
        super(ThreadRTL, self).__init__()
        self.konumlar = mission.konumOku()
        self.mission = mission
        self.calisiyor = False

    def run(self):
        self.calisiyor = True

        enlem = self.konumlar["home"]["lat"]
        boylam = self.konumlar["home"]["lon"]
        hiz = self.konumlar["home"]["spd"]
        irtifa = self.konumlar["home"]["alt"]

        self.msg_signal.emit(f"kalkis noktasina donuluyor")

        #roi
        #kalkis konumunda degil ise roi yap. kalkis konumundayken roi yapilinca arac kendi etrafinda donmeye basliyor.
        if not self.mission.konumKontrol(enlem=enlem, boylam=boylam):
            try:
                # 0,0,0 means reset the roi which means follow the direction of travale
                self.mission.set_roi(lat=0, lon=0, alt=0)
                self.msg_signal.emit(f"RTL ROI gerceklestirildi.")
            except:
                self.msg_signal.emit(f"ERROR: Asama4 ROI gerceklestirilemiyor!")

        #main
        try:
            self.msg_signal.emit(f"Kalkis konumuna ({enlem}, {boylam}, {irtifa}m) gidiliyor.")
            self.mission.git(enlem=enlem, boylam=boylam, hiz=hiz, irtifa=irtifa)
            self.msg_signal.emit(f"kalkis noktasina donuldu")
        except:
            self.msg_signal.emit("ERROR: kalkis noktasina donulemedi")
        self.end_signal.emit(True)
        #del self.mission

    def stop(self):
        self.calisiyor = False
        self.terminate()