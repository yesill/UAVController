import sys, cv2
from datetime import datetime
from PyQt5 import QtWidgets
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from gui.orion_main_window import Ui_MainWindow
from gui.orion_dialog_logs import DialogLogs
from gui.orion_dialog_connect import DialogConnect
from gui.orion_dialog_asama3 import DialogAsama3
from subprocesses.orion_kalkis_inis_rtl import ThreadKalkis, ThreadInis, ThreadRTL
from subprocesses.orion_asama2 import ThreadAsama2
from subprocesses.orion_asama3 import ThreadAsama3
from subprocesses.orion_asama4 import ThreadAsama4
from subprocesses.orion_kamera import ThreadKamera
import gorev


yazilim_mesajlari = []
telemetri = []
global_ip_port = None
mission = gorev.Mission()

def log_msg_add(msg:str):
    if msg is not None:
        yazilim_mesajlari.append([str(datetime.now().strftime("%H:%M:%S")),msg])

def update_ip_port(ip_port):
    global global_ip_port
    global_ip_port = str(ip_port)


class orion(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.kamera_no = 0

        self.kameraAsama2Aktif = False
        self.kameraAsama3Aktif = False
        self.kameraAsama4Aktif = False

        self.port_name = "/dev/ttyUSB0"

        #threads
        self.thread_kamera = ThreadKamera(kamera_no=self.kamera_no)
        self.threads = {"kalkis": ThreadKalkis(mission=mission),
                        "inis": ThreadInis(mission=mission),
                        "rtl": ThreadRTL(mission=mission),
                        "asama2": ThreadAsama2(mission=mission, threadKamera=self.thread_kamera, port_name=self.port_name),
                        "asama3": ThreadAsama3(mission=mission, threadKamera=self.thread_kamera, port_name=self.port_name),
                        "asama4": ThreadAsama4(mission=mission, threadKamera=self.thread_kamera, port_name=self.port_name),
                        }

        #thread connects
        self.thread_kamera.frame_signal.connect(self.kamera)

        self.threads["kalkis"].end_signal.connect(self.takeoffStop)
        self.threads["kalkis"].msg_signal.connect(log_msg_add)

        self.threads["inis"].end_signal.connect(self.landStop)
        self.threads["inis"].msg_signal.connect(log_msg_add)

        self.threads["rtl"].end_signal.connect(self.returnToLaunchStop)
        self.threads["rtl"].msg_signal.connect(log_msg_add)

        #asama2
        self.threads["asama2"].end_signal.connect(self.asama2Terminater)
        self.threads["asama2"].kamera_signal.connect(self.kameraAsama2)
        self.threads["asama2"].msg_signal.connect(log_msg_add)

        #asama3
        self.threads["asama3"].end_signal.connect(self.asama3Terminater)
        self.threads["asama3"].kamera_signal.connect(self.kameraAsama3)
        self.threads["asama3"].msg_signal.connect(log_msg_add)

        #asama4
        self.threads["asama4"].end_signal.connect(self.asama4Terminater)
        self.threads["asama4"].kamera_signal.connect(self.kameraAsama4)
        self.threads["asama4"].msg_signal.connect(log_msg_add)

        #timers
        self.timer_message = QTimer(self)
        self.timer_message.timeout.connect(self.updateMessage)

        self.timer_telemetry = QTimer()
        self.timer_telemetry.timeout.connect(self.updateTelemetry)

        #dialog connect
        self.win_dialog_connect = DialogConnect(mission=mission)
        self.win_dialog_connect.start_signal.connect(self.controlTimerMessage)
        self.win_dialog_connect.connect_signal.connect(self.kameraStart)
        self.win_dialog_connect.connect_signal.connect(self.controlTimerTelemetry)
        self.win_dialog_connect.connect_signal.connect(self.connect)
        self.win_dialog_connect.ip_port_signal.connect(update_ip_port)
        self.win_dialog_connect.msg_signal.connect(log_msg_add)

        #dialog asama3
        self.win_dialog_asama3 = DialogAsama3(mission=mission,ip_port=global_ip_port)
        self.win_dialog_asama3.start_signal.connect(self.asama3Start)

        #main window
        self.ui.btn_connect.clicked.connect(self.connect)
        self.ui.btn_logs.clicked.connect(self.logs)
        self.ui.btn_cut.clicked.connect(self.cut)
        self.ui.btn_cut.clicked.connect(self.kameraStop)
        self.ui.btn_asama1.clicked.connect(self.asama1)
        self.ui.btn_asama2.clicked.connect(self.asama2Start)
        self.ui.btn_asama3.clicked.connect(self.asama3)
        self.ui.btn_asama4.clicked.connect(self.asama4Start)
        self.ui.btn_takeoff.clicked.connect(self.takeoffStart)
        self.ui.btn_land.clicked.connect(self.landStart)
        self.ui.btn_rtl.clicked.connect(self.returnToLaunchStart)

    def threadTerminator(self):
        if self.threads["kalkis"].calisiyor:
            print("terminate kalkis")
            self.threads["kalkis"].stop()

        if self.threads["inis"].calisiyor:
            print("terminate inis")
            self.threads["inis"].stop()

        if self.threads["rtl"].calisiyor:
            print("terminate rtl")
            self.threads["rtl"].stop()

        if self.threads["asama2"].calisiyor:
            print("terminate asama2")
            self.threads["asama2"].stop()

        if self.threads["asama3"].calisiyor:
            print("terminate asama3")
            self.threads["asama3"].stop()

        if self.threads["asama4"].calisiyor:
            print("terminate asama4")
            self.threads["asama4"].stop()

    def logs(self):
        win_dialog_logs = DialogLogs(yazilim_mesajlari=yazilim_mesajlari)
        win_dialog_logs.exec()

    def connect(self):
        self.win_dialog_connect.exec()

    def cut(self):
        self.kameraStop()
        self.threadTerminator()
        self.ui.label_cam.setText("Kamera")
        self.ui.btn_connect.setText("BAĞLAN")
        self.ui.btn_connect.setEnabled(True)
        try:
            #arac baglimizi kontrol ediyoruz
            if str(type(mission.arac)) == "<class 'dronekit.Vehicle'>":
                mission.baglantiyi_kes()
                #telemetry: start
                self.timer_telemetry.stop()
                self.ui.label_lat.setText(f"Enlem: -")
                self.ui.label_lon.setText(f"Boylam: -")
                self.ui.label_alt.setText(f"Irtifa: -")
                self.ui.label_spd.setText(f"Hız: -")
                self.ui.label_btr.setText(f"Batarya: -")
                self.ui.label_hdg.setText(f"Heading: -")
                self.ui.label_vmd.setText(f"Mod: -")
                self.ui.label_armed.setText(f"Disarmed")
                #telemetry: end
                log_msg_add("baglanti kesildi")
            else:
                log_msg_add("ERROR: baglanti bulunamadi")

            #saves buğra
            with open("son_ucus_kayitlari.txt","w") as file:
                for t, msg in yazilim_mesajlari:
                    file.writelines(f"{t} -> {msg}\n")

            with open("son_ucus_telemetri.txt", "w") as file:
                for t, telem in telemetri:
                    file.writelines(f"{t} -> {telem}\n")

        except:
            log_msg_add("ERROR: baglanti kesilemedi!")

    def updateMessage(self):
        if len(yazilim_mesajlari) >= 1:
            self.ui.label_msg.setText(f"{yazilim_mesajlari[-1][1]}")

    def controlTimerMessage(self):
        self.timer_message.start(1000)
        self.ui.btn_connect.setText("Bagli")
        self.ui.btn_connect.setEnabled(False)

    def updateTelemetry(self):
        lat = str(mission.arac.location.global_relative_frame.lat)
        lon = str(mission.arac.location.global_relative_frame.lon)
        alt = str(mission.arac.location.global_relative_frame.alt)
        spd = "{:.3f}".format(mission.arac.groundspeed)
        btr = str(mission.arac.battery.level)
        hea = str(mission.arac.heading)
        mod = str(str(mission.arac.mode).split(':')[1])
        arm = "Armed" if mission.arac.armed else "Disarmed"

        self.ui.label_lat.setText(f"Enlem: {lat}")
        self.ui.label_lon.setText(f"Boylam: {lon}")
        self.ui.label_alt.setText(f"Irtifa: {alt} m")
        self.ui.label_spd.setText(f"Hız: {spd} m/sn")
        self.ui.label_btr.setText(f"Batarya: %{btr}")
        self.ui.label_hdg.setText(f"Heading: {hea}")
        self.ui.label_vmd.setText(f"Mod: {mod}")
        self.ui.label_armed.setText(arm)

        telemetri.append([str(datetime.now().strftime("%H:%M:%S")),
                          f"lat:{lat}, lon:{lon}, alt:{alt}, spd:{spd}, btr:{btr}, hea:{hea}, mod:{mod}, arm:{arm}"])

    def controlTimerTelemetry(self):
        self.timer_telemetry.start(500)

    def kamera(self, frame):
        if not self.kameraAsama2Aktif and not self.kameraAsama3Aktif and not self.kameraAsama4Aktif:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #yatayda ayna yansimasi aliyor
            frame = cv2.flip(frame, 1)
            # hedef alani cizdirme
            cv2.rectangle(frame, (mission.hedef[0], mission.hedef[1]),
                          (mission.hedef[2], mission.hedef[3]),
                          (170, 20, 125), 3)
            height, width, channel = frame.shape
            step = channel * width
            qImg = QImage(frame.data, width, height, step, QImage.Format_RGB888)
            self.ui.label_cam.setPixmap(QPixmap.fromImage(qImg))

    def kameraStart(self):
        self.thread_kamera.start()

    def kameraStop(self):
        self.thread_kamera.stop()

    #asama1
    def asama1(self):
        log_msg_add("asama1 baslatildi")
        self.threadTerminator()

    #asama2
    def asama2Start(self):
        self.threadTerminator()
        self.threads["asama2"].start()

    def asama2Terminater(self):
        self.threads["asama2"].terminater()
        self.kameraAsama2Aktif = False

    def kameraAsama2(self, frame):
        self.kameraAsama2Aktif = True
        #self.kamera(frame=frame)
        temp_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = temp_frame.shape
        step = channel * width
        qImg = QImage(temp_frame.data, width, height, step, QImage.Format_RGB888)
        self.ui.label_cam.setPixmap(QPixmap.fromImage(qImg))

    #asama3
    def asama3(self):
        self.win_dialog_asama3.exec()

    def asama3Start(self, renk):
        self.threads["asama3"].start()
        print("ThreadTerminator Calistirildi")
        self.threadTerminator()

    def asama3Terminater(self):
        self.threads["asama3"].terminater()
        self.kameraAsama3Aktif = False

    def kameraAsama3(self, frame):
        self.kameraAsama3Aktif = True
        temp_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = temp_frame.shape
        step = channel * width
        qImg = QImage(temp_frame.data, width, height, step, QImage.Format_RGB888)
        self.ui.label_cam.setPixmap(QPixmap.fromImage(qImg))

    #asama 4
    def asama4Start(self):
        self.threadTerminator()
        self.threads["asama4"].start()
        print("ThreadTerminator Calistirildi")
        self.threadTerminator()

    def asama4Terminater(self):
        self.threads["asama4"].terminater()
        self.kameraAsama4Aktif = False

    def kameraAsama4(self, frame):
        self.kameraAsama4Aktif = True
        temp_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = temp_frame.shape
        step = channel * width
        qImg = QImage(temp_frame.data, width, height, step, QImage.Format_RGB888)
        self.ui.label_cam.setPixmap(QPixmap.fromImage(qImg))

    #thread functions: start
    def takeoffStart(self):
        self.threads["kalkis"].start()
        print("ThreadTerminator Calistirildi")
        self.threadTerminator()

    def takeoffStop(self):
        self.threads["kalkis"].stop()

    def landStart(self):
        self.threads["inis"].start()
        print("ThreadTerminator Calistirildi")
        self.threadTerminator()

    def landStop(self):
        self.threads["inis"].stop()

    def returnToLaunchStart(self):
        self.threads["rtl"].start()
        print("ThreadTerminator Calistirildi")
        self.threadTerminator()

    def returnToLaunchStop(self):
        self.threads["rtl"].stop()
    #thread functions: end

    def atisIzni(self):
        log_msg_add("atis izni fonksiyonuna girildi")
        pass


app = QtWidgets.QApplication(sys.argv)
win = orion()
win.show()
sys.exit(app.exec())