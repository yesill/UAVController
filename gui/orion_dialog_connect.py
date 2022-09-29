from datetime import datetime
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from gui.orion_gui_dialog_connect import Ui_Baglanti


class DialogConnect(QtWidgets.QDialog, Ui_Baglanti):

    connect_signal = pyqtSignal(bool)
    start_signal = pyqtSignal(bool)
    msg_signal = pyqtSignal(str)
    ip_port_signal = pyqtSignal(str)

    def __init__(self, mission):
        super(DialogConnect, self).__init__()
        #self.ui = Ui_MainWindow()
        self.setupUi(self)

        self.mission = mission

        self.btn_pos.clicked.connect(self.connect)
        self.btn_neg.clicked.connect(self.close)

    def connect(self):
        #if self.exec_() == QtWidgets.QDialog.Accepted:
        self.close()
        self.start_signal.emit(True)
        ip_port = self.edit_ip_port.text()
        try:
            self.mission.baglan(ip_port=ip_port)
            self.connect_signal.emit(True)
            self.ip_port_signal.emit(str(ip_port))
            self.msg_signal.emit(f"Baglanti kuruldu -> {ip_port}")
        except:
            self.msg_signal.emit("ERROR: baglanti kurulamadi")
        #return ip_port