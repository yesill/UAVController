import os, subprocess
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from gui.orion_gui_dialog_asama3 import Ui_Dialog
import json, os
from os.path import dirname


class DialogAsama3(QtWidgets.QDialog, Ui_Dialog):

    start_signal = pyqtSignal(str)

    def __init__(self, mission, ip_port):
        super(DialogAsama3, self).__init__()
        self.setupUi(self)

        # a -> hepsi, r -> kirmizi, b -> movi, g -> yesil
        self.renk = "a"

        self.btn_start.clicked.connect(self.start)
        self.btn_cancel.clicked.connect(self.close)

        self.btn_color_red.clicked.connect(self.color_red)
        self.btn_color_blue.clicked.connect(self.color_blue)
        self.btn_color_green.clicked.connect(self.color_green)

    def color_red(self):
        self.renk = "r"
        self.btn_color_red.setEnabled(False)
        self.btn_color_blue.setEnabled(True)
        self.btn_color_green.setEnabled(True)

    def color_blue(self):
        self.renk = "b"
        self.btn_color_red.setEnabled(True)
        self.btn_color_blue.setEnabled(False)
        self.btn_color_green.setEnabled(True)

    def color_green(self):
        self.renk = "g"
        self.btn_color_red.setEnabled(True)
        self.btn_color_blue.setEnabled(True)
        self.btn_color_green.setEnabled(False)

    def start(self):
        self.close()
        temp = dict()
        with open(f"{dirname(dirname(__file__))}/konumlar.json", "r") as file:
            temp = json.load(file)
            temp["asama3_balon"]["renk"] = self.renk

        with open(f"{dirname(dirname(__file__))}/konumlar.json", "w") as file:
            json.dump(temp, file, indent=4)

        self.start_signal.emit(self.renk)
