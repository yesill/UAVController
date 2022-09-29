# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'orion_main_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1201, 701)
        MainWindow.setMinimumSize(QtCore.QSize(1201, 701))
        MainWindow.setMaximumSize(QtCore.QSize(1201, 701))
        font = QtGui.QFont()
        font.setPointSize(12)
        MainWindow.setFont(font)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 90, 811, 551))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_cam = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_cam.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cam.setObjectName("label_cam")
        self.verticalLayout.addWidget(self.label_cam)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(130, 10, 691, 61))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_msg = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_msg.setAlignment(QtCore.Qt.AlignCenter)
        self.label_msg.setObjectName("label_msg")
        self.horizontalLayout.addWidget(self.label_msg)
        self.btn_logs = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.btn_logs.setMaximumSize(QtCore.QSize(16777, 16777215))
        self.btn_logs.setObjectName("btn_logs")
        self.horizontalLayout.addWidget(self.btn_logs)
        self.horizontalLayout.setStretch(0, 80)
        self.horizontalLayout.setStretch(1, 20)
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(840, 90, 351, 181))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.label_spd = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_spd.setObjectName("label_spd")
        self.gridLayout.addWidget(self.label_spd, 1, 1, 1, 1)
        self.label_lat = QtWidgets.QLabel(self.gridLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_lat.setFont(font)
        self.label_lat.setObjectName("label_lat")
        self.gridLayout.addWidget(self.label_lat, 0, 0, 1, 1)
        self.label_lon = QtWidgets.QLabel(self.gridLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_lon.setFont(font)
        self.label_lon.setObjectName("label_lon")
        self.gridLayout.addWidget(self.label_lon, 0, 1, 1, 1)
        self.label_alt = QtWidgets.QLabel(self.gridLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_alt.setFont(font)
        self.label_alt.setObjectName("label_alt")
        self.gridLayout.addWidget(self.label_alt, 1, 0, 1, 1)
        self.label_btr = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_btr.setObjectName("label_btr")
        self.gridLayout.addWidget(self.label_btr, 2, 0, 1, 1)
        self.label_hdg = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_hdg.setObjectName("label_hdg")
        self.gridLayout.addWidget(self.label_hdg, 2, 1, 1, 1)
        self.label_vmd = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_vmd.setObjectName("label_vmd")
        self.gridLayout.addWidget(self.label_vmd, 3, 0, 1, 1)
        self.label_armed = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_armed.setObjectName("label_armed")
        self.gridLayout.addWidget(self.label_armed, 3, 1, 1, 1)
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(841, 291, 351, 121))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.btn_asama1 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.btn_asama1.setObjectName("btn_asama1")
        self.gridLayout_2.addWidget(self.btn_asama1, 0, 0, 1, 1)
        self.btn_asama3 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.btn_asama3.setObjectName("btn_asama3")
        self.gridLayout_2.addWidget(self.btn_asama3, 1, 0, 1, 1)
        self.btn_asama2 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.btn_asama2.setObjectName("btn_asama2")
        self.gridLayout_2.addWidget(self.btn_asama2, 0, 1, 1, 1)
        self.btn_asama4 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.btn_asama4.setObjectName("btn_asama4")
        self.gridLayout_2.addWidget(self.btn_asama4, 1, 1, 1, 1)
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(841, 431, 351, 71))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.btn_takeoff = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.btn_takeoff.setObjectName("btn_takeoff")
        self.horizontalLayout_2.addWidget(self.btn_takeoff)
        self.btn_land = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.btn_land.setObjectName("btn_land")
        self.horizontalLayout_2.addWidget(self.btn_land)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(840, 10, 351, 61))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.btn_connect = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.btn_connect.setObjectName("btn_connect")
        self.horizontalLayout_3.addWidget(self.btn_connect)
        self.btn_cut = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.btn_cut.setObjectName("btn_cut")
        self.horizontalLayout_3.addWidget(self.btn_cut)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(16, 14, 101, 51))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_4.setGeometry(QtCore.QRect(841, 521, 351, 71))
        self.horizontalLayoutWidget_4.setObjectName("horizontalLayoutWidget_4")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_4)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.btn_sht = QtWidgets.QPushButton(self.horizontalLayoutWidget_4)
        self.btn_sht.setObjectName("btn_sht")
        self.horizontalLayout_5.addWidget(self.btn_sht)
        self.btn_rtl = QtWidgets.QPushButton(self.horizontalLayoutWidget_4)
        self.btn_rtl.setObjectName("btn_rtl")
        self.horizontalLayout_5.addWidget(self.btn_rtl)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1201, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "KIZIL KUASAR - ORION"))
        self.label_cam.setText(_translate("MainWindow", "Kamera"))
        self.label_msg.setText(_translate("MainWindow", "Yazılım Mesajları"))
        self.btn_logs.setText(_translate("MainWindow", "Kayıtlar"))
        self.label_spd.setText(_translate("MainWindow", "Hız: -"))
        self.label_lat.setText(_translate("MainWindow", "Enlem: -"))
        self.label_lon.setText(_translate("MainWindow", "Boylam: -"))
        self.label_alt.setText(_translate("MainWindow", "Irtifa: -"))
        self.label_btr.setText(_translate("MainWindow", "Batarya: -"))
        self.label_hdg.setText(_translate("MainWindow", "Heading: -"))
        self.label_vmd.setText(_translate("MainWindow", "Mod: -"))
        self.label_armed.setText(_translate("MainWindow", "Disarmed"))
        self.btn_asama1.setText(_translate("MainWindow", "Aşama 1"))
        self.btn_asama3.setText(_translate("MainWindow", "Aşama 3"))
        self.btn_asama2.setText(_translate("MainWindow", "Aşama 2"))
        self.btn_asama4.setText(_translate("MainWindow", "Aşama 4"))
        self.btn_takeoff.setText(_translate("MainWindow", "Kalkış"))
        self.btn_land.setText(_translate("MainWindow", "Iniş"))
        self.btn_connect.setText(_translate("MainWindow", "BAĞLAN"))
        self.btn_cut.setText(_translate("MainWindow", "BAĞLANTIYI KES"))
        self.label.setText(_translate("MainWindow", "LOGO"))
        self.btn_sht.setText(_translate("MainWindow", "Atış Izni Ver"))
        self.btn_rtl.setText(_translate("MainWindow", "Kalkış Noktasına Dön"))
