# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'orion_diyalog_connect.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Baglanti(object):
    def setupUi(self, Baglanti):
        Baglanti.setObjectName("Baglanti")
        Baglanti.resize(517, 181)
        Baglanti.setMinimumSize(QtCore.QSize(517, 181))
        Baglanti.setMaximumSize(QtCore.QSize(517, 181))
        self.horizontalLayoutWidget = QtWidgets.QWidget(Baglanti)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(70, 120, 381, 41))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.btn_pos = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.btn_pos.setObjectName("btn_pos")
        self.horizontalLayout.addWidget(self.btn_pos)
        self.btn_neg = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.btn_neg.setObjectName("btn_neg")
        self.horizontalLayout.addWidget(self.btn_neg)
        self.label_ip_port = QtWidgets.QLabel(Baglanti)
        self.label_ip_port.setGeometry(QtCore.QRect(120, 20, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_ip_port.setFont(font)
        self.label_ip_port.setWordWrap(True)
        self.label_ip_port.setObjectName("label_ip_port")
        self.edit_ip_port = QtWidgets.QLineEdit(Baglanti)
        self.edit_ip_port.setGeometry(QtCore.QRect(70, 70, 381, 31))
        self.edit_ip_port.setObjectName("edit_ip_port")

        self.retranslateUi(Baglanti)
        QtCore.QMetaObject.connectSlotsByName(Baglanti)

    def retranslateUi(self, Baglanti):
        _translate = QtCore.QCoreApplication.translate
        Baglanti.setWindowTitle(_translate("Baglanti", "BA??LAN"))
        self.btn_pos.setText(_translate("Baglanti", "BAGLAN"))
        self.btn_neg.setText(_translate("Baglanti", "IPTAL"))
        self.label_ip_port.setText(_translate("Baglanti", "ip:port girin (127.0.0.1:10000)"))
