# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'orion_diyalog_logs.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Logs(object):
    def setupUi(self, Logs):
        Logs.setObjectName("Logs")
        Logs.resize(621, 671)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Logs.sizePolicy().hasHeightForWidth())
        Logs.setSizePolicy(sizePolicy)
        Logs.setMinimumSize(QtCore.QSize(621, 671))
        Logs.setMaximumSize(QtCore.QSize(621, 671))
        self.listWidget_logs = QtWidgets.QListWidget(Logs)
        self.listWidget_logs.setGeometry(QtCore.QRect(10, 10, 601, 591))
        self.listWidget_logs.setObjectName("listWidget_logs")
        self.horizontalLayoutWidget = QtWidgets.QWidget(Logs)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 610, 601, 51))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.btn_ok = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.btn_ok.setFont(font)
        self.btn_ok.setObjectName("btn_ok")
        self.horizontalLayout.addWidget(self.btn_ok)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)

        self.retranslateUi(Logs)
        QtCore.QMetaObject.connectSlotsByName(Logs)

    def retranslateUi(self, Logs):
        _translate = QtCore.QCoreApplication.translate
        Logs.setWindowTitle(_translate("Logs", "LOGS"))
        self.btn_ok.setText(_translate("Logs", "Tamam"))
