from PyQt5 import QtWidgets
from gui.orion_gui_dialog_logs import Ui_Logs


class DialogLogs(QtWidgets.QDialog, Ui_Logs):

    def __init__(self, yazilim_mesajlari):
        super(DialogLogs, self).__init__()
        #self.ui = Ui_MainWindow()
        self.setupUi(self)

        self.btn_ok.clicked.connect(self.close)

        for t, msg in yazilim_mesajlari:
            self.listWidget_logs.addItem(QtWidgets.QListWidgetItem(f"{t} -> {msg}"))
