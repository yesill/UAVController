from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from gui.orion_gui_dialog_asama2 import Ui_Dialog
from subprocesses import konumlar


class DialogAsama2(QtWidgets.QDialog, Ui_Dialog):

    start_signal = pyqtSignal(bool)

    def __init__(self, mission, ip_port):
        super(DialogAsama2, self).__init__()
        self.setupUi(self)

        self.line_lat.setText(f"{konumlar.asama2['lat']}")
        self.line_lon.setText(f"{konumlar.asama2['lon']}")
        self.line_alt.setText(f"{konumlar.asama2['alt']}")
        self.line_spd.setText(f"{konumlar.asama2['spd']}")

        self.btn_start.clicked.connect(self.start)
        self.btn_cancel.clicked.connect(self.close)

    def start(self):
        self.close()
        self.start_signal.emit(True)
