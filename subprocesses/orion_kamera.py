#orion kamera
import cv2, numpy, time
from PyQt5.QtCore import QThread, pyqtSignal


class ThreadKamera(QThread):

    #numpy.ndarray
    frame_signal = pyqtSignal(numpy.ndarray)

    def __init__(self, kamera_no):
        super(ThreadKamera, self).__init__()
        try:
            self.cap = cv2.VideoCapture(kamera_no)
            print(f"kamera baglantisi basarili. kamera_no: {kamera_no}")
        except:
            print(f"kamera baglantisi basarisiz! kamera_no: {kamera_no}")
            #belirtilen kameraya baglanilamazsa 0. kameraya baglanmayi dene(0'a baglanamiyorsa gg!)
            #self.__init__(kamera_no=0)

    def run(self):
        while True:
            _, frame = self.cap.read()
            self.frame_signal.emit(frame)
            #time.sleep(1/60)

    def stop(self):
        self.cap.release()
        self.terminate()
