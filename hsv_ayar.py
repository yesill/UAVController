import cv2
import numpy as np


def param():
    global sol_tik, sag_tik
    sol_tik = 0
    sag_tik = 0


def renktespit(bgr_goruntu, hsv_goruntu):
    def renkuzayi(durum, yty, dky, byrk, prm):
        global sol_tik, sag_tik
        global min_mavi, max_mavi, min_yesil, max_yesil, min_kirmizi, max_kirmizi
        global min_renk, max_renk, min_doygunluk, max_doygunluk, min_deger, max_deger
        if durum == cv2.EVENT_LBUTTONDOWN:  # Farenin sol tuş durumunu kontrol eder.
            mavi = bgr_goruntu[
                dky, yty, 0]  # Kare değişkeninin dikey(dky) ve yatay(yty) kordinatının RGB renk değerini kontrol eder.3.Parametre-- B=0,G=1,R=2
            yesil = bgr_goruntu[dky, yty, 1]
            kirmizi = bgr_goruntu[dky, yty, 2]
            rgb_uzay = bgr_goruntu[dky, yty]
            print("BGR Değerleri: ", rgb_uzay)
            # print("Piksel Kordinatları: X:",yty,"Y:",dky)
            cv2.namedWindow('BGR Degeri')  # BGR Resmi adında mini pencere oluşturur
            bgr_rengi = np.zeros((250, 250, 3), np.uint8)  # Bgr renginin gösterilmesi için boş resim oluşturuldu.
            bgr_rengi[:] = (mavi, yesil, kirmizi)  # bgr resmi seçilen noktanın rengine göre boyandı.
            cv2.imshow('BGR Degeri', bgr_rengi)  # Bgr resminin görüntüsü ekrana verildi.
            if sol_tik == 0:
                min_mavi = max_mavi = mavi
                min_yesil = max_yesil = yesil
                min_kirmizi = max_kirmizi = kirmizi
            if mavi < min_mavi:
                min_mavi = mavi
            if mavi > max_mavi:
                max_mavi = mavi
            if yesil < min_yesil:
                min_yesil = yesil
            if yesil > max_yesil:
                max_yesil = yesil
            if kirmizi < min_kirmizi:
                min_kirmizi = kirmizi
            if kirmizi > max_kirmizi:
                max_kirmizi = kirmizi
            sol_tik = sol_tik + 1
            if sol_tik == 5:
                print("-------------------Önerilen MİN BGR Değerleri------------------------")
                print("[", min_mavi, min_yesil, min_kirmizi, "]")
                print("-------------------Önerilen MAX BGR Değerleri------------------------")
                print("[", max_mavi, max_yesil, max_kirmizi, "]")
                print("---------------------------------------------------------------------")
                sol_tik = 0

        if durum == cv2.EVENT_RBUTTONDOWN:  # Farenin sağ tuş durumunu kontrol eder.
            renk = hsv_goruntu[dky, yty, 0]  # Hue değeri
            doygunluk = hsv_goruntu[dky, yty, 1]  # Saturation değeri
            deger = hsv_goruntu[dky, yty, 2]  # Value değeri
            hsv_uzay = hsv_goruntu[dky, yty]  # dikey ve yatay kordinatında bulunan pikselin HSV Değerini çeker
            print("HSV Değerleri: ", hsv_uzay)
            # print("Piksel Kordinatları: X:",yty,"Y:",dky)
            cv2.namedWindow('HSV Degeri')  # HSV Değeri adında mini pencere oluşturur
            hsv_rengi = np.zeros((250, 250, 3), np.uint8)  # HSV renginin gösterilmesi için boş resim oluşturuldu.
            hsv_rengi[:] = (renk, doygunluk, deger)  # HSV resmi seçilen noktanın rengine göre boyandı.
            cv2.imshow('HSV Degeri', hsv_rengi)  # HSV resminin görüntüsü ekrana verildi.
            if sag_tik == 0:
                min_renk = max_renk = renk
                min_doygunluk = max_doygunluk = doygunluk
                min_deger = max_deger = deger
            if renk < min_renk:
                min_renk = renk
            if renk > max_renk:
                max_renk = renk
            if doygunluk < min_doygunluk:
                min_doygunluk = doygunluk
            if doygunluk > max_doygunluk:
                max_doygunluk = doygunluk
            if deger < min_deger:
                min_deger = deger
            if deger > max_deger:
                max_deger = deger
            sag_tik = sag_tik + 1
            if sag_tik == 5:
                print("-------------------Önerilen MİN HSV Değerleri------------------------")
                print(f"[{min_renk},{min_doygunluk},{min_deger}]")
                print("-------------------Önerilen MAX HSV Değerleri------------------------")
                print(f"[{max_renk},{max_doygunluk},{max_deger}]")
                print("---------------------------------------------------------------------")
                sag_tik = 0
                msg = f"[{min_renk},{min_doygunluk},{min_deger}]\n[{max_renk},{max_doygunluk},{max_deger}]"
                with open(f"hsv.txt", "w") as file:
                    file.write(msg)

    cv2.imshow('Renk Tespit', bgr_goruntu)
    cv2.setMouseCallback('Renk Tespit', renkuzayi)


param()
kamera = cv2.VideoCapture(1, cv2.CAP_DSHOW)
while (True):
    _, kare = kamera.read()
    hsv = cv2.cvtColor(kare, cv2.COLOR_BGR2HSV)
    renktespit(kare, hsv)
    if cv2.waitKey(1) == 27:
        break
kamera.release()
cv2.destroyAllWindows()