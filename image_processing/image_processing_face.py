import cv2
import numpy as np
import time

# modelin yüklenmesi ve hazırlığı
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

cap = cv2.VideoCapture(0)

size = 1.6
sil, img = cap.read()
img = cv2.resize(img, None, fx=size, fy=size)
height, width, channels = img.shape  #640,480 if size == 1
oran = 0.4
#hedef alaninin kenar uzunluklarının yaty ve dikeyle oranı
hedef = (int(width * (1-oran)//2), int(height * (1-oran)//2), int(width*oran), int(height*oran))
izgara = [((0, 0), (hedef[0], hedef[1])),                                   #kuzey-batı bolgesi
          ((hedef[0], 0), (hedef[0]+hedef[2], hedef[1])),                   #kuzey bölgesi
          ((hedef[0]+hedef[2], 0), (width, hedef[1])),                      #kuzey-doğu bölgesi
          ((0, hedef[1]), (hedef[0], hedef[1]+hedef[3])),                   #batı bölgesi
          ((hedef[0], hedef[1]), (hedef[0]+hedef[2], hedef[1]+hedef[3])),   #merkez bölge
          ((hedef[0]+hedef[2], hedef[1]), (width, hedef[1]+hedef[3])),      #doğu bölgesi
          ((0, hedef[1]+hedef[3]), (hedef[0], height)),                     #güney-batı bölgesi
          ((hedef[0], hedef[1]+hedef[3]), (hedef[0]+hedef[2], height)),     #güney bölgesi
          ((hedef[0]+hedef[2], hedef[1]+hedef[3]), (width, height))]        #güney-doğu bölgesi
starting_time = time.time()
font = cv2.FONT_HERSHEY_PLAIN

def icinde_mi(merkez_nokta, kose_nokta1, kose_nokta2):
    # girilen merkez nokta, sol-üst ve sağ-alt noktasi verilen dikdörtgenin içerisinde mi kontrol eden fonksiyon
    if kose_nokta1[0] <= merkez_nokta[0] <= kose_nokta2[0] and kose_nokta1[1] <= merkez_nokta[1] <= kose_nokta2[1]:
        return True
    else:
        return False

def merkezler_arasi_mesafe(balon):
    # girilen balon dikdörtgeninin merkezi ile hedef alanı merkezi arasındaki mesafeyi veren fonksiyon
    x1, y1, w1, h1 = balon
    x2, y2, w2, h2 = hedef
    return int(
        (abs((x1 + (w1 // 2)) - (x2 + (w2 // 2))) ** 2 + abs((y1 + (h1 // 2)) - (y2 + (h2 // 2))) ** 2) ** (1 / 2))

def yon(balon):
    # verilen balonun izgara içerisinde ki hangi bölgede olduğu bilgisini veren fonksiyon
    x1, y1, w1, h1 = balon
    balon_merkez_nokta = (x1+(w1//2), y1+(h1//2))

    if icinde_mi(balon_merkez_nokta, izgara[0][0], izgara[0][1]):
        return f"kb"
    elif icinde_mi(balon_merkez_nokta, izgara[1][0], izgara[1][1]):
        return f"kk"
    elif icinde_mi(balon_merkez_nokta, izgara[2][0], izgara[2][1]):
        return f"kd"
    elif icinde_mi(balon_merkez_nokta, izgara[3][0], izgara[3][1]):
        return f"bb"
    elif icinde_mi(balon_merkez_nokta, izgara[4][0], izgara[4][1]):
        return f"mm"
    elif icinde_mi(balon_merkez_nokta, izgara[5][0], izgara[5][1]):
        return f"dd"
    elif icinde_mi(balon_merkez_nokta, izgara[6][0], izgara[6][1]):
        return f"gb"
    elif icinde_mi(balon_merkez_nokta, izgara[7][0], izgara[7][1]):
        return f"gg"
    elif icinde_mi(balon_merkez_nokta, izgara[8][0], izgara[8][1]):
        return f"gd"


def main_loop():
    # ana fonksiyon
    frame_counter = 0
    while cap.isOpened():
        frame_counter += 1
        _, img = cap.read()
        img = cv2.resize(img, None, fx=size, fy=size)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # nesne tanima
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                roi_gray = gray[y:y + h, x:x + w]
                color = (0, 255, 0)  # BGR
                stroke = 2

                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(img, ((x + (w // 2)), (y + (h // 2))), 10, (0, 250, 0), cv2.FILLED)
                print(yon([x,y,w,h]))

        #hedef alani cizdirme
        cv2.rectangle(img, (hedef[0], hedef[1]),
                      (hedef[0] + hedef[2], hedef[1] + hedef[3]),
                      (170, 20, 125), 3)
        #cv2.putText(img, "Hedef Alani", (hedef[0]+5, hedef[1]+20), font, 1, (170, 20, 125), 2)

        #fps
        elapsed_time = time.time() - starting_time
        fps = frame_counter / elapsed_time
        cv2.putText(img, "FPS: " + str(round(fps, 2)), (10, 50), font, 2, (0, 0, 0), 2)

        #görüntüyü ekrana bastirma
        cv2.imshow("iha goruntu", img)

        # komutlar (renk degistirme, kapatma, vb.)
        komut = cv2.waitKey(20)
        if komut & 0xFF == ord('q'):
            print(f"akış sonlandırıldı..!")
            break

    cap.release()
    cv2.destroyAllWindows()

#start
main_loop()
