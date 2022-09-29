import cv2
import numpy as np
import time

# modelin yüklenmesi ve hazırlığı
net = cv2.dnn.readNet('../../yolo_custom_detection/yolov3_balloons.weights',
                      '../../yolo_custom_detection/yolov3_testing_balloons.cfg')
object = "balloon"
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

cap = cv2.VideoCapture("../../images/test/v2.mp4")

kirmizi_sinirlar = ((17, 20, 46), (84, 82, 166))    #BGR!!!
yesil_sinirlar = ((84, 83, 62), (185, 183, 148))    #BGR!!!
mavi_sinirlar = ((0, 0, 0), (0, 0, 0,)) #BGR!!

size = 0.7
sil, img = cap.read()
img = cv2.resize(img, None, fx=size, fy=size)
height, width, channels = img.shape  #640,480 if size == 1
oran = 0.1
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

def renk_yogunlugu(balon, frame):
    # girilen balonun merkez noktasından yatayda 20%, dikeyde 20% lik bir dikdörtgen belirler.
    # merkeze yakın bu dikdörtgenin içeriisndeki her bir pikselin BGR değerlerini okuyup ort. alır.
    # bulunan ortalama değer belirtilen rengin sınırları içerisinde ise renk baş harfini çıktı verir.
    tx, ty, tw, th = balon
    temp_oran = 0.1
    merkez_nokta = (tx+tw//2, ty+th//2)
    box_pt1 = (int(merkez_nokta[0]-tw*temp_oran), int(merkez_nokta[1]-th*temp_oran))
    box_pt2 = (int(merkez_nokta[0]+tw*temp_oran), int(merkez_nokta[1]+th*temp_oran))
    color = [0, 0, 0]   #bgr
    sayac = 0
    for j in range(box_pt1[0], box_pt2[0]-1):
        for i in range(box_pt1[1], box_pt2[1]-1):
            temp_color = frame[i, j]
            color[0] += int(temp_color[0])
            color[1] += int(temp_color[1])
            color[2] += int(temp_color[2])
            sayac += 1
    color[0] = color[0]//sayac
    color[1] = color[1]//sayac
    color[2] = color[2]//sayac
    #cv2.rectangle(frame, box_pt1, box_pt2, (0, 0, 0), 2)
    ks = kirmizi_sinirlar
    ys = yesil_sinirlar
    """ms = mavi_sinirlar"""
    if ks[0][0] <= color[0] <= ks[1][0] and ks[0][1] <= color[1] <= ks[1][1] and ks[0][2] <= color[2] <= ks[1][2]:
        return "r"
    elif ys[0][0] <= color[0] <= ys[1][0] and ys[0][1] <= color[1] <= ys[1][1] and ys[0][2] <= color[2] <= ys[1][2]:
        return f"g"
    """ mavi renk balonum yok o yüzden bu kısım hazır değil
    elif ms[0][0] <= color[0] <= ms[1][0] and ms[0][1] <= color[1] <= ms[1][1] and ms[0][2] <= color[2] <= ms[1][2]:
        return f"b" """

def main_loop():
    # ana fonksiyon
    frame_counter = 0
    renk = "a"
    while cap.isOpened():
        frame_counter += 1
        _, img = cap.read()
        img = cv2.resize(img, None, fx=size, fy=size)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # nesne tanima
        blob = cv2.dnn.blobFromImage(img, 0.00392, (256, 256), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                confidence = scores[np.argmax(scores)]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))

        # aynı balonu birden fazla kez tanımasını önlemek için indexes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        #en yakın balonun: (hedef merkeze uzaklığı, boxes listesi içerisindeki index numarası)
        en_yakin = (9999, 0)
        # istenilen renkten bir balon dahi yoksa istenilmeyen renktende olsa bir balon seçiyor bunu engellemek için
        istenilen_renkten_en_az_bir_balon_var = False
        for i in range(len(boxes)):
            # aynı balonu birden fazla kez tanımasını önlemek için indexes
            if i in indexes:
                x, y, w, h = boxes[i]
                label = f"{object} ({round(confidences[i]*100)}%)"

                #renk kontrolü
                if renk == renk_yogunlugu(balon=boxes[i], frame=img) or renk == "a":
                    istenilen_renkten_en_az_bir_balon_var = True
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(img, label, (x, y + 30), font, 2, (0, 255, 0), thickness=3)

                    #merkeze en yakın balonu bulmak için mesafeleri karşılaştırıyoruz
                    mesafe = merkezler_arasi_mesafe(boxes[i])
                    if mesafe <= en_yakin[0]:
                        en_yakin = (mesafe, i)
                else:
                    continue

        #en yakin balonun merkezinden, hedef alani merkezine çizgi çizdirme
        if len(boxes) > 0 and istenilen_renkten_en_az_bir_balon_var:
            x, y, w, h = boxes[en_yakin[1]]
            cv2.circle(img, ((x + (w // 2)), (y + (h // 2))), 10, (0, 250, 0), cv2.FILLED)
            cv2.line(img, ((x + w // 2), (y + h // 2)),
                     ((hedef[0] + hedef[2] // 2), (hedef[1] + hedef[3] // 2)),
                     (125, 125, 0), 3)

            # hedef alani balon içerisinde kalıyor mu?
            print(yon(boxes[en_yakin[1]]))
        #ekranda hic balon yok ise
        elif len(boxes) <= 0:
            print(f"görüş alanı içerisinde balon yok! kendi etrafında yavaşça dön!")

        #hedef alani cizdirme
        cv2.rectangle(img, (hedef[0], hedef[1]),
                      (hedef[0] + hedef[2], hedef[1] + hedef[3]),
                      (170, 20, 125), 3)
        #cv2.putText(img, "Hedef Alani", (hedef[0]+5, hedef[1]+20), font, 1, (170, 20, 125), 2)

        #fps
        elapsed_time = time.time() - starting_time
        fps = frame_counter / elapsed_time
        #cv2.putText(img, "FPS: " + str(round(fps, 2)), (10, 50), font, 2, (255, 0, 0), 2)

        #görüntüyü ekrana bastirma
        cv2.imshow("iha goruntu", img)

        # komutlar (renk degistirme, kapatma, vb.)
        komut = cv2.waitKey(20)
        if komut & 0xFF == ord('q'):
            print(f"akış sonlandırıldı..!")
            renk = False
            break
        elif komut & 0xFF == ord('a'):
            print("bütün balonlağg tespit ediliyoğg")
            renk = "a"
            continue
        elif komut & 0xFF == ord('r'):
            print("kirmizi balonlağg tespit ediliyoğg")
            renk = 'r'
            continue
        elif komut & 0xFF == ord('g'):
            print("yesil balonlağg tespit ediliyoğg")
            renk = 'g'
            continue
        """    
            print("movi balonlağg tespit ediliyoğg")
            renk = 'b'
            continue
        """

    cap.release()
    cv2.destroyAllWindows()

#start
main_loop()
