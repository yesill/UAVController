import serial, time, gorev


#mission = gorev.Mission()

#mission.baglan(ip_port="port_name")

port = serial.Serial("COM3")

while True:
    time.sleep(1)
    port.write(b"Naber lan")

    """
    while True:
        girdi = str(input("girdi: "))
    
        if girdi != "q" or girdi != "Q":
            port.write(bytes(girdi, "utf-8"))
        else:
            break
    """


