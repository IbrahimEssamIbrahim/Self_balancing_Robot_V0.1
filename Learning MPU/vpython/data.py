import serial
import time
ArduinoData = serial.Serial('com3', 115200)
time.sleep(1)
while(1 == 1):
    while(ArduinoData.inWaiting == 0):
        pass
    dataPacket = ArduinoData.readline()
    dataPacket=str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(' ')
    pitch= float(splitPacket[0])
    roll= float(splitPacket[1])
    print("pitch=", pitch," roll=", roll)
    