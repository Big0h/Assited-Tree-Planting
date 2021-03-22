import serial
import time
import string
import pynmea2

#("zero")
while True:
    port="/dev/ttyAMA0"
    ser=serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata=ser.readline()
    #print("one")
    print(newdata)
    #print(type(newdata[0:6]))
    #print(type(str(newdata[0:6])))
    if newdata[0:6]==bytes('$GPRMC','utf-8'):
    #    print("two")
        newmsg=pynmea2.parse(str(newdata))
    #    lat=newmsg.latitude
    #    lng=newmsg.longitude
    #    gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
    #    print(gps)