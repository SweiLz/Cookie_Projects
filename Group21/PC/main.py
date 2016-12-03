import cv2
import numpy as np
import serial
import time


# Serial = serial.Serial('com21')
# Serial.baudrate = 57600

def send(param):
    cmd = '@'
    for para in param:
        cmd += str(para)+','
    cmd += '#'
    print (cmd)
    # Serial.write(cmd)


# time.sleep(5)
value = [1000,500,1200,200,100]
send(value)
# time.sleep(5)
value = [1500,1500,1000,100,80]
send(value)
# time.sleep(5)
value = [0,10,100,10,20]
send(value)