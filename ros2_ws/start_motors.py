import math
import serial
import sys
import time

SP = serial.Serial('/dev/TEENSY_PORT', 115200)

SP.write("M,89,89,**".encode())
time.sleep(.25)
print("Enable power to motor controller now.")
SP.write("G,0,**".encode())
time.sleep(.25)
SP.write("Y,0,**".encode())
time.sleep(.25)
SP.write("B,0,**".encode())
time.sleep(.25)
input('press enter when done')
time.sleep(1)
