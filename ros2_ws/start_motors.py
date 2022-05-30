import serial
import time

SP = serial.Serial('/dev/TEENSY_PORT', 115200)
SP.write("M,89,89,**".encode())
print("Enable power to motor controller now.")
time.sleep(5)
