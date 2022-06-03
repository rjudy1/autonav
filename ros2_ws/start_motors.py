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

if len(sys.argv) > 1:
  angle = float(sys.argv[1])
else:
  angle = int(input("compass reported heading: "))

heading = angle * math.pi / 180
if angle > 180:
  heading -= math.pi * 2

print(f"initial_heading: {heading}")
