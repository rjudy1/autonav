# change the command to set the lights, the buzzer, and the motors
# must not send commands to the motors back to back
import serial
import time

SP = serial.Serial('/dev/TEENSY_PORT', 115200)

SP.write(f"B,0,**".encode())
time.sleep(.5)
print("start motor power now")
SP.write("M,89,89,**".encode())
time.sleep(5)
p = True

print("starting actual commands")
n = True
while n:
	x = int(input("speed: "))
	a = int(input("angle: "))
	SP.write(f"M,{x+89},{a+89},**\n".encode())
	time.sleep(.5)
#	# other examples
#	SP.write(f"Y,{int(p)},**".encode())
#	SP.write(f"G,{int(p)},**".encode())
#	SP.write(f"B,{int(p)},**".encode())
#	p = not p

SP.close()
print('finished')
