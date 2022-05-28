import serial
import time
from utils.utils import *

RIGHT_WHEEL = 0x08


class Wheels:
    def __init__(self, port="/dev/ttyUSB0", BAUD=57600, addr = 0b110):
        self.serialPort = serial.Serial(port, BAUD)
        start = 0x80
        self.addr = addr
        self.serialPort.write(start.to_bytes(1, 'big'))

    def __del__(self):
        self.serialPort.close()

    def control_wheels(self, left_speed, right_speed):
        header = 85
        left_speed = int(left_speed*2 + 127)
        right_speed = int(right_speed*2 + 127)

        # send the left command
        left_cmd = [header&0xFF, self.addr&0xFF, left_speed&0xFF, (left_speed+header+self.addr)&0xFF]
        right_cmd = [header&0xFF, RIGHT_WHEEL|self.addr&0xFF, right_speed&0xFF, (right_speed+header+RIGHT_WHEEL|self.addr)&0xFF]

        self.serialPort.write(bytes(right_cmd))
        # time.sleep(.05)
        self.serialPort.write(bytes(left_cmd))

        self.serialPort.flush()
        return left_speed, right_speed