import serial
import time
from utils.utils import *

RIGHT_WHEEL = 0x80
BAUD_RATE = 19200
CCW = 0x40


class Wheels:
    def __init__(self, port="/dev/ttyUSB0", BAUD=19200):
        self.serialPort = serial.Serial(port, BAUD)

    def __del__(self):
        self.serialPort.close()

    def control_wheels(self, left_speed, right_speed):
        # control right wheel
        right_cmd = self.convert_to_hex_cmd(right_speed) | RIGHT_WHEEL

        # control left wheel
        left_cmd = self.convert_to_hex_cmd(left_speed)

        # self.serialPort.write(bytes('testing', "utf-8"))
        self.serialPort.write(right_cmd.to_bytes(1, 'big'))
        time.sleep(0.1)
        self.serialPort.write(left_cmd.to_bytes(1, 'big'))

        self.serialPort.flush()
        return self.convert_to_hex_cmd(left_speed), self.convert_to_hex_cmd(right_speed)

    def convert_to_hex_cmd(self, speed):
        # get magnitude of speed and direction
        cmd = abs(int(speed)) & 0xFF
        if speed < 0:
            cmd = (cmd | CCW)
        return cmd
