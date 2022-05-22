################################
# AutoNav 2022 Competition Robot
# Package: encoders
# File: encoder_pub.py
# Purpose: interface with teensy to report wheel turning and
# Date Modified: 21 May 2022
# To run: ros2 run rs2l_transform transform
################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
from time import sleep

ticks_per_meter = 4500


class Encoder(Node):
    def __init__(self):
        super().__init__('encoders')
        self.declare_parameter('/TeensyEncodersPort', '/dev/ttyACM0')
        self.declare_parameter('/TeensyBaudrate', 115200)
        self.declare_parameter('/TeensyUpdateDelay', .10)
        self.declare_parameter('/Debug', True)

        self.serialPort = serial.Serial(self.get_parameter('/TeensyEncodersPort').value,
                                        self.get_parameter('/TeensyBaudrate').value)
        self.serialPort.flushInput()
        self.serialPort.flushOutput()

        #  publish for right and left encoder distances
        self.leftEncoder = self.create_publisher(Float32, 'left_dist', 10)
        self.rightEncoder = self.create_publisher(Float32, 'right_dist', 10)
        self.rate = self.get_parameter('/TeensyUpdateDelay').value

        self.serialPort.write('R,1,**'.encode('utf-8'))
        self.timer = self.create_timer(self.rate, self.timer_callback)

    def timer_callback(self):
        try:
            self.serialPort.write('Q,**'.encode('utf-8'))
            read = self.serialPort.readline().decode('utf-8')
            data = read.split(',')
            if data[0] == 'E' and len(data) == 4 and data[3] == '**\r\n':
                leftTicks = int(data[1])
                rightTicks = int(data[2])
                leftDist = leftTicks / ticks_per_meter
                rightDist = rightTicks / ticks_per_meter
                if self.get_parameter('/Debug'):
                    self.get_logger().info(leftDist)
                    self.get_logger().info(rightDist)

                msg = Float32()
                msg.data = leftDist
                self.leftEncoder.publish(msg)
                msg.data = rightDist
                self.rightEncoder.publish(msg)
            else:
                self.serialPort.flushInput()
            sleep(self.rate)
        except serial.serialutil.SerialException:
            sleep(0.5)
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            self.get_logger().warning(message)
            sleep(0.5)
            self.close()

    def close(self):
        self.serialPort.write('R,0,**'.encode('utf-8'))
        self.serialPort.close()


def main(args=None):
    rclpy.init()
    try:
        encoder_pub = Encoder()
        rclpy.spin(encoder_pub)
    except KeyboardInterrupt:
        encoder_pub.close()
