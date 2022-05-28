################################
# AutoNav 2022 Competition Robot
# Package: encoders
# File: encoder_pub.py
# Purpose: interface with teensy to report wheel turning and
# Date Modified: 21 May 2022
# To run: ros2 run rs2l_transform transform
################################

from custom_msgs.msg import EncoderData
from custom_msgs.msg import LightCmd
import rclpy
from rclpy.node import Node
import serial
from time import sleep

ticks_per_meter = 1404


class Encoder(Node):
    def __init__(self):
        super().__init__('encoders')
        self.declare_parameter('/TeensyEncodersPort', '/dev/ttyACM1')
        self.declare_parameter('/TeensyBaudrate', 115200)
        self.declare_parameter('/TeensyUpdateDelay', .02)
        self.declare_parameter('/Debug', False)

        self.serialPort = serial.Serial(self.get_parameter('/TeensyEncodersPort').value,
                                        self.get_parameter('/TeensyBaudrate').value)
        self.serialPort.flushInput()
        self.serialPort.flushOutput()

        #  publish for right and left encoder distances
        self.encoder_pub = self.create_publisher(EncoderData, 'encoder_data', 10)
        self.rate = self.get_parameter('/TeensyUpdateDelay').value

        self.serialPort.write('R,1,**'.encode('utf-8'))
        self.timer = self.create_timer(self.rate, self.timer_callback)
        self.light_sub = self.create_subscription(LightCmd, "light_events", self.light_callback, 10)

        self.past_left_ticks = 0
        self.past_right_ticks = 0
        self.toggle = False

    def __del__(self):
        self.close()

    def light_callback(self, msg):
        cmd = ''
        if msg.type == 'G' or msg.type == 'Y' or msg.type == 'B':
            cmd += msg.type
            if msg.on:
                cmd += ',1,**'
            else:
                cmd += ',0,**'
            try:
                self.serialPort.write(cmd.encode('utf-8'))
            except serial.serialutil.SerialException:
                sleep(0.5)
            except Exception as ex:
                template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                message = template.format(type(ex).__name__, ex.args)
                self.get_logger().warning(message)
                sleep(0.5)
                self.close()
        else:
            self.get_logger().info(f"Message type {msg.type} invalid")


    def timer_callback(self):
        try:
            self.serialPort.write('Q,**'.encode('utf-8'))
            read = self.serialPort.readline().decode('utf-8')
            data = read.split(',')
            if data[0] == 'E' and len(data) == 4 and data[3] == '**\r\n':
                # find ticks since last change
                left_ticks = int(data[1]) - self.past_left_ticks
                right_ticks = int(data[2]) - self.past_right_ticks

                # update history
                self.past_left_ticks = int(data[1])
                self.past_right_ticks = int(data[2])

                left_dist = left_ticks / ticks_per_meter
                right_dist = right_ticks / ticks_per_meter

                if self.get_parameter('/Debug').value:
                    self.get_logger().info(f'left_dist {left_dist}')
                    self.get_logger().info(f'right_dist {right_dist}')

                msg = EncoderData()
                msg.left = -left_dist
                msg.right = -right_dist
                self.encoder_pub.publish(msg)
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
    encoder_pub = Encoder()
    try:
        rclpy.spin(encoder_pub)
    except KeyboardInterrupt:
        encoder_pub.close()
