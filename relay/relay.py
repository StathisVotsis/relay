#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

#sudo apt install rpi.gpio-common
#sudo adduser "${USER}" dialout
#sudo reboot - all three needed

class RelayPublisher(Node):
    def __init__(self):
        super().__init__("relay_publisher")
        self.relay_pin = 12
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin , GPIO.LOW)

        self.sub_ = self.create_subscription(String, "relay", self.msgCallback, 10)#siren is the topic
        self.sub_

    def msgCallback(self, msg):
        self.get_logger().info("New message received on topic relay")
        x = int(msg.data.encode("utf-8"))
        if x == 0:
            self.get_logger().info("New message 0")
            GPIO.output(self.relay_pin, GPIO.LOW)
        elif x == 1:
            self.get_logger().info("New message 1")
            GPIO.output(self.relay_pin, GPIO.HIGH)
        else:
            self.get_logger().info("New message ")


def main():
    rclpy.init()

    relay_publisher = RelayPublisher()
    rclpy.spin(relay_publisher)
    
    relay_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

