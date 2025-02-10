#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select


class TutleBotPlayer():

    def __init__(self):
        self.__init__("turtle_bot_player")

def main(args=None):
    rclpy.init(args=args)
    node = TutleBotPlayer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()