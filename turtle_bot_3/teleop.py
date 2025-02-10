#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TurtleBotTeleop(Node):
    def __init__(self):
        super().__init__('turtle_bot_teleop')
        
        # Configuración inicial del terminal
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Parámetros de velocidad
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_speed', 0.2),
                ('angular_speed', 1.0)
            ]
        )
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Publicador
        self.publisher = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        
        # Temporizador para leer el teclado
        self.timer = self.create_timer(0.1, self.read_keyboard)
        
        self.get_logger().info("Nodo iniciado. Usa las teclas WASD para mover el robot.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def read_keyboard(self):
        twist = Twist()
        key = self.get_key()
        if key == 'w':
            twist.linear.x = self.linear_speed
        elif key == 's':
            twist.linear.x = -self.linear_speed
        elif key == 'a':
            twist.angular.z = self.angular_speed
        elif key == 'd':
            twist.angular.z = -self.angular_speed
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.publisher.publish(twist)

    def __del__(self):
        """Restablece la configuración del terminal al destruir el nodo"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.get_logger().info("Configuración del terminal restablecida")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
