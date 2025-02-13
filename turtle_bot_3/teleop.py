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
        self.linear_speed = self.declare_parameter('linear_speed', 10.0).value
        self.angular_speed = self.declare_parameter('angular_speed', 5.0).value

        
        # Publicador
        self.publisher = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        
        # Temporizador para leer el teclado
        self.timer = self.create_timer(0.1, self.read_keyboard)
        
        self.get_logger().info("Nodo turtle_bot_teleop iniciado. Usa las teclas WASD para mover el robot.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key.lower()

    def read_keyboard(self):
        twist = Twist()
        key = self.get_key()

        if key == 'w':
            twist.linear.x = self.linear_speed   # Adelante
            twist.angular.z = 0.0
            self.get_logger().info("Movimiento: Adelante")
        elif key == 's':
            twist.linear.x = -self.linear_speed  # Atrás
            twist.angular.z = 0.0
            self.get_logger().info("Movimiento: Atrás")
        elif key == 'a':
            twist.angular.z = self.angular_speed # Izquierda
            twist.linear.x = 0.0
            self.get_logger().info("Movimiento: Izquierda")
        elif key == 'd':
            twist.angular.z = -self.angular_speed # Derecha
            twist.linear.x = 0.0
            self.get_logger().info("Movimiento: Derecha")
        elif key == ' ':  # Barra espaciadora
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Robot detenido")
        else:
            # Mantiene el último movimiento (sin detenerse)
            return
        
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
