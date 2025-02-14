#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TurtleBotTeleop(Node):
    """
    Nodo de teleoperación para controlar el TurtleBot usando el teclado.
    Permite mover el robot en las direcciones adelante, atrás, izquierda y derecha,
    así como detenerlo con la barra espaciadora.
    """
    def __init__(self):
        """
        Inicializa el nodo de teleoperación y configura el terminal.
        """
        super().__init__('turtle_bot_teleop')
        
        # Configuración inicial del terminal para lectura no bloqueante de teclas
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Parámetros de velocidad para movimiento lineal y angular
        self.linear_speed = self.declare_parameter('linear_speed', 10.0).value
        self.angular_speed = self.declare_parameter('angular_speed', 5.0).value
        
        # Publicador de mensajes Twist en el tópico /turtlebot_cmdVel
        self.publisher = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        
        # Temporizador para verificar continuamente las teclas presionadas
        self.timer = self.create_timer(0.5, self.read_keyboard)
        
        self.get_logger().info("Nodo turtle_bot_teleop iniciado. Usa las teclas WASD para mover el robot.")

    def get_key(self):
        """
        Lee una tecla presionada sin bloquear el flujo del programa.
        
        Returns:
            str: La tecla presionada en minúsculas o cadena vacía si no se presionó ninguna.
        """
        tty.setraw(sys.stdin.fileno())  # Cambia a modo de lectura sin bloqueo
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Espera brevemente a la entrada del usuario
        if rlist:
            key = sys.stdin.read(1)  # Lee un solo caracter
        else:
            key = ''  # No se presionó ninguna tecla
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)  # Restaura la configuración del terminal
        return key.lower()

    def read_keyboard(self):
        """
        Lee el teclado y publica comandos de movimiento en función de la tecla presionada.
        
        Las teclas asignadas son:
        - W: Adelante
        - S: Atrás
        - A: Girar a la izquierda
        - D: Girar a la derecha
        - Espacio: Detener el robot
        """
        twist = Twist()  # Inicializa el mensaje Twist
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
        else: 
            twist.linear.x = 0.0 # Si no se presiona ninguna tecla se detiene el robot
            twist.angular.z = 0.0
            self.get_logger().info("Robot detenido")
        
        # Publica el mensaje Twist con el movimiento correspondiente
        self.publisher.publish(twist)

    def __del__(self):
        """
        Restablece la configuración del terminal al destruir el nodo.
        Esto asegura que el terminal vuelva a su modo normal al salir.
        """
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.get_logger().info("Configuración del terminal restablecida")

def main(args=None):
    """
    Función principal que inicializa el nodo y lo mantiene en ejecución hasta que se interrumpa.
    """
    rclpy.init(args=args)
    node = TurtleBotTeleop()
    try:
        rclpy.spin(node)  # Mantiene el nodo en ejecución
    except KeyboardInterrupt:
        pass  # Maneja la interrupción con Ctrl+C
    finally:
        node.destroy_node()  # Destruye el nodo correctamente
        rclpy.shutdown()     # Finaliza el contexto de ROS 2

if __name__ == '__main__':
    main()
