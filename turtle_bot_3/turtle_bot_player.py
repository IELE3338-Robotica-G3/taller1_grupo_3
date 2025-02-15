#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import time

class TurtleBotPlayerNode(Node):
    def __init__(self):
        super().__init__('turtle_bot_player')

        # Declarar el parámetro para la ruta del archivo
        self.declare_parameter('file_path', '')

        # Publicador de mensajes Twist en el tópico /turtlebot_cmdVel
        self.publisher = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        
        # Servicio para recibir el nombre del archivo
        self.srv = self.create_service(
            Trigger,
            'play_recording',
            self.reproducir_callback
        )

        self.get_logger().info('Nodo TurtleBotPlayer listo para reproducir recorridos.')

    def reproducir_callback(self, request, response):
        """
        Callback para el servicio de reproducción de trayectoria.
        """
        file_path = self.get_parameter('file_path').get_parameter_value().string_value
        if not file_path:
            response.success = False
            response.message = "No se ha especificado ningún archivo."
            return response
        self.get_logger().info(f"Reproduciendo trayectoria desde: {file_path}")
        self.play_recording(file_path)
        response.success = True
        response.message = "Reproducción completada con éxito."
        return response

    def play_recording(self, file_path):
        """
        Lee el archivo de trayectoria y publica los comandos.
        """
        try:
            with open(file_path, 'r') as archivo:
                self.get_logger().info("Archivo abierto exitosamente")
                lines = archivo.readlines()

                for line in lines:
                    # Parseo de la línea
                    data = line.strip().split(', ')
                    lin_x = float(data[1].split('=')[1])
                    ang_z = float(data[2].split('=')[1])

                    time.sleep(0.5)  # Espera el tiempo necesario

                    # Publica el comando de velocidad
                    twist_msg = Twist()
                    twist_msg.linear.x = lin_x
                    twist_msg.angular.z = ang_z
                    self.get_logger().info(f"Send: linear_x={lin_x}, angular_z={ang_z}")
                    self.publisher.publish(twist_msg)

                self.get_logger().info("Reproducción terminada")
        except Exception as e:
            self.get_logger().info(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotPlayerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
