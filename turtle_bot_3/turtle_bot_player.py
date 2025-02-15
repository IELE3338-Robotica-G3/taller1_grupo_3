#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import time

class TurtleBotPlayerNode(Node):
    """
    Nodo que reproduce una secuencia de acciones previamente guardada para el TurtleBot.
    La trayectoria se reproduce leyendo un archivo .txt con comandos de movimiento.
    """
    def __init__(self):
        """
        Inicializa el nodo TurtleBotPlayerNode.
        Declara el parámetro para la ruta del archivo.
        Crea el publicador de comandos Twist y el servicio de reproducción.
        """
        super().__init__('turtle_bot_player')

        # Declarar el parámetro para la ruta del archivo
        # Este parámetro debe establecerse antes de reproducir la trayectoria
        self.declare_parameter('file_path', '')

        # Publicador de mensajes Twist en el tópico /turtlebot_cmdVel
        self.publisher = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
        
        # Servicio para recibir el nombre del archivo y reproducir la trayectoria
        self.srv = self.create_service(
            Trigger,
            'play_recording',
            self.reproducir_callback
        )

        self.get_logger().info('Nodo TurtleBotPlayer listo para reproducir recorridos.')

    def reproducir_callback(self, request, response):
        """
        Callback para el servicio de reproducción de trayectoria.
        Verifica si el archivo está especificado y llama a la función de reproducción.
        
        Args:
            request (Trigger.Request): La solicitud del servicio (sin datos en Trigger).
            response (Trigger.Response): La respuesta del servicio que indica éxito o fallo.
        
        Returns:
            Trigger.Response: La respuesta con éxito o mensaje de error.
        """
        # Obtiene la ruta del archivo del parámetro
        file_path = self.get_parameter('file_path').get_parameter_value().string_value

        # Verifica si se especificó el archivo
        if not file_path:
            response.success = False
            response.message = "No se ha especificado ningún archivo."
            return response

        # Inicia la reproducción de la trayectoria
        self.get_logger().info(f"Reproduciendo trayectoria desde: {file_path}")
        self.play_recording(file_path)
        
        response.success = True
        response.message = "Reproducción completada con éxito."
        return response

    def play_recording(self, file_path):
        """
        Lee el archivo de trayectoria y publica los comandos Twist en el tópico /turtlebot_cmdVel.
        Reproduce la secuencia de acciones del robot en base a los datos guardados en el archivo.
        
        Args:
            file_path (str): Ruta del archivo .txt con la secuencia de comandos.
        """
        try:
            # Abre el archivo de trayectoria
            with open(file_path, 'r') as archivo:
                self.get_logger().info("Archivo abierto exitosamente")

                # Lee todas las líneas del archivo
                lines = archivo.readlines()

                # Recorre cada línea del archivo
                for line in lines:
                    # Parseo de la línea para obtener lin_x y ang_z
                    data = line.strip().split(', ')
                    lin_x = float(data[1].split('=')[1])  # Velocidad lineal en X
                    ang_z = float(data[2].split('=')[1])  # Velocidad angular en Z

                    # Espera antes de enviar el siguiente comando
                    time.sleep(0.5)  # Ajusta este tiempo según sea necesario

                    # Publica el comando de velocidad
                    twist_msg = Twist()
                    twist_msg.linear.x = lin_x
                    twist_msg.angular.z = ang_z
                    self.get_logger().info(f"Send: linear_x={lin_x}, angular_z={ang_z}")
                    self.publisher.publish(twist_msg)

                self.get_logger().info("Reproducción terminada")
        except Exception as e:
            # Captura cualquier excepción (como errores de archivo o de formato)
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    """
    Función principal que inicializa el nodo y lo mantiene en ejecución.
    Gestiona la inicialización y cierre del contexto de ROS 2.
    """
    rclpy.init(args=args)  # Inicializa el contexto de ROS 2
    node = TurtleBotPlayerNode()  # Crea la instancia del nodo
    rclpy.spin(node)  # Ejecuta el nodo en modo de espera (spin)
    rclpy.shutdown()  # Finaliza el contexto de ROS 2

if __name__ == '__main__':
    main()  # Llama a la función principal
