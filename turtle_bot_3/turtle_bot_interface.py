#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from threading import Thread
import tkinter as tk
from tkinter import filedialog
import time
from std_srvs.srv import Trigger

class TurtleBotInterfaceNode(Node):
    """
    Nodo de interfaz para el TurtleBot.
    Este nodo permite:
    - Visualizar la trayectoria del robot en tiempo real usando Matplotlib.
    - Guardar la secuencia de acciones realizadas por el usuario en un archivo .txt.
    - Reproducir una trayectoria guardada solicitándola desde la interfaz.
    """
    def __init__(self):
        """
        Inicializa el nodo TurtleBotInterfaceNode.
        Suscribe a los tópicos 'turtlebot_position' y '/turtlebot_cmdVel'.
        Crea el cliente para el servicio de reproducción.
        Administra la visualización gráfica y la gestión de archivos.
        """
        super().__init__('turtle_bot_interface')

        # Suscripción al tópico turtlebot_position para visualizar la trayectoria
        self.subscription_position = self.create_subscription(
            Twist, 'turtlebot_position', self.position_callback, 10
        )

        # Suscripción al tópico turtlebot_cmdVel para guardar la secuencia de acciones
        self.subscription_velocity = self.create_subscription(
            Twist, '/turtlebot_cmdVel', self.cmdVel_callback, 10
        )

        # Cliente para llamar al servicio de reproducción en turtle_bot_player
        self.client_player = self.create_client(Trigger, 'play_recording')

        # Almacenamiento de datos de trayectoria para la gráfica y archivo
        self.positions_x = []  # Coordenadas X para el gráfico
        self.positions_y = []  # Coordenadas Y para el gráfico
        self.trajectory_data = []  # Almacena toda la trayectoria
        self.saved_txt = None  # Ruta del archivo donde se guarda la trayectoria

        # Pregunta inicial para reproducir archivo guardado
        self.replaying = self.ask_play_trajectory()  # Pregunta si desea reproducir una trayectoria guardada

        # Pregunta inicial para guardar archivo solo si no se está reproduciendo una trayectoria
        if not self.replaying:
            self.ask_save_initial()  # Pregunta si desea guardar el archivo al iniciar

        # Hilo para gestionar la gráfica sin bloquear el nodo ROS
        self.graph_thread = Thread(target=self.graph_manage)
        self.graph_thread.start()

    def ask_play_trajectory(self):
        """
        Pregunta al usuario si desea reproducir una trayectoria.
        Si la respuesta es sí, solicita el archivo .txt y llama al servicio de reproducción.
        
        Returns:
            bool: True si se va a reproducir una trayectoria, False en caso contrario.
        """
        root = tk.Tk()
        root.withdraw()  # Oculta la ventana principal de Tkinter
        
        # Pregunta al usuario si quiere reproducir una trayectoria
        play_decision = tk.messagebox.askyesno("Reproducir Recorrido", "¿Desea reproducir un recorrido guardado?")

        if play_decision:
            # Solicita el archivo .txt con la trayectoria guardada
            file_path = filedialog.askopenfilename(
                title="Seleccionar Recorrido Guardado",
                filetypes=[("Archivo de texto", "*.txt")]
            )
            if file_path:
                self.get_logger().info(f"Reproduciendo recorrido desde: {file_path}")
                self.set_file_path_param(file_path)
                self.call_player_service()
                return True
            else:
                self.get_logger().info("No se seleccionó ningún archivo.")
                return False
        else:
            return False
        
    def set_file_path_param(self, file_path):
        """
        Configura el parámetro 'file_path' en turtle_bot_player.
        Utiliza un comando del sistema para enviar el parámetro a través de ROS 2.
        
        Args:
            file_path (str): Ruta del archivo de trayectoria a reproducir.
        """
        self.get_logger().info("Estableciendo ruta del archivo en turtle_bot_player...")
        set_param_cmd = f"ros2 param set /turtle_bot_player file_path '{file_path}'"
        self.get_logger().info(set_param_cmd)  # Muestra el comando ejecutado
        os.system(set_param_cmd)  # Ejecuta el comando en la terminal

    def call_player_service(self):
        """
        Llama al servicio en turtle_bot_player para iniciar la reproducción.
        Utiliza el servicio Trigger para activar la reproducción de la trayectoria.
        """
        while not self.client_player.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio de reproducción esté disponible...')

        # Crea la solicitud de Trigger
        request = Trigger.Request()
        future = self.client_player.call_async(request)

        # Callback para manejar la respuesta del servicio
        def callback(future):
            try:
                response = future.result()
                self.get_logger().info(f"Respuesta del servicio: {response.message}")
            except Exception as e:
                self.get_logger().error(f"Servicio falló: {e}")

        future.add_done_callback(callback)

    def ask_save_initial(self):
        """
        Pregunta al usuario si desea guardar el archivo al iniciar el nodo.
        Si la respuesta es sí, pregunta el nombre del archivo y el directorio donde guardarlo.
        """
        root = tk.Tk()
        root.withdraw()  # Oculta la ventana principal de Tkinter
        
        # Pregunta al usuario si quiere guardar la trayectoria
        save_decision = tk.messagebox.askyesno("Guardar Recorrido", "¿Desea guardar el recorrido del robot?")

        if save_decision:
            # Solicita el nombre y directorio del archivo .txt para guardar la trayectoria
            self.saved_txt = filedialog.asksaveasfilename(
                title="Guardar Recorrido del TurtleBot",
                defaultextension=".txt",
                filetypes=[("Archivo de texto", "*.txt")]
            )
            if self.saved_txt:
                self.get_logger().info(f"Se guardará el recorrido en: {self.saved_txt}")
                self.initial_time = time.time()  # Inicia a tomar el tiempo de partida
            else:
                self.get_logger().info("No se especificó archivo. No se guardará el recorrido.")

    def graph_manage(self):
        """
        Gestiona la visualización de la trayectoria en tiempo real usando Matplotlib.
        Se ejecuta en un hilo separado para no bloquear el nodo ROS.
        """
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-o', label="Trayectoria")

        # Configuración de la gráfica
        self.ax.set_title("Turtlebot Position")
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid()

        ani = FuncAnimation(self.fig, self.update_graph, interval=500)  
        plt.show()

    def update_graph(self, frame):
        """
        Actualiza el gráfico en tiempo real con las posiciones acumuladas.
        """
        self.line.set_data(self.positions_x, self.positions_y)  
        self.ax.relim()  
        self.ax.autoscale_view()  
        return self.line,

    def position_callback(self, msg):
        """
        Callback que actualiza las coordenadas del gráfico con la posición del TurtleBot.
        
        Args:
            msg (Twist): Mensaje con la posición actual del TurtleBot.
        """
        x = msg.linear.x
        y = msg.linear.y
        self.positions_x.append(x)
        self.positions_y.append(y)
        self.trajectory_data.append((x, y))
        self.get_logger().info(f"Position updated: X={x}, Y={y}")

    def cmdVel_callback(self, msg):
        """
        Callback que guarda la secuencia de acciones realizadas por el usuario.
        """
        if self.saved_txt:
            lin_x = msg.linear.x
            ang_z = msg.angular.z
            elapsed = round(time.time() - self.initial_time, 3)
            try:
                with open(self.saved_txt, 'a') as archivo:
                    archivo.write(f't={elapsed}, lin_x={lin_x}, ang_z={ang_z}\n')
            except Exception as e:
                self.get_logger().error(f'Error escribiendo en el archivo: {e}')

    def spin_node(self):
        """
        Mantiene el nodo en ejecución hasta que se interrumpe con Ctrl+C.
        Maneja el cierre del nodo de manera segura.
        """
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass  # Permite salir del nodo con Ctrl+C
        finally:
            plt.close()  # Cierra la ventana de la gráfica
            self.destroy_node()  # Destruye el nodo ROS
            if rclpy.ok():
                rclpy.shutdown()  # Finaliza el contexto de ROS 2

def main(args=None):
    """
    Función principal que inicializa el nodo y lo mantiene en ejecución.
    Gestiona la inicialización y cierre del contexto de ROS 2.
    """
    rclpy.init(args=args)  # Inicializa el contexto de ROS 2
    node = TurtleBotInterfaceNode()  # Crea la instancia del nodo
    node.spin_node()  # Ejecuta el nodo en modo de espera (spin)

if __name__ == '__main__':
    main()  # Llama a la función principal
