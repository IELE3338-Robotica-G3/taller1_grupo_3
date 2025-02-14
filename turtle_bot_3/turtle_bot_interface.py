#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from threading import Thread
import tkinter as tk
from tkinter import filedialog
import time

class TurtleBotInterfaceNode(Node):
    def __init__(self):
        """
        Inicializa el nodo TurtleBotInterfaceNode.
        Crea una suscripción al tópico 'turtlebot_position' y prepara el manejo de gráficos.
        """
        super().__init__('turtle_bot_interface')

        # Suscripción al tópico turtlebot_position
        self.subscription_position = self.create_subscription(
            Twist, 'turtlebot_position', self.position_callback, 10
        )

        # Suscripción al tópico turtlebot_cmdVel
        self.subscription_velocity = self.create_subscription(
            Twist, '/turtlebot_cmdVel', self.cmdVel_callback, 10
        )
        
        # Almacenamiento de datos de trayectoria
        self.positions_x = []  # Coordenadas X para el gráfico
        self.positions_y = []  # Coordenadas Y para el gráfico
        self.trajectory_data = []  # Almacena toda la trayectoria
        self.saved_txt = None  # Ruta del archivo donde se guarda la trayectoria

        # Pregunta inicial para guardar archivo
        self.ask_save_initial()  # Pregunta si desea guardar el archivo al iniciar

        # Hilo para gestionar la gráfica sin bloquear el nodo ROS
        self.graph_thread = Thread(target=self.graph_manage)
        self.graph_thread.start()

    def ask_save_initial(self):
        """
        Pregunta al usuario si desea guardar el archivo al iniciar el nodo.
        Si la respuesta es sí, pregunta el nombre del archivo y el directorio.
        """
        root = tk.Tk()
        root.withdraw()  # Oculta la ventana principal de Tkinter
        save_decision = tk.messagebox.askyesno("Guardar Recorrido", "¿Desea guardar el recorrido del robot?")

        if save_decision:
            self.saved_txt = filedialog.asksaveasfilename(
                title="Guardar Recorrido del TurtleBot",
                defaultextension=".txt",
                filetypes=[("Archivo de texto", "*.txt")]
            )
            if self.saved_txt:
                self.get_logger().info(f"Se guardará el recorrido en: {self.saved_txt}")
                self.initial_time = time.time() # Inicia a tomar el tiempo de partida
            else:
                self.get_logger().info("No se especificó archivo. No se guardará el recorrido.")

    def graph_manage(self):
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-o', label="Trayectoria")

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
        self.line.set_data(self.positions_x, self.positions_y)  
        self.ax.relim()  
        self.ax.autoscale_view()  
        return self.line,

    def position_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        self.positions_x.append(x)
        self.positions_y.append(y)
        self.trajectory_data.append((x, y))
        self.get_logger().info(f"Position updated: X={x}, Y={y}")

    def cmdVel_callback(self, msg):
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
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close()  
            self.destroy_node()  
            if rclpy.ok():
                rclpy.shutdown()  

def main(args=None):
    rclpy.init()
    node = TurtleBotInterfaceNode()
    node.spin_node()

if __name__ == '__main__':
    main()
