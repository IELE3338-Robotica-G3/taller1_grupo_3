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


class TurtleBotInterfaceNode(Node):
    def __init__(self):
        """
        Inicializa el nodo TurtleBotInterfaceNode.
        Crea una suscripción al tópico 'turtlebot_position' y prepara el manejo de gráficos.
        """
        super().__init__('turtle_bot_interface')

        # Suscripción al tópico turtlebot_position
        self.subscription = self.create_subscription(
            Twist, 'turtlebot_position', self.position_callback, 10
        )
        
        # Almacenamiento de datos de trayectoria
        self.positions_x = []  # Coordenadas X para el gráfico
        self.positions_y = []  # Coordenadas Y para el gráfico
        self.trajectory_data = []  # Almacena toda la trayectoria
        self.saved_txt = None  # Ruta del archivo donde se guarda la trayectoria

        # Hilo para gestionar la gráfica sin bloquear el nodo ROS
        self.graph_thread = Thread(target=self.graph_manage)
        self.graph_thread.start()

        # TODO 0: Guardar la secuencia de acciones que realizó el usuario durante el recorrido del robo
        # TODO 1: Create a button that asks if the user wants to save de path and assign a name
        # TODO 2: Asks the user the directory path to save the previous file
        # TODO 3: create a subscription to a service 

    def graph_manage(self):
        """
        Gestiona la configuración y actualización del gráfico de trayectoria.
        Ejecutado en un hilo separado para no bloquear el nodo.
        """
        self.fig, self.ax = plt.subplots()  # Configurar el gráfico
        self.line, = self.ax.plot([], [], 'b-o', label="Trayectoria")  # Línea inicial

        # Configuración del gráfico
        self.ax.set_title("Turtlebot Position")
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid()

        # Botón para guardar recorrido
        ax_button = plt.axes([0.8, 0.02, 0.15, 0.05])  # [left, bottom, width, height]
        btn_save = Button(ax_button, 'Save route')
        btn_save.on_clicked(self.save_path)  # Asocia el botón a save_path()

        ani = FuncAnimation(self.fig, self.update_graph, interval=500)  # Actualiza la gráfica
        plt.show()  # Muestra la ventana del gráfico

    def save_path(self, event=None):
        """
        Muestra un cuadro de diálogo para elegir el directorio y nombre del archivo.
        Al seleccionarlo, guarda toda la trayectoria acumulada.
        """
        root = tk.Tk()
        root.withdraw()  # Oculta la ventana principal de Tkinter
        
        # Abre el cuadro de diálogo para guardar el archivo
        file_path = filedialog.asksaveasfilename(
            title="Guardar Recorrido del TurtleBot",
            defaultextension=".txt",
            filetypes=[("Archivo de texto", "*.txt")]
        )
        
        if file_path:  # Si el usuario selecciona una ruta
            self.saved_txt = file_path
            self.get_logger().info(f"Se guardará el recorrido en: {self.saved_txt}")
            
            # Guardar toda la trayectoria acumulada hasta ahora
            try:
                with open(self.saved_txt, 'w') as archivo:
                    for x, y in self.trajectory_data:
                        archivo.write(f'X={x}, Y={y}\n')
                self.get_logger().info("Trayectoria acumulada guardada correctamente.")
            except Exception as e:
                self.get_logger().error(f'Error escribiendo la trayectoria acumulada: {e}')
        else:  # Si cancela el diálogo
            self.get_logger().info("No se guardará el recorrido.")
            self.saved_txt = None

    def update_graph(self, frame):
        """
        Actualiza el gráfico en tiempo real con las posiciones acumuladas.
        """
        self.line.set_data(self.positions_x, self.positions_y)  # Actualiza la línea
        self.ax.relim()  # Recalcula los límites del gráfico
        self.ax.autoscale_view()  # Ajusta la vista automáticamente
        return self.line,

    def position_callback(self, msg):
        """
        Callback que se ejecuta al recibir un mensaje de 'turtlebot_position'.
        Actualiza las coordenadas y guarda la trayectoria completa.
        """
        x = msg.linear.x
        y = msg.linear.y
        # Datos para el gráfico en tiempo real
        self.positions_x.append(x)
        self.positions_y.append(y)
        # Almacenamiento interno de la trayectoria completa
        self.trajectory_data.append((x, y))

        # Guardado en archivo (si ya se seleccionó uno)
        if self.saved_txt:
            try:
                with open(self.saved_txt, 'a') as archivo:
                    archivo.write(f'X={x}, Y={y}\n')
            except Exception as e:
                self.get_logger().error(f'Error escribiendo en el archivo: {e}')

        self.get_logger().info(f"Position updated: X={x}, Y={y}")

    def spin_node(self):
        """
        Mantiene el nodo en ejecución hasta que se interrumpe con Ctrl+C.
        Maneja el cierre de manera segura.
        """
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close()  # Cierra la ventana de la gráfica
            self.destroy_node()  # Destruye el nodo ROS
            if rclpy.ok():
                rclpy.shutdown()  # Apaga rclpy correctamente

def main(args=None):
    """
    Función principal que inicializa el nodo y lo ejecuta.
    """
    rclpy.init()
    node = TurtleBotInterfaceNode()
    node.spin_node()

if __name__ == '__main__':
    main()