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
        super().__init__('turtle_bot_interface')

        # Echo turtlebot_position
        self.subscription = self.create_subscription(
            Twist, 'turtlebot_position', self.position_callback, 10
        )
        
        self.positions_x = []
        self.positions_y = []
        self.trajectory_data = []  # Almacena temporalmente toda la trayectoria
        self.saved_txt = None  # Nombre del archivo a guardar

        self.graph_thread = Thread(target=self.graph_manage)
        self.graph_thread.start()

    def graph_manage(self):
        self.fig, self.ax = plt.subplots()  # Configurar el gráfico
        self.line, = self.ax.plot([], [], 'b-o', label="Trayectoria")  # Línea inicial

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
        btn_save.on_clicked(self.save_path)

        ani = FuncAnimation(self.fig, self.update_graph, interval=500)
        plt.show()

    def save_path(self, event=None):
        """
        Muestra un cuadro de diálogo para elegir el directorio y nombre del archivo.
        Al seleccionarlo, guarda toda la trayectoria acumulada.
        """
        import tkinter as tk
        from tkinter import filedialog
        
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
        # Actualizar los datos de la línea
        self.line.set_data(self.positions_x, self.positions_y)
        # Ajustar automáticamente los límites del gráfico si es necesario
        self.ax.relim()
        self.ax.autoscale_view()
        return self.line,

    def position_callback(self, msg):
        """
        Actualiza los datos de la grafica y guarda los datos en trajectory_data
        """
        x = msg.linear.x
        y = msg.linear.y
        # Data for graph
        self.positions_x.append(x)
        self.positions_y.append(y)
        # Data for internal storage
        self.trajectory_data.append((x, y))  # Almacena toda la trayectoria

        # Data for txt (solo si ya se eligió un archivo)
        if self.saved_txt:
            try:
                with open(self.saved_txt, 'a') as archivo:
                    archivo.write(f'X={x}, Y={y}\n')
            except Exception as e:
                self.get_logger().error(f'Error escribiendo en el archivo: {e}')

        self.get_logger().info(f"Position updated: X={x}, Y={y}")

    def spin_node(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup
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
