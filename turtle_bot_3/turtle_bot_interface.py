import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from threading import Thread

class TurtleBotInterfaceNode(Node):
    def __init__(self):
        super().__init__('turtle_bot_interface')
        
        # Asks if it wants to save path
        self.saved_txt = ""
        self.save_path()

        # Echo turtlebot_position
        self.subscription = self.create_subscription(
            Twist, 'turtlebot_position', self.position_callback, 10
        )
        self.positions_x = []
        self.positions_y = []

        self.graph_thread = Thread(target=self.graph_manage)
        self.graph_thread.start()
        
        # TODO 0: Guardar la secuencia de acciones que realizó el usuario durante el recorrido del robo
        # TODO 1: Create a button that asks if the user wants to save de path and assign a name
        # TODO 2: Asks the user the directory path to save the previous file
        # TODO 3: create a subscription to a service 
        
    def save_path(self):
        """
        Asks the user if it wants to save the path of the robot
        """
        respuesta = input("Desea guardar el recorrido del turtle_bot_3? (Y/n):").strip().lower()
        
        invalid = True
        while invalid:
            if respuesta == 'y':
                self.get_logger().info("Se guardara el recorrido ...")
                self.saved_txt = input("Digite el nombre del archivo (sin la extension):") + ".txt"
                invalid = False
            elif respuesta == 'n':
                self.get_logger().info("No se guardara el recorrido ...")
                invalid = False
            else:
                respuesta = input("Desea guardar el recorrido del turtle_bot_3? (Y/n):").strip().lower()

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

        time.sleep(0.5)
        ani = FuncAnimation(self.fig, self.update_graph, interval=500)
        plt.show()

    def update_graph(self, frame):
        # Actualizar los datos de la línea
        self.line.set_data(self.positions_x, self.positions_y)
        # Ajustar automáticamente los límites del gráfico si es necesario
        self.ax.relim()
        self.ax.autoscale_view()
        return self.line,

    def position_callback(self, msg):
        """
        Actualiza los datos de la grafica y guardarlos en el archivo .txt
        """
        x = msg.linear.x
        y = msg.linear.y
        # Data for graph
        self.positions_x.append(x)
        self.positions_y.append(y)
        # Data for txt
        try:
            with open(self.saved_txt, 'a') as archivo:
                archivo.write(f'{msg}\n')  # Guarda los datos como una nueva línea
            #self.get_logger().info(f'Datos guardados: {msg}') #TODO: Cambiar msg por x, y
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
            rclpy.shutdown()

def main(args=None):
    rclpy.init()
    node = TurtleBotInterfaceNode()
    node.spin_node()

if __name__ == '__main__':
    main()
