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
        self.subscription = self.create_subscription(
            Twist, 'turtlebot_position', self.position_callback, 10
        )
        self.positions_x = []
        self.positions_y = []

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
        # Assume linear.x and linear.y as robot positions for simplicity
        x = msg.linear.x
        y = msg.linear.y
        self.positions_x.append(x)
        self.positions_y.append(y)
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
