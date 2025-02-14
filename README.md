# turtle_bot_3

Este paquete ROS 2 permite controlar y visualizar la trayectoria de un TurtleBot. Incluye dos nodos:

- **turtle_bot_interface**: Visualiza la trayectoria en tiempo real y permite guardar los recorridos.
- **turtle_bot_teleop**: Controla el TurtleBot mediante el teclado.

## Requisitos

Este proyecto requiere ROS 2 Iron o superior y Python 3.x. Además, utiliza las siguientes bibliotecas:
- `matplotlib`: Para la visualización de gráficos.
- `tkinter`: Para la selección de archivos (normalmente ya incluido en Python, pero puede requerir instalación en algunas distribuciones de Linux).

Instala las dependencias con:
```bash
sudo apt update
sudo apt install python3-tk
pip install matplotlib
```

## Instalación

1. Clona el repositorio en tu espacio de trabajo de ROS 2:
```bash
cd ~/ros2_ws/src
git clone https://github.com/IELE3338-Robotica-G3/taller1_grupo_3.git
```

2. Compila el paquete:
```bash
cd ~/ros2_ws
colcon build --packages-select turtle_bot_3
source install/setup.bash
```

## Nodos

### 1. turtle_bot_interface
Este nodo permite visualizar en tiempo real la trayectoria del TurtleBot y guardar el recorrido.

- **Funcionalidades:**
  - Muestra la trayectoria del TurtleBot en un gráfico en tiempo real.
  - Ofrece un botón para guardar el recorrido en un archivo de texto.
- **Ejecutar el nodo:**
```bash
ros2 run turtle_bot_3 turtle_bot_interface
```

### 2. turtle_bot_teleop
Este nodo permite controlar el TurtleBot mediante el teclado.

- **Controles del teclado:**
  - `W`: Adelante
  - `S`: Atrás
  - `A`: Girar a la izquierda
  - `D`: Girar a la derecha
  - `Espacio`: Detener el robot
- **Ejecutar el nodo:**
```bash
ros2 run turtle_bot_3 turtle_bot_teleop
```

## Ejecución

1. Abre una terminal y ejecuta el nodo de teleoperación:
```bash
ros2 run turtle_bot_3 turtle_bot_teleop
```

2. En otra terminal, ejecuta el nodo de interfaz para visualizar la trayectoria:
```bash
ros2 run turtle_bot_3 turtle_bot_interface
```
