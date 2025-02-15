# Turtle Bot 3

Este paquete permite controlar el TurtleBot en CoppeliaSim usando ROS 2. Incluye tres nodos principales:
- `turtle_bot_teleop`: Control del TurtleBot mediante el teclado.
- `turtle_bot_interface`: Visualización en tiempo real de la trayectoria y gestión de guardado de secuencias.
- `turtle_bot_player`: Reproducción de trayectorias guardadas.

---

## 📖 **Tabla de Contenidos**
- [Descripción](#descripción)
- [Nodos](#nodos)
  - [turtle_bot_teleop](#turtle_bot_teleop)
  - [turtle_bot_interface](#turtle_bot_interface)
  - [turtle_bot_player](#turtle_bot_player)
- [Servicios](#servicios)
- [Instalación](#instalación)
  - [Requisitos](#requisitos)
  - [Dependencias](#dependencias)
  - [Clonación e Instalación](#clonación-e-instalación)
- [Ejecución](#ejecución)
- [Colaboradores](#colaboradores)
- [Repositorio](#repositorio)

---

## Descripción
Este paquete permite:
- Controlar el TurtleBot con el teclado.
- Visualizar en tiempo real la trayectoria del robot.
- Guardar una grafica de la trayectoria del robot en un archivo `.png`.
- Guardar la secuencia de acciones realizadas en un archivo `.txt`.
- Reproducir una trayectoria guardada desde el archivo.

---

## Nodos

### `turtle_bot_teleop`
Nodo de teleoperación para controlar el TurtleBot usando el teclado.
- Solicita al usuario la velocidad lineal y angular al iniciar.
- Publica comandos de velocidad en el tópico `/turtlebot_cmdVel`.
- Control de movimiento:
  - `W`: Adelante
  - `S`: Atrás
  - `A`: Girar a la izquierda
  - `D`: Girar a la derecha
  - `Otra tecla (o soltar las teclas)`: Detiene el robot

**Publica en:**
- `/turtlebot_cmdVel` (`geometry_msgs/Twist`): Comandos de velocidad lineal y angular.

---

### `turtle_bot_interface`
Nodo de interfaz para visualizar y gestionar la trayectoria del TurtleBot.
- Muestra la trayectoria en tiempo real usando Matplotlib.
- Pregunta al usuario si desea:
  - Guardar la secuencia de acciones en un archivo `.txt`.
  - Reproducir una trayectoria guardada.
- Llama al servicio en `turtle_bot_player` para reproducir una trayectoria.

**Publica en:**
- Ningún tópico propio, pero escucha `/turtlebot_cmdVel` y `/turtlebot_position` para registrar acciones.

**Suscribe a:**
- `turtlebot_position` (`geometry_msgs/Twist`): Para actualizar la gráfica en tiempo real.
- `/turtlebot_cmdVel` (`geometry_msgs/Twist`): Para guardar la secuencia de acciones.

**Servicios Utilizados:**
- `play_recording` (`std_srvs/Trigger`): Llama al nodo `turtle_bot_player` para iniciar la reproducción.

**Otras Funcionalidades**
- Puede guardar un archivo `.png` de la trayectoria actual en el directorio deseado pulsando el botón de guardar en la interfaz gráfica.

---

### `turtle_bot_player`
Nodo que reproduce una trayectoria guardada para el TurtleBot2.
- Lee un archivo `.txt` con la secuencia de acciones.
- Publica comandos de velocidad para reproducir la trayectoria.

**Publica en:**
- `/turtlebot_cmdVel` (`geometry_msgs/Twist`): Comandos de velocidad lineal y angular.

**Servicios Ofrecidos:**
- `play_recording` (`std_srvs/Trigger`): Inicia la reproducción de la trayectoria desde el archivo.

**Uso del Parámetro:**
- `file_path`: Ruta del archivo `.txt` a reproducir. Este parámetro es configurado por `turtle_bot_interface`.

---

## Servicios

### `play_recording`
- **Tipo:** `std_srvs/Trigger`
- **Descripción:** Inicia la reproducción de una trayectoria guardada.
- **Proveedor:** Nodo `turtle_bot_player`.
- **Cliente:** Nodo `turtle_bot_interface`.
- **Uso:**
  ```bash
  ros2 service call /play_recording std_srvs/srv/Trigger
  ```

---

## Instalación

### Requisitos
- **Ubuntu 22.04 LTS**
- **ROS 2 Humble**
- **Python 3.10 o superior**

---

### Dependencias
Asegúrate de tener instaladas las siguientes dependencias:

```bash
sudo apt update
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs python3-matplotlib python3-tk
```

- `rclpy`: Comunicación ROS 2 en Python.
- `geometry_msgs`: Mensajes de geometría para comandos de velocidad.
- `matplotlib`: Visualización en tiempo real de la trayectoria.
- `tkinter`: Interfaz gráfica para ventanas emergentes.

---

### Clonación e Instalación
1. Clona el repositorio en tu espacio de trabajo de ROS 2:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/IELE3338-Robotica-G3/taller1_grupo_3.git
    ```

2. Compila el paquete:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select turtle_bot_3
    ```

3. Fuente al entorno:
    ```bash
    source install/setup.bash
    ```

---

## Ejecución

### 1. **Nodo de Teleoperación**
```bash
ros2 run turtle_bot_3 turtle_bot_teleop
```

### 2. **Nodo de Reproducción**
```bash
ros2 run turtle_bot_3 turtle_bot_player
```

### 3. **Nodo de Interfaz**
```bash
ros2 run turtle_bot_3 turtle_bot_interface
```

---

## Colaboradores
- **Alan Villa** (a.villap@uniandes.edu.co)
- **Jaime Rueda** (j.ruedap@uniandes.edu.co)
- **Jesus Sandoval** (je.sandovals1@uniandes.edu.co)
- **Leonardo Sawamoto** (l.sawamoto@uniandes.edu.co)

---

## Repositorio
[Turtle Bot 3 - IELE3338-Robotica-G3](https://github.com/IELE3338-Robotica-G3/taller1_grupo_3)

