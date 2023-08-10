# Configuración del Agente micro-ROS

Esta guía proporciona instrucciones para configurar el agente micro-ROS en tu entorno de desarrollo.

## Inicialización del entorno ROS2

Asegúrate de tener ROS2 Foxy instalado y configurado:

```bash
source /opt/ros/foxy/setup.bash
```
## Espacio de trabajo para micro-ROS

Crea un directorio para tu espacio de trabajo y clona el repositorio micro_ros_setup dentro del directorio src:

```bash
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
## Instalar dependencias

Actualiza los paquetes y las dependencias de rosdep:

```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```
## Instalar pip

Instala el administrador de paquetes Python pip si aún no está instalado:

```bash

sudo apt-get install python3-pip
```
## Compilar el espacio de trabajo y habilitar el entorno micro-ROS tools

Compila el espacio de trabajo usando colcon y habilita las herramientas de micro-ROS:

```bash

colcon build
```
## Creación de un agente 
Descarga y contruye paquetes del agente micro-ROS
```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
Ejecuta un agente micro-ROS usando Wi-Fi
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

¡Ahora tienes tu entorno de trabajo configurado con micro-ROS listo para su uso!
