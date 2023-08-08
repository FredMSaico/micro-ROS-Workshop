# Guía de Instalación ROS2 Foxy - Host

Esta guía proporciona instrucciones paso a paso para instalar ROS2 Foxy en tu sistema host.

## Configuración local

Verifica que tu sistema esté configurado para UTF-8:

```bash
locale
```

Si no está configurado, sigue los siguientes pasos para configurar UTF-8:

```bash

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Verifica la configuración
```
## Añadir repositorios ROS2

Instala las herramientas necesarias:

```bash

sudo apt install software-properties-common
sudo add-apt-repository universe
```
Añade la clave GPG con apt:

```bash

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Añade el repositorio a la lista de fuentes:

```bash

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
## Instalación ROS2

Actualiza la lista de paquetes e instala ROS2 Foxy:

```bash

sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools
```
## Inicialización del entorno

Inicializa el entorno ROS2:

```bash

source /opt/ros/foxy/setup.bash
```
## Verificar instalación
```bash
rosversion -d
```
¡Ahora estás listo para comenzar a trabajar con ROS2 Foxy!
