# Instalación de Arduino IDE, configuración del parche para usar ESP32 con micro-ROS y ROS2 Foxy

Esta guía proporciona instrucciones paso a paso para instalar y configurar Arduino IDE, así como la configuración necesaria para utilizar ESP32 con micro-ROS y ROS2 Foxy.

# Instalación de Arduino IDE

1. Descarga la versión 1.8.19 de Arduino IDE: [arduino-1.8.19-linux64.tar.xz](https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz)
2. Extrae el paquete descargado:

   ```bash
   tar -xvf arduino-1.8.19-linux64.tar.xz
   ```
# Navega al directorio de Arduino y ejecuta el script de instalación:

```bash
    cd arduino-1.8.19
    sudo sh install.sh
```
Abre el icono del escritorio.
En File → Preferences → Additional Boards Manager URLs, pega el siguiente enlace: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json.
En Tools → Board→ Board Manager, instala la Librería de ESP32 V2.0.0.
Selecciona el hardware adecuado.

# Descarga de la biblioteca micro-ROS para Arduino IDE
Descarga la biblioteca micro-ROS para Arduino IDE desde: micro-ROS Arduino Releases
Extrae el archivo descargado en /home/$USERNAME/Arduino/libraries/. Asegúrate de que la estructura de carpetas se vea como sigue:

    ```bash

    /home/$USERNAME/Arduino/libraries/micro_ros_arduino
      ```

# Inicia el agente micro-ROS

 Navega al directorio de tu espacio de trabajo micro-ROS:

```bash

cd microros_ws
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
```
Lanzar el agente con el ejemplo micro-ros_publisher.ino, usando puerto serial:

```bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
Asegúrate de reemplazar /dev/ttyUSB0 con el puerto serial correcto de tu ESP32.

En caso de requerir permisos, ejecuta:

```bash

sudo chmod 666 /dev/ttyUSB0
```
