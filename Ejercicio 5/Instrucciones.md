# Crea un nuevo nodo de teleoperación en ROS 2
En el espacio de trabajo microros_ws, clona y compila el repositorio de teleoperación
```bash
source /opt/ros/foxy/setup.bash
cd microros_ws/src
git clone https://github.com/tonynajjar/keyboard_teleop
cd ..
rosdep install --from-paths src --ignore-src -r
colcon build
```
# Ejecuta el nodo y revisa el tipo de mesaje que envía
```bash

ros2 run keyboard_teleop keyboard_teleop_hold
ros2 topic echo /cmd_vel
```
# Usando el archivo microros_template, crea un nodo subscriptor y controla un robot diferencial
