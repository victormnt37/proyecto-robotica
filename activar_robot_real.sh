#!/bin/bash

# Configuración
ROBOT_IP="192.168.0.136"
USER="ubuntu"
PASSWORD="turtlebot"
LAUNCH_COMMAND="ros2 launch turtlebot3_bringup robot.launch.py"
CAMERA_COMMAND="ros2 run image_tools cam2image --ros-args -p burger_mode:=false -p frequency:=10.0 -p reliability:=best_effort"

# Función para abrir pestañas con SSH
function open_ssh_tabs() {
    gnome-terminal --window --title="Robot Control" -- bash -ic "\
        sshpass -p '$PASSWORD' ssh -t $USER@$ROBOT_IP '$LAUNCH_COMMAND'; \
        exec bash" \
    --tab --title="Cámara" -- bash -ic "\
        sleep 5; \
        sshpass -p '$PASSWORD' ssh -t $USER@$ROBOT_IP '$CAMERA_COMMAND'; \
        exec bash" \
    --tab --title="SSH Extra" -- bash -ic "\
        sshpass -p '$PASSWORD' ssh -t $USER@$ROBOT_IP; \
        exec bash"
}

# Instalar sshpass si no está presente
if ! command -v sshpass &> /dev/null; then
    echo "Instalando sshpass..."
    sudo apt-get install -y sshpass
fi

# Ejecutar el proceso completo
open_ssh_tabs

echo "Proceso iniciado en pestañas separadas!"