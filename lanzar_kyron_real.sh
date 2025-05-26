#!/bin/bash

# Lanzador para el robot real.

# Arbrir navegador: http://localhost:8000/web-page/

# Terminal 1 - Navegación
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
export TURTLEBOT3_MODEL=burger_pi
source install/setup.bash
ros2 launch kyron_nav kyron_nav_real.launch.py
exec bash"

# Terminal 2 - kyron vision
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
export ROS_DOMAIN_ID=9
ros2 launch kyron_vision kyron_vision_irl.launch.py
exec bash"

# Terminal 3 - rosbridge websocket
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
export ROS_DOMAIN_ID=9
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash"

# Terminal 4 - web_video_server
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
export ROS_DOMAIN_ID=9
ros2 run web_video_server web_video_server
exec bash"

# Terminal 5 - Servidor web
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
python3 -m http.server 8000
exec bash"

# Terminal 6 - Servidor websocket
gnome-terminal -- bash -c "
source install/setup.bash
cd ~/proyecto-robotica/web-page/server
python3 websocket_server.py
exec bash"

# Terminal 7 - servidor backend
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
cd ~/web-page/
python3 app.py
exec bash"

# lanzar camara