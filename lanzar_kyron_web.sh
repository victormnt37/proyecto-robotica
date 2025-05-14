#!/bin/bash

# Arbrir navegador: http://localhost:8000/web-page/

# export TURTLEBOT3_MODEL=burger_pi

# Terminal 1 - Navegación
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
export TURTLEBOT3_MODEL=burger_pi
source install/setup.bash
ros2 launch kyron_nav kyron_nav_v2.launch.py
exec bash"

# Terminal 2 - Esperar 3 minutos y luego cargar mapa + pose inicial
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
echo 'Esperando 3 minutos antes de cargar el mapa...'
sleep 180
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{map_url: $HOME/proyecto-robotica/src/kyron/kyron_nav/config/hospital_world.yaml}'
ros2 run kyron_nav kyron_initial_pose_pub
exec bash"


# Terminal 3 - rosbridge websocket
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
ros2 launch kyron_vision kyron_vision.launch.py
exec bash"

# Terminal 3 - rosbridge websocket
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash"

# Terminal 4 - web_video_server
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
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

# Terminal 7
gnome-terminal -- bash -c "
source install/setup.bash
cd ~/proyecto-robotica/web-page/
python3 app.py
exec bash"