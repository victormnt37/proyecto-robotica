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

# Terminal 2 - Cargar mapa del hospital
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
echo 'Esperando 30 segundos antes de cargar el mapa...'
sleep 30
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{map_url: $HOME/proyecto-robotica/src/kyron/kyron_nav/config/hospital_world.yaml}'
ros2 run kyron_nav kyron_initial_pose_pub
exec bash"

# Terminal 3 - kyron vision
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
export ROS_DOMAIN_ID=9
ros2 launch kyron_vision kyron_vision_irl.launch.py
exec bash"

# Terminal 4 - rosbridge websocket
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
export ROS_DOMAIN_ID=9
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
exec bash"

# Terminal 5 - web_video_server
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
source install/setup.bash
export ROS_DOMAIN_ID=9
ros2 run web_video_server web_video_server
exec bash"

# Terminal 6 - Servidor web
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/
python3 -m http.server 8000
exec bash"

# Terminal 7 - Servidor websocket
gnome-terminal -- bash -c "
source install/setup.bash
cd ~/proyecto-robotica/web-page/server
python3 websocket_server.py
exec bash"

# Terminal 8 - servidor backend
gnome-terminal -- bash -c "
cd ~/proyecto-robotica/web-page/
python3 app.py
exec bash"
