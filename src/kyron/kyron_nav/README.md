# Kyron_nav

Paquete que se encarga de la navegacion del robot.


## Como usar este paquete

### Terminal 1

** Lanza todo lo necesario para simular (gazebo,rviz,y nodos de nav2)**
cd proyecto-robotica/
source install/setup.bash
ros2 launch kyron_nav kyron_nav_sim.launch.py 


### Terminal 2

** Carga el mapa (Primero hacer esto) **

ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{
  map_url: '$(ros2 pkg prefix kyron_nav)/share/kyron_nav/config/hospital_world.yaml'
}"

**Publica la posicion inicial del robot **
ros2 run kyron_nav kyron_initial_pose_pub

** Instruccion de mover a destino con coordenadas **

ros2 run kyron_nav kyron_goal_pub 1.7 13.0 0.003

** Ejecuta el waypoint follower (Nota, este no va por alguna razon con los waypoints que deberia, pero si con unos al azar que he puesto) **

ros2 run kyron_nav kyron_nav_wf


