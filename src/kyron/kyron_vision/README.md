# Uso del Paquete kyron_vision

Este paquete es el que se encarga de la vision del robot para que pueda reconocer caras, cuerpos, y formas.

**id_cara** : nodo que se usa para identificar y reconocer  caras.

**id_cuerpo** : nodo que se usa para identificar y reconocer cuerpos.

## Mundo de prueba

SI quieres lanzar el mundo especifico para probarlo:

```
ros2 launch kyron_mundo vision_test_world.launch.py 

```

## Launch

```bash
#Este launch si quieres lanzar el del robot real.
ros2 launch kyron_vision kyron_vision_irl.launch.py

#Este launch si quieres usar el de gazebo
ros2 launch kyron_vision kyron_vision.launch.py

# Si quieres lanzar por separado cada nodo:

#ROBOT REAL
ros2 run kyron_vision id_cara --ros-args -p modo:=irl 
ros2 run kyron_vision id_cuerpo --ros-args -p modo:=irl

#SIMULACION
ros2 run kyron_vision id_cara --ros-args -p modo:=  sim
ros2 run kyron_vision id_cuerpo --ros-args -p modo:= sim


```
