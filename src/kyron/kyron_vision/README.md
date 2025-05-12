# Uso del Paquete kyron_vision


Este paquete es el que se encarga de la vision del robot para que pueda reconocer caras, cuerpos, y formas.

**id_cara** : nodo que se usa para identificar y reconocer  caras.

**id_cuerpo** : nodo que se usa para identificar y reconocer cuerpos.

### Mundo de prueba

SI quieres lanzar el mundo especifico para probarlo:

```
ros2 launch kyron_mundo vision_test_world.launch.py 

```

### Activar camara del robot real.

#### Conexion

>ADVERTENCIA:
> Asegurate que esten los dos (PC Y ROBOT) esten en el domain correcto

```bash

echo  $ROS_DOMAIN_ID

export ROS_DOMAIN_ID=X # donde X es vuestro número de equipo

```


En la terminal ssh:
```bash
ssh ubuntu@IP_DEL_ROBOT

ros2 launch turtlebot3_bringup robot.launch.py
```

#### Activar camara

[Terminal 1]
```bash
ros2 run image_tools cam2image --ros-args -p burger_mode:=false -p frequency:=10.0 -p reliability:=best_effort 


```
[Terminal 2] (opcional)
Este es solo para visualizar la camara(los nodos ya lo hace)
```bash
ros2 run image_tools showimage --ros-args -p show_image:=true -p reliability:=best_effort 
```
---



## Launch del paquete

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
### Listeners
Hay unos listeners que fueron creados de prueba pero lo pueden usar si quieren ver que mensajes reciben
```bash

#Listener de id cara
 ros2 run kyron_vision listener

#LIstener de id cuerpo
  ros2 run kyron_vision listener_cuerpo

```
