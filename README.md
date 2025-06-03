# Kyron

Robot de  patrullaje hospitalario, pensado para apoyar en la logística (transporte de medicamentos) y asistir a pacientes con información sobre el hospital(dirección de consulta, siguiente cita libre, etc.)

## Dependencias

```
#servidor 
pip install flask

# Reconocimiento de caras
pip3 install face_recognition

# Procesamiento de imagenes
pip3 install opencv-python

#=====================
# Modelos IA
#=====================
# Para crear tus propios lables (Solo necesario si necesitas crear tu proprio dataset)
pip install label-studio
label-studio start#ejecutar label studio


#Pytorch
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

#Para yolo
pip install ultralytics

# Reconocimiento de voz
pip install SpeechRecognition

# Mic
pip install pyaudio

#Si no va ese intentar con este
sudo apt install python3-pyaudio



```

## Ejecuciòn

```
# Viene con archivos automatizados
cd proyecto-robotica

#Lanze dependiendo de lo que quiera
activar_robot_real.sh
lanzar_kyron_real.sh
lanzar_kyron_web.sh
```

## Activar camara del robot real.

### Conexion

> ADVERTENCIA:
> Asegurate que esten los dos (PC Y ROBOT) esten en el domain correcto

```bash

echo  $ROS_DOMAIN_ID

export ROS_DOMAIN_ID=X # donde X es vuestro número de equipo

```

En la terminal ssh:

```bash
ssh ubuntu@192.168.0.134

ros2 launch turtlebot3_bringup robot.launch.py
```

### Activar camara

[Terminal 1]

```bash
ssh ubuntu@192.168.0.134
ros2 run image_tools cam2image --ros-args -p burger_mode:=false -p frequency:=10.0 -p reliability:=best_effort 


```

[Terminal 2] (opcional)
Este es solo para visualizar la camara(los nodos ya lo hace)

```bash
ros2 run image_tools showimage --ros-args -p show_image:=true -p reliability:=best_effort 
```

---

# Paquetes

Cada paquete tiene un read me con sus instrucciones de lanzamiento

* **kyron_vision**: se encarga de procesar las imagenes capturadas por el robot(lo que ve).
* **kyron_ia**: tiene los modelos que se utilizan en el el procesamiento de imagenes.
* kyron_nav: se encarga de la navegaciòn del robot.
* **kyron_mundo**: se encarga  de lanzar el mundo simulado en gazebo.
