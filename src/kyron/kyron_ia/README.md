# Kyron IA

En este paquete se guardan los modelos , datasets y todo lo necesario para que funcione la IA del robot. Tambien el script que ejecuta el nodo de comandos de voz.

# Librerias necesarias

```bash
# Para crear tus propios lables (Solo necesario si necesitas crear tu proprio dataset)
pip install label-studio
label-studio start#ejecutar label studio

#Pytorch
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

#Para yolo
pip install ultralytics
```

## Documentaciòn de las librerias utilizadas

**YOLO**:

* [Yolo y ros](https://docs.ultralytics.com/es/guides/ros-quickstart/#ros-messages-and-topics)
* [Yolo docu](https://docs.ultralytics.com/#where-to-start)
* [Yolo github](https://github.com/ultralytics/ultralytics)

Speech recognition:

* [Web de biblioteca ](https://pypi.org/project/SpeechRecognition/)

# Guia de Uso

Los scripts de prueba + train no se usan directamente aqui. Estan puestos solo para que se pueda ver su codigo. Se usa mas para guardar los modelos   y usarlos en otros paquetes.

``voicecmd`` si se ejecuta, que es un nodo que transcribe lo que se hable a un mic y basado ene l texto extraido hace algo. Solo ejecuta:

```ros2 run kyron_ia voicecmd```
