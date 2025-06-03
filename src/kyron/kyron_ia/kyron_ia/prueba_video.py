from ultralytics import YOLO

model_path = r'C:\Users\bigli\runs\detect\train6\weights\best.pt'
# Cargar el modelo entrenado
model = YOLO(model_path)

# Desde archivo de vídeo
model.predict(source='test/enfermeras.mp4', show=True)

# Desde webcam (índice 0)
#model.predict(source=0, show=True)
