from ultralytics import YOLO

model= YOLO("yolo11m.pt")


model.train(data ="data.yaml",imgsz=640,epochs=85, workers=0 ,device=0,project=r"C:\Users\bigli\Desktop\Programacion y Codigo\YOLO\Modelo_2\model")