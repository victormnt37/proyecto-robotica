from ultralytics import YOLO
import cv2

# Ruta al modelo entrenado
model = YOLO(r'model/train/weights/best.pt')

# Abrir webcam (índice 0 es la webcam por defecto)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Ejecutar inferencia en el frame
    results = model(frame)

    # Dibujar resultados sobre el frame
    annotated_frame = results[0].plot()

    # Mostrar el frame con anotaciones
    cv2.imshow('YOLO Webcam', annotated_frame)

    # Salir con la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
